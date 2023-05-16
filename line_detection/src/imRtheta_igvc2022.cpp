#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>

using namespace std;


class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	chrono::system_clock::time_point start_time, end_time;
	double  total_time = 0.0;
	int time_counter = 0;
	
	public:
	int row,col;
	int remapIdx[85*360]; // rtheta image size (85 x 360)
	
	// Constructor
	ImageConverter()
		: it_(nh_)
	{
		image_sub_ = it_.subscribe("/lane_img", 1, &ImageConverter::imageCb, this); // size (120 x 120)
		image_pub_ = it_.advertise("/rtheta_img", 1);  //  size (85 x 360)
		row = 85; col = 360;
		string str, token;
		double temp;
		ifstream ifs("/home/ubuntu/catkin_ws/src/line_detection/src/conv_to_rtheta.csv"); // lookup table for remap to rtheta image
		int count = 0;
		while(getline(ifs,str)){
			istringstream stream(str);
			while(getline(stream,token,',')){
				remapIdx[count++] = atof(token.c_str());
			}
		}
		
		return;
	}
	
	// Destructor
	~ImageConverter()
	{
		cv::destroyAllWindows;
	}
	
	// Callbuck function
	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		//tart_time = chrono::system_clock::now();
		
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg,"8UC1");
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		
		cv::Mat lane_img = cv_ptr->image;
		cv::Mat rtheta_img = cv::Mat_<unsigned char>(row,col);
		unsigned char *rth_ptr;
		unsigned char *lane_ptr;
		for(int i = 0; i < 30600; i++) {
			rth_ptr = rtheta_img.ptr<unsigned char>(i/360);
			lane_ptr = lane_img.ptr<unsigned char>(remapIdx[i]/120);
			rth_ptr[i%360] = lane_ptr[remapIdx[i]%120];
			//rtheta_img.at<unsigned char>(i/360, i%360) = lane_img.at<unsigned char>(remapIdx[i]/120, remapIdx[i]%120); // this command is also ok.
		}
		
		cv_bridge::CvImagePtr newPtr = cv_bridge::toCvCopy(msg,"");
		newPtr->image = rtheta_img;
		image_pub_.publish(newPtr->toImageMsg());
		
		/*
		//----------Time count-----------
		end_time = chrono::system_clock::now();
		total_time += static_cast<double>(chrono::duration_cast<chrono::microseconds>(end_time - start_time).count()/1000.0);
		time_counter++;
		if (time_counter == 100) {
			printf("imRtheta %lf[ms]\n", total_time/100.0);
			total_time = 0.0;
			time_counter = 0;
		}
		*/
		
		// cv::resize(rtheta_img, rtheta_img, cv::Size(), 2, 2);
		// cv::imshow("r-theta image", rtheta_img*255);
		// cv::waitKey(1);
	}

	
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "imRtheta_igvc2022");
	ImageConverter ic;
	ros::spin();
	return 0;
}


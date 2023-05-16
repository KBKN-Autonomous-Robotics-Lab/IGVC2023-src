#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>
#include <limits>

using namespace std;


class RthetaToLaserScan
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Publisher scan_pub_;
	chrono::system_clock::time_point start_time, end_time;
	double  total_time = 0.0;
	int time_counter = 0;
	
	public:
	
	RthetaToLaserScan()
		: it_(nh_)
	{
		image_sub_ = it_.subscribe("/rtheta_img", 1, &RthetaToLaserScan::imageCallback, this); // size (85 x 360)
		scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/camera_scan", 50); 
	}
	
	~RthetaToLaserScan()
	{
		cv::destroyAllWindows;
	}
	
	void imageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		//start_time = chrono::system_clock::now();
		
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
		
		cv::Mat rtheta_img = cv_ptr->image;
		double max_distance = rtheta_img.rows;
		int num_readings = rtheta_img.cols;
		double laser_frequency = 30.0;
		
		// create LaserScan message from r-theta image		
		sensor_msgs::LaserScan camera_scan;
		camera_scan.header.stamp = ros::Time::now();
		camera_scan.header.frame_id = "camera_link";
		camera_scan.angle_max = M_PI; //rad
		camera_scan.angle_min = -camera_scan.angle_max;
		camera_scan.angle_increment = -(camera_scan.angle_max-camera_scan.angle_min) / num_readings;
		camera_scan.time_increment = (1. / laser_frequency) / (num_readings);
		camera_scan.scan_time = 0.1;
		camera_scan.range_min = 0.3;
		camera_scan.range_max = 10.0; //[m]
		camera_scan.intensities = {};
		
		vector<float> ranges(num_readings, numeric_limits<float>::infinity());
		float r2d = 0.063;  // multiplication value for convert to real distance from pixel distance
		// float r2d = 0.09;
		unsigned char *ptr;
		unsigned char pixval;
		for(int i=0; i<max_distance; i++) {
			ptr = rtheta_img.ptr<unsigned char>(i);  // get pointer of each row
			for(int j=90; j<num_readings+90; j++) { // to adjust to order the LaserScan message, start col 90 
				pixval = ptr[j%360];
				if(pixval != 0) {
					ranges[j-90] = (max_distance-i)*r2d;
				}
			}
		}
		camera_scan.ranges = ranges;
		scan_pub_.publish(camera_scan);
		
		
		/*
		//----------Time count-----------
		end_time = chrono::system_clock::now();
		total_time += static_cast<double>(chrono::duration_cast<chrono::microseconds>(end_time - start_time).count()/1000.0);
		time_counter++;
		if (time_counter == 100) {
			printf("rtheta2scan %lf[ms]\n", total_time/100.0);
			total_time = 0.0;
			time_counter = 0;
		}
		*/
		
		//----------For debug-------------
		//cv::imshow("remap Image", rtheta_img*255);
		//cv::waitKey(1);
		
	}

};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "rtheta2scan_igvc2022");
	RthetaToLaserScan rth2scan;
	ros::spin();
	return 0;

}

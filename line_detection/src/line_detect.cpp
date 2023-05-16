#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <line_detection/reconfigConfig.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <string>
#include <math.h>
#include <chrono>

using namespace std;

class LineDetector
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	cv::Mat map1_, map2_;
	bool sim_; // is true when simulation
	bool show_;
	uint8_t thresh_val_;
	
	public:
	
	// Constructor
	LineDetector()
		: it_(nh_)
	{
		image_sub_ = it_.subscribe("/cv_camera/image_raw", 1, &LineDetector::imageCallback, this); // size (85 x 360)
		image_pub_ = it_.advertise("/lane_img_test", 1);
		nh_.getParam("/sim", sim_);
		cv::Mat K, D;
		cv::Size S;
		if(sim_) {
			K = (cv::Mat_<float>(3,3) << 197.33531703864534,0.0,399.5746325526872, 0.0,197.33723610911758,399.51648288464474, 0.0,0.0,1.0);
			D = (cv::Mat_<float>(1,4) << 0.08515002883616878, 0.012936405125198296, -0.0018582939071174716, 0.0008693014693496443);
			S = cv::Size(800,800);
		} else {
			K = (cv::Mat_<float>(3,3) << 472.4101671967401,0.5799913144184203,750.6027871515556, 0.0,472.753090371875,753.8213756085806, 0.0,0.0,1.0);
			D = (cv::Mat_<float>(1,4) << -0.05367925701562622, 0.07140332123124533, -0.08089849852274485, 0.03184938555925065);
			S = cv::Size(1504,1504);
		}
		cv::fisheye::initUndistortRectifyMap(K, D, cv::Mat::eye(3,3,CV_32F), K, S, CV_16SC2, map1_, map2_);
		
		show_ = false;
		}
	
	// Deconstructor
	~LineDetector()
	{
		cv::destroyAllWindows();
	}
	
	// Callback function
	void imageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");
		} catch(cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		
		// Convert to grayscale & initialize
		cv::Mat raw_img = cv_ptr->image;
		cv::Mat ground_img;
		if(sim_) {
			ground_img = raw_img;
			
		} else {
			int row = raw_img.rows;
			int col = raw_img.cols;
			ground_img = raw_img(cv::Rect(0, 0, row, col/2));
			cv::rotate(ground_img, ground_img, cv::ROTATE_90_CLOCKWISE);
		}
		cv::Mat gray_img;
		cv::cvtColor(ground_img, gray_img, CV_RGB2GRAY);
		
		// Undistortion & Resize
		cv::Mat undis;
		cv::remap(gray_img, undis, map1_, map2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
		cv::Size output_size = cv::Size(120,120);
		cv::resize(gray_img, gray_img, output_size);
		
		// Threshold
		cv::Mat thresh;
		cv::threshold(undis, thresh, this->thresh_val_, 255, cv::THRESH_BINARY);
		
		// Show images
		//if(this->show_) {
		if(true) {
			cv::Size s = cv::Size(400,400);
			cv::resize(ground_img, ground_img, s);
			cv::resize(undis, undis, s); 
			cv::resize(thresh, thresh, s);
			cv::imshow("Raw Image", ground_img);
			cv::imshow("Undistorted Gray Image", undis);
			cv::imshow("Threshold Image", thresh);
		} else {
			cv::destroyAllWindows();
		}
		cv::waitKey(10);
		
	}
	
	
	void reconfigCallback(line_detection::reconfigConfig &config, uint32_t level)
	{
		this->show_ = config.show_images;
		this->thresh_val_ = config.thresh_before_temp_match;
	}

};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "line_detect");
	LineDetector *ld = new LineDetector;
	
	dynamic_reconfigure::Server<line_detection::reconfigConfig> server;
	dynamic_reconfigure::Server<line_detection::reconfigConfig>::CallbackType f;
	f = boost::bind(&LineDetector::reconfigCallback, ld, _1, _2);
	server.setCallback(f);
	
	ros::spin();
	return 0;

}

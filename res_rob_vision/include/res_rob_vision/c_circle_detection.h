#ifndef CIRCLE_DETECTION_H
#define CIRCLE_DETECTION_H

#include "ros/ros.h" 

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp> 

#include "std_msgs/String.h" 

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <vector>

#include <res_rob_vision/config_PP.h>  //PACKAGE_PATH

#include <dynamic_reconfigure/server.h>
#include <res_rob_vision/CircleDetectionConfig.h>

#include <res_rob_vision/freqfunctions.hpp>

enum FilterMethod {GAUSBLUR_AND_ERODE, BLUR_AND_THRSHOLD, MORPH_OPENING};

class CircleDetection
{
public:
	CircleDetection()
	{	
		cv::String topic_to_subscribe = "camera/image_raw";
		cv::String topic_to_publish = "circle_detection/info";
		DEBUGGING_ = false;
		filter_method_ = MORPH_OPENING;
	    HC_minDist_ = 2;
	    HC_param1_ = 200;
	    HC_param2_ = 0;
	    HC_minRadius_  = 10;
	    HC_maxRadius_ = 2;

		show_colors_.clear(); show_colors_.push_back(true); show_colors_.push_back(true); show_colors_.push_back(true);

		loadParameters(nl_, topic_to_subscribe, topic_to_publish);

		image_transport::ImageTransport it_(n_);
		imag_sub_ = it_.subscribe(topic_to_subscribe, 1, &CircleDetection::imageCallback, this);
		// imag_pub_ = it_.advertise(topic_to_publish, 1);

		dyn_rec_server_.setCallback(boost::bind(&CircleDetection::dynRecCallback, this, _1, _2));
	}
	~CircleDetection()  {}

private:
	ros::NodeHandle n_;
	ros::NodeHandle nl_;

	dynamic_reconfigure::Server<res_rob_vision::CircleDetectionConfig> dyn_rec_server_;

	image_transport::Subscriber imag_sub_;
	image_transport::Publisher imag_pub_;

	cv::Mat camera_input_;

	// cv::String somevariable_;
	// int someint_;

	bool DEBUGGING_;
	std::vector<bool> show_colors_;
	FilterMethod filter_method_;
	int threshold_value_;
	int er_dir_value_;

	int HC_minDist_;
	int HC_param1_;
	int HC_param2_;
	int HC_minRadius_;
	int HC_maxRadius_;

	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void loadParameters(const ros::NodeHandle &node, cv::String &topic_to_subscribe, cv::String &topic_to_publish );
	void dynRecCallback(res_rob_vision::CircleDetectionConfig &config, uint32_t level);
	std::string getFilterMethodName(const FilterMethod what_method);

	cv::Mat equalizeIntensity(const cv::Mat& inputImage);

	bool loadColors(cv::String filename);
	int wall_color_hsv_values_[1][6];

};//End of class QrExtractor


#endif // CIRCLE_DETECTION_H
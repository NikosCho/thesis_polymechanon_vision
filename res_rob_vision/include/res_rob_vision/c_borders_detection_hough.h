#ifndef BORDERS_DETECTION_H
#define BORDERS_DETECTION_H

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
#include <res_rob_vision/AreaBordersConfig.h>

#include <res_rob_vision/freqfunctions.hpp>

enum BorderColor {YELLOW , ORANGE, RED, WALL};		// yellow: 0,255,255 | orange: BGR-0,122,255 | red: BGR-0,0,255 |

class BordersDetection
{
public:
	BordersDetection()
	{	
		cv::String topic_to_subscribe = "camera/image_raw";
		cv::String topic_to_publish = "area_borders/borders_cutted";
		DEBUGGING_ = false;

		show_colors_.clear(); show_colors_.push_back(true); show_colors_.push_back(true); show_colors_.push_back(true);
		houghlinesp_parameters_.clear(); houghlinesp_parameters_.push_back(100); houghlinesp_parameters_.push_back(100); houghlinesp_parameters_.push_back(50); 

		loadParameters(nl_, topic_to_subscribe, topic_to_publish);

		image_transport::ImageTransport it_(n_);
		imag_sub_ = it_.subscribe(topic_to_subscribe, 1, &BordersDetection::imageCallback, this);
		imag_pub_ = it_.advertise(topic_to_publish, 1);

		dyn_rec_server_.setCallback(boost::bind(&BordersDetection::dynRecCallback, this, _1, _2));
	}
	~BordersDetection()  {}

private:
	ros::NodeHandle n_;
	ros::NodeHandle nl_;

	dynamic_reconfigure::Server<res_rob_vision::AreaBordersConfig> dyn_rec_server_;

	image_transport::Subscriber imag_sub_;
	image_transport::Publisher imag_pub_;

	cv::Mat camera_input_;

	int color_hsv_values_[4][7];

	bool DEBUGGING_;
	std::vector<bool> show_colors_;
	std::vector<int> houghlinesp_parameters_;  // 0-threshold 1-minLineLength 2-maxLineGap
	

	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void loadParameters(const ros::NodeHandle &node, cv::String &topic_to_subscribe, cv::String &topic_to_publish );
	void dynRecCallback(res_rob_vision::AreaBordersConfig &config, uint32_t level);

	bool loadColors(cv::String filename);
	void drawBorders(cv::Mat &input_image, cv::Mat &hsv_input_image);
	void drawALLBorders(cv::Mat &input_image, cv::Mat &hsv_input_image);
	float findLength(cv::Point2d A, cv::Point2d B);
	float findLength(cv::Vec4i line);
	void findPerfectPoints(std::vector<cv::Vec4i> &lines,std::vector<cv::Point2d> &templine);
	void findBiggest(std::vector<cv::Vec4i> lines,std::vector<cv::Point2d> &templine);


	void printColors();
	cv::String getStringColorName(BorderColor whatcolor);

};//End of class QrExtractor


#endif // BORDERS_DETECTION_H
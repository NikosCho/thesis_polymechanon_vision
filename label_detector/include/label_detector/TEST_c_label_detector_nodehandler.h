#ifndef TEST_LABEL_DETECTOR_NODEHANDLER_H
#define TEST_LABEL_DETECTOR_NODEHANDLER_H

#include <iostream>
#include <string>

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

#include <dynamic_reconfigure/server.h>
#include <label_detector/LabelDetectorConfig.h>

// Other class dependecies
#include "label_detector/c_label_detector.h"

namespace polymechanon_vision {

class TestLabelDetectorNodeHandler
{	
public:
	// TestLabelDetectorNodeHandler();
	TestLabelDetectorNodeHandler(std::string filepath, std::string type);
	~TestLabelDetectorNodeHandler();
	void start();

private:
	cv::Mat _camera_input;
	std::string _file_path;
	bool _image_video_switch;


	ros::NodeHandle _node;
	// image_transport::Subscriber _subscriber_to_img_node;
	dynamic_reconfigure::Server<label_detector::LabelDetectorConfig> _dyn_rec_server;

	LabelDetector _detector;
	
	// void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	std::string loadTopic(const ros::NodeHandle& node, std::string topic_name = "camera/image_raw");	
	void loadDetectorSettings(const ros::NodeHandle& node);
	void dynRecCallback(label_detector::LabelDetectorConfig &config, uint32_t level);


};

} // "namespace polymechanon_vision"
#endif // TEST_LABEL_DETECTOR_NODEHANDLER_H
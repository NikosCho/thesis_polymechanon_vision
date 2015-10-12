#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <iostream>
#include <string>
#include <vector>


#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

// Dynamic Reconfigure Headers
#include <dynamic_reconfigure/server.h>
#include <victim_detector/ObjectDetectorConfig.h>
#include <victim_detector/config_PP.h>  //PACKAGE_PATH

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace polymechanon_vision {


class ObjectDetector
{
public:
	ObjectDetector();
	~ObjectDetector();


private:
	// ROS node's setup
	ros::NodeHandle _node;
	image_transport::Subscriber _subscriber_to_img_node;
	dynamic_reconfigure::Server<object_detector::ObjectDetectorConfig> _dyn_rec_server;

	cv::CascadeClassifier _object_cascade;

	bool _debugging;
	int _minimum_object_size;
	int _maximum_object_size;

	// Node's setup
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	std::string loadTopic(const ros::NodeHandle& node, std::string topic_name = "/camera/image_raw");	
	void loadDetectorSettings(const ros::NodeHandle& node);
	// void dynRecCallback(victim_detector::VictimDetectorConfig &config, uint32_t level);
	void dynRecCallback(object_detector::ObjectDetectorConfig &config, uint32_t level);
	void loadCascadeClassifier(const std::string& path);



	void detectObject(cv::Mat& image_to_scan, cv::CascadeClassifier& classifier);











};

} // "namespace polymechanon_vision"
#endif // OBJECT_DETECTOR_H
















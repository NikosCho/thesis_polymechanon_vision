#ifndef THERMAL_DETECTOR_H
#define THERMAL_DETECTOR_H

#include <iostream>
#include <string>


#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

// Dynamic Reconfigure Headers
#include <dynamic_reconfigure/server.h>
#include <victim_detector/ThermalDetectorConfig.h>


namespace polymechanon_vision {


class ThermalDetector
{
public:
	ThermalDetector();
	~ThermalDetector();


private:
	// ROS node's setup
	ros::NodeHandle _node;
	image_transport::Subscriber _subscriber_to_img_node;
	dynamic_reconfigure::Server<thermal_detector::ThermalDetectorConfig> _dyn_rec_server;

	bool _debugging;
	int _minimum_blob_size;
	int _maximum_blob_size;

	cv::SimpleBlobDetector _detector;

	// cv::SimpleBlobDetector _ptr;
	// Node's setup
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	std::string loadTopic(const ros::NodeHandle& node, std::string topic_name = "/camera/image_raw");	
	void loadDetectorSettings(const ros::NodeHandle& node);
	// void dynRecCallback(victim_detector::VictimDetectorConfig &config, uint32_t level);
	void dynRecCallback(thermal_detector::ThermalDetectorConfig &config, uint32_t level);

	void setupDetector();
	void detectBlobs(cv::Mat& image_to_scan);




};

} // "namespace polymechanon_vision"
#endif // THERMAL_DETECTOR_H
















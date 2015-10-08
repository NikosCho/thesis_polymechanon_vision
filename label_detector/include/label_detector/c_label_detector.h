#ifndef LABEL_DETECTOR_H
#define LABEL_DETECTOR_H

#include <iostream>
#include <string>

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

// Dynamic Reconfigure Headers
#include <dynamic_reconfigure/server.h>
#include <label_detector/LabelDetectorConfig.h>

// Package's Headers
#include "label_detector/detector_settings.h"    // struct DetectorSettings
#include "label_detector/c_label.h"    // class Label
#include "label_detector/c_locator.h"    // class Locator
#include "label_detector/c_scanner.h"    // class Scanner
#include "label_detector/scanners/c_qr_scanner.h"
#include "label_detector/scanners/c_hzl_scanner.h"

#include <visualization_msgs/Marker.h> //for markers in rviz

// #include <tf/transform_broadcaster.h>

namespace polymechanon_vision {

class LabelDetector
{
public:

	LabelDetector();
	~LabelDetector();

private:
	// ROS node's setup
	ros::NodeHandle _node;
	image_transport::Subscriber _subscriber_to_img_node;
	dynamic_reconfigure::Server<label_detector::LabelDetectorConfig> _dyn_rec_server;

	// tf::TransformBroadcaster _tf_broadcaster;
	// tf::Transform transform;

	ros::Publisher _3D_points_pub;

	std::shared_ptr<cv::Mat> _input_image;
	DetectorSettings _settings;
	Locator _locator;
	std::vector<Scanner*> _scanners;
	std::vector<polymechanon_vision::Label> _labels;

	// Node's setup
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	std::string loadTopic(const ros::NodeHandle& node, std::string topic_name = "/usb_camera/image_raw");	
	void loadDetectorSettings(const ros::NodeHandle& node);
	void dynRecCallback(label_detector::LabelDetectorConfig &config, uint32_t level);
	// LabelDetector's setup
	void setSettings(DetectorSettings& settings);
	bool setInputImage(cv::Mat input_image);
	bool detect();
	// Scanners' setup
	void setupScanners();
	bool checkScannerbyType(const LabelType& label);
	void removeScannerbyType(const LabelType& label);










	////////// Debugging //////////
	void drawInfo();
	void calculate3DCoordinates(Label& label);
	void publish3DCoordinates(Label& label);


	////////// Test //////////
	cv::Mat getImageOutput();

};

} // "namespace polymechanon_vision"
#endif // LABEL_DETECTOR_NODEHANDLER_H
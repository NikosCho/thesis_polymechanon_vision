#ifndef BARCODE_DETECOR_H
#define BARCODE_DETECOR_H

#include <iostream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>

#include <dynamic_reconfigure/server.h>
#include <label_detector/LabelDetectorConfig.h>

namespace polymechanonvision {

struct VisionSettings
{	
	std::string TOPIC_TO_SUBSCRIBE;
	bool DEBUGGING;
	bool QR_ENABLED;
	bool HZL_ENABLED;
	int QRSIDE_LENGTH;		// milimeters
};


class LabelDetector
{
public:
	LabelDetector();
	~LabelDetector();

	void detect(cv::Mat& camera_input);

private:
	ros::NodeHandle n_;
	image_transport::Subscriber sub_to_imag_;
	dynamic_reconfigure::Server<label_detector::LabelDetectorConfig> dyn_rec_server_;
	VisionSettings settings_;

	// cv::Mat camera_input_;
	// cv::Mat camera_input_;

	boost::shared_ptr<cv::Mat> sp_camera_input_;

	// cv::Mat camera_input_;

	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void loadSettings(const ros::NodeHandle& node, VisionSettings& settings);
	void dynRecCallback(label_detector::LabelDetectorConfig &config, uint32_t level);
	

	void setCameraInput(cv::Mat input);
};
 
} // "namespace polymechanonvision"

#endif // BARCODE_DETECOR_H
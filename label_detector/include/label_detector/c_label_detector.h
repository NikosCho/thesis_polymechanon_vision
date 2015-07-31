#ifndef LABEL_DETECTOR_H
#define LABEL_DETECTOR_H

#include <iostream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>

#include "label_detector/c_label.h"


namespace polymechanon_vision {

struct DetectorSettings
{	
	bool DEBUGGING;
	bool QR_ENABLED;
	bool HZL_ENABLED;
	int QRSIDE_LENGTH;		// milimeters

	DetectorSettings(): DEBUGGING(false), QR_ENABLED(true), HZL_ENABLED(true), QRSIDE_LENGTH(182){}
};


class LabelDetector
{

public:
	LabelDetector();
	LabelDetector(cv::Mat input_image);
	~LabelDetector();

	void setInputImage(cv::Mat input_image);
	void setSettings(DetectorSettings& settings);

private:
	boost::shared_ptr<cv::Mat> input_image_;
	DetectorSettings settings_;
	std::vector<polymechanon_vision::Label> labels_;

};







} // "namespace polymechanon_vision"
#endif // LABEL_DETECTOR_H
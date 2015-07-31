#ifndef LABEL_DETECTOR_H
#define LABEL_DETECTOR_H

#include <iostream>
#include <string>
#include <memory>

#include "ros/ros.h"
#include <opencv2/opencv.hpp>

// Other class dependecies
#include "label_detector/c_label.h"
#include "label_detector/c_scanner.h"
#include "label_detector/scanners/c_qr_scanner.h"

using std::shared_ptr;
using std::make_shared;

class Scanner;

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

	void detect();

private:
	shared_ptr<cv::Mat> input_image_;
	DetectorSettings settings_;
	std::vector<polymechanon_vision::Label> labels_;

	shared_ptr<Scanner> scanner_;

};







} // "namespace polymechanon_vision"
#endif // LABEL_DETECTOR_H
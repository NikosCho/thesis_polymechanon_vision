#ifndef LABEL_DETECTOR_H
#define LABEL_DETECTOR_H

#include <iostream>
#include <string>
#include <memory>
#include <algorithm>
#include <iterator>
#include <numeric>

#include "ros/ros.h"
#include <opencv2/opencv.hpp>

// Other class dependecies
#include "label_detector/c_label.h"
#include "label_detector/c_scanner.h"
#include "label_detector/scanners/c_qr_scanner.h"

using std::vector;
using std::shared_ptr;
using std::make_shared;
using std::none_of;
using std::remove_if;

typedef cv::Point2f Point2D;



namespace polymechanon_vision {

struct DetectorSettings
{	
	bool DEBUGGING;
	bool QR_ENABLED;
	bool HZL_ENABLED;
	int QRSIDE_LENGTH;		// milimeters

	DetectorSettings(): DEBUGGING(true), QR_ENABLED(true), HZL_ENABLED(true), QRSIDE_LENGTH(182){}
};

class LabelDetector
{
public:
	LabelDetector();
	LabelDetector(cv::Mat input_image);
	~LabelDetector();

	void setSettings(DetectorSettings& settings);
	bool setInputImage(cv::Mat input_image);
	bool detect();

	////////// Test //////////
	cv::Mat getImageOutput();
	int getNumberOfDetectedLabels();
	
private:
	shared_ptr<cv::Mat> _input_image;
	DetectorSettings _settings;
	std::vector<polymechanon_vision::Label> _labels;
	std::vector<vector<Point2D> > _labels_contours;
	std::vector<Scanner*> _scanners;

	// Scanners setup
	void setupScanners();
	bool checkScannerbyType(const LabelType& label);
	void removeScannerbyType(const LabelType& label);

	////////// Debugging //////////
	void drawInfo();


};

} // "namespace polymechanon_vision"
#endif // LABEL_DETECTOR_H
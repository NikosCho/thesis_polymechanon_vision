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
#include "label_detector/scanners/c_hzl_scanner.h"

#include "label_detector/c_locator.h"

typedef cv::Point2f Point2D;
typedef cv::Point3f Point3D;

namespace polymechanon_vision {

struct DetectorSettings
{	
	bool DEBUGGING;    ///< Debugging switch. If true image windows, showing the results of scanning, will be opened.
	
	//QR parameters
	bool QR_ENABLED;    ///< Enable/Disable Qr code scanning.
	bool QR_DBG_ENABLED;    
	int QR_CANNY_PAR1;
	int QR_CANNY_PAR2;
	int QRSIDE_LENGTH;		// milimeters

	//HZL parameters
	bool HZL_ENABLED;    ///< Enable/Disable Hazardous label scanning.
	bool HZL_DBG_ENABLED;    
	int HZL_CANNY_PAR1;
	int HZL_CANNY_PAR2;
	int HZL_MATCHING_METHOD;	
	int HZL_TEMPLATE_MATCHING_METHOD;	
	bool ENABLE_COLOR_MATCHING; 
	int HZLSIDE_LENGTH;		// milimeters

	// Localizing parameters
	bool LOC_DBG_ENABLED;
	int LOCALIZING_METHOD;

	DetectorSettings(): DEBUGGING(true), 
						QR_ENABLED(true), 
						QR_DBG_ENABLED(false), 
						QR_CANNY_PAR1(100), 
						QR_CANNY_PAR2(200), 
						QRSIDE_LENGTH(182), 
						HZL_ENABLED(true), 
						HZL_DBG_ENABLED(false), 
						HZL_CANNY_PAR1(100), 
						HZL_CANNY_PAR2(200), 
						HZL_MATCHING_METHOD(0), 
						HZL_TEMPLATE_MATCHING_METHOD(4), 
						ENABLE_COLOR_MATCHING(true), 
						HZLSIDE_LENGTH(182), 
						LOC_DBG_ENABLED(false),
						LOCALIZING_METHOD(0)
						{}
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
	std::shared_ptr<cv::Mat> _input_image;
	DetectorSettings _settings;
	std::vector<polymechanon_vision::Label> _labels;
	std::vector<std::vector<Point2D> > _labels_contours;
	std::vector<Scanner*> _scanners;
	Locator _locator;

	// Scanners setup
	void setupScanners();
	bool checkScannerbyType(const LabelType& label);
	void removeScannerbyType(const LabelType& label);

	////////// Debugging //////////
	void drawInfo();
	vector< Point3D > find3DCoordinates(Label& label);


};

} // "namespace polymechanon_vision"
#endif // LABEL_DETECTOR_H
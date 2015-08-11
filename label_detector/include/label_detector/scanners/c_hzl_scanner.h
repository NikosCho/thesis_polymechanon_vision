#ifndef HZL_SCANNER_H
#define HZL_SCANNER_H

#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp>

// Other class dependecies
#include "label_detector/c_scanner.h"
#include "label_detector/c_label.h"

using std::shared_ptr;
using std::vector;

namespace polymechanon_vision {

class HzlScanner : public Scanner
{	

public:
	HzlScanner(shared_ptr<cv::Mat> input_image = nullptr);
	virtual ~HzlScanner();

	LabelType getType() const;
	cv::Mat getDebuggingImage();
	void setImageToScan(const shared_ptr<cv::Mat> input_image); 
	bool scan();

	vector<vector<Point2D> > getDetectedLabels();

	///////////////////// Debugging Functions /////////////////////
	bool drawDetectedLabels(shared_ptr<cv::Mat> inputimage);
	bool drawDetectedLabels(cv::Mat &inputimage);
	cv::Mat getImageOfDetectedLabels(const cv::Mat &inputimage);


private:
	static const LabelType _type = LabelType::HZL;
	/* shared_ptr<cv::Mat> _image_to_scan */


};

} // "namespace polymechanon_vision"
#endif // HZL_SCANNER_H
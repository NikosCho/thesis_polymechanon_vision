#ifndef SCANNER_H
#define SCANNER_H

#include <iostream>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>

// Other class dependecies
#include "label_detector/c_label.h"

using std::shared_ptr;

namespace polymechanon_vision {

class Scanner
{

public:
	Scanner();
	~Scanner();

	// void setImageToScan(cv::Mat* input_image);
	void setImageToScan(shared_ptr<cv::Mat> input_image);
	bool scan();
	void showImage();

protected:
	shared_ptr<cv::Mat> image_to_scan_;

private:
	LabelType type_;
	

};

} // "namespace polymechanon_vision"
#endif // SCANNER_H
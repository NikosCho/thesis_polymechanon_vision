#ifndef SCANNER_H
#define SCANNER_H

#include <iostream>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>

// Other class dependecies
#include "label_detector/c_label.h"

#include "ros/ros.h"

using std::shared_ptr;

namespace polymechanon_vision {

class Scanner
{

public:
	Scanner(shared_ptr<cv::Mat> input_image = nullptr);
	virtual ~Scanner();

	virtual LabelType getType() const;
	void setImageToScan(const shared_ptr<cv::Mat> input_image);
	virtual bool scan();
	
protected:
	shared_ptr<cv::Mat> _image_to_scan;
	
private:
	LabelType _type;
};

} // "namespace polymechanon_vision"
#endif // SCANNER_H
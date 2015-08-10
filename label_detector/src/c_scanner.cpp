#include "label_detector/c_scanner.h"

namespace polymechanon_vision {

Scanner::Scanner(shared_ptr<cv::Mat> input_image /* = nullptr */)
{	
	setImageToScan(input_image);
	ROS_WARN("Scanner created!");
}

Scanner::~Scanner(){ ROS_ERROR("Scanner deleted!"); }

LabelType Scanner::getType() const
{
	return _type;
}

void Scanner::setImageToScan(const shared_ptr<cv::Mat> input_image)
{	
	_image_to_scan = input_image;
}

cv::Mat Scanner::getDebuggingImage()
{
	return *_image_to_scan;
}

bool Scanner::scan()
{	

	ROS_WARN("Scanner called");

	return true;
}

bool Scanner::drawDetectedLabels(shared_ptr<cv::Mat> inputimage)
{	
	ROS_WARN("Scanner called");
	return false;
}

bool Scanner::drawDetectedLabels(cv::Mat &inputimage)
{
	return false;
}

cv::Mat Scanner::getImageDetectedLabels(const cv::Mat &inputimage)
{	
	cv::Mat image_to_draw = inputimage.clone();
	ROS_WARN("Scanner called");

	return image_to_draw;
}

vector<vector<Point2D> > Scanner::getDetectedLabels()
{	
	return _detected_labels;
}



} // "namespace polymechanon_vision"
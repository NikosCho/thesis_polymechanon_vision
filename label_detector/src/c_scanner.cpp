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

bool Scanner::scan()
{	


	return true;
}



} // "namespace polymechanon_vision"
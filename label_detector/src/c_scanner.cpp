#include "label_detector/c_scanner.h"

namespace polymechanon_vision {

Scanner::Scanner()
{
	image_to_scan_ = nullptr;
}

Scanner::~Scanner(){}

void Scanner::setImageToScan(shared_ptr<cv::Mat> input_image)
{	
	image_to_scan_ = input_image;
}

bool Scanner::scan()
{	


	return true;
}

void Scanner::showImage()
{
	// if (image_to_scan_)
		// cv::imshow("fads",*image_to_scan_);
}

} // "namespace polymechanon_vision"
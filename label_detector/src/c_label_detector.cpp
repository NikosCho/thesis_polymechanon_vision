#include "label_detector/c_label_detector.h"

namespace polymechanon_vision {

LabelDetector::LabelDetector()
{
	input_image_ = nullptr;
}

LabelDetector::LabelDetector(cv::Mat input_image)
{
	setInputImage(input_image);
}

LabelDetector::~LabelDetector(){}

void LabelDetector::setInputImage(cv::Mat input_image)
{	
	try  {
		input_image_ = make_shared<cv::Mat>(input_image);
	}
	catch ( std::bad_alloc& e)  {
		ROS_ERROR("LabelDetector - setCameraInput(): bad_alloc exception while setting camera input.");
	}
}

void LabelDetector::setSettings(DetectorSettings& settings)
{
	settings_ = settings;
}

void LabelDetector::detect()
{
	scanner_ = make_shared<QrScanner>();
	scanner_->setImageToScan(input_image_);
	scanner_->showImage();
}




} // "namespace polymechanon_vision"
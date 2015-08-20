#include "label_detector/scanners/c_hzl_scanner.h"

namespace polymechanon_vision {

HzlScanner::HzlScanner(shared_ptr<cv::Mat> input_image /* = nullptr */)
{	
	// if ( input_image != nullptr)    setImageToScan(input_image);
	if ( input_image != nullptr)    Scanner::setImageToScan(input_image);
	ROS_WARN("HZL-Scanner created!");
}

HzlScanner::~HzlScanner() { 
	ROS_ERROR("HZL-Scanner deleted!"); 
}

/// Get the type of scanner's detection
/**
*	Virtual method of base class 'Scanner' 
*/
LabelType HzlScanner::getType() const
{
	return _type;
}

/// 'Main' function of scanner.
/**
*	Implementation of whole scanner process.
*	
*	\returns true/false accοording to the progress of scanning (true - if anything was detected  false - if not).
*	\exception std::runtime_error - if camera input's pointer is 'nullptr'.
*/
bool HzlScanner::scan()
{	
	if (_image_to_scan == nullptr ) throw std::runtime_error("[HzlScanner]-scan() : image's pointer is nullptr (use 'setImageToScan()'). ");




	return false;
}

vector<vector<Point2D> > HzlScanner::getDetectedLabels()
{	
	vector<vector<Point2D> > detected_labels;
	return detected_labels;
}










///////////////////////// Debugging Functions /////////////////////////
bool HzlScanner::drawDetectedLabels(shared_ptr<cv::Mat> inputimage)
{	
	return false;
}

bool HzlScanner::drawDetectedLabels(cv::Mat &inputimage)
{
	return false;
}

cv::Mat HzlScanner::getImageOfDetectedLabels(const cv::Mat &inputimage)
{	
	cv::Mat image_to_draw = inputimage.clone();
	return image_to_draw;
}



} // "namespace polymechanon_vision"
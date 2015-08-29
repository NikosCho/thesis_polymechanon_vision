#include "label_detector/c_scanner.h"

namespace polymechanon_vision {

Scanner::Scanner(shared_ptr<cv::Mat> input_image /* = nullptr */)
{	
	if ( input_image != nullptr)    setImageToScan(input_image);
	ROS_WARN("Scanner created!");
}

Scanner::~Scanner(){ 
	ROS_ERROR("Scanner deleted!"); 
}

LabelType Scanner::getType() const
{
	return _type;
}

void Scanner::setImageToScan(const shared_ptr<cv::Mat> input_image)
{	
	if ( input_image == nullptr )  throw std::runtime_error("[Scanner]-setImageToScan() : image's pointer is nullptr. ");
	_image_to_scan = input_image;
}

bool Scanner::scan()
{	
	if (_image_to_scan == nullptr ) throw std::runtime_error("[Scanner]-scan() : image's pointer is nullptr (use 'setImageToScan()'). ");	
	ROS_WARN("Scanner called");
	return false;
}

vector<vector<Point2D> > Scanner::getDetectedLabels()
{	
	return _detected_labels;
}

bool Scanner::setParameters(int par1, int par2)
{
	return true;
}

bool Scanner::setParameters(int par1, int par2, int side_length, int match_mehod, int contour_area_thres)
{
	return true;
}


//////////////////////////////// Debugging Functions ////////////////////////////////
bool Scanner::drawDetectedLabels(shared_ptr<cv::Mat> inputimage)
{	
	ROS_WARN("Scanner called");
	return false;
}

bool Scanner::drawDetectedLabels(cv::Mat &inputimage)
{
	return false;
}

cv::Mat Scanner::getImageOfDetectedLabels(const cv::Mat &inputimage)
{	
	cv::Mat image_to_draw = inputimage.clone();
	ROS_WARN("Scanner called");

	return image_to_draw;
}





} // "namespace polymechanon_vision"
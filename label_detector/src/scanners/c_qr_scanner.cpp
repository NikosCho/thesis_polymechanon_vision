#include "label_detector/scanners/c_qr_scanner.h"

namespace polymechanon_vision {

QrScanner::QrScanner() //: Scanner()
{
	ROS_WARN("QR-Scanner created!");
}

QrScanner::~QrScanner(){ ROS_ERROR("QR-Scanner deleted!"); }

LabelType QrScanner::getType() const
{
	return _type;
}

bool QrScanner::scan()
{	

	cv::imshow("piou piou", *image_to_scan_);	

	return true;
}






} // "namespace polymechanon_vision"
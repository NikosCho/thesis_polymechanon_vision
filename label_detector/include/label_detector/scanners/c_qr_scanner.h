#ifndef QR_SCANNER_H
#define QR_SCANNER_H

#include <iostream>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>

// Other class dependecies
#include "label_detector/c_scanner.h"
#include "label_detector/c_label.h"

#include "ros/ros.h"

using std::shared_ptr;

namespace polymechanon_vision {

class QrScanner : public Scanner
{	

public:
	QrScanner();
	virtual ~QrScanner();

	LabelType getType() const;
	/* void setImageToScan(const shared_ptr<cv::Mat> input_image); */
	bool scan();


private:
	static const LabelType _type = LabelType::QRCODE;
	/* shared_ptr<cv::Mat> image_to_scan_ */


};

} // "namespace polymechanon_vision"
#endif // QR_SCANNER_H
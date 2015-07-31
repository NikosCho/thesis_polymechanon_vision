#ifndef QR_SCANNER_H
#define QR_SCANNER_H

#include <iostream>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>

// Other class dependecies
#include "label_detector/c_scanner.h"
#include "label_detector/c_label.h"

using std::shared_ptr;

namespace polymechanon_vision {

class QrScanner : public Scanner
{	

public:
	QrScanner();
	~QrScanner();

	bool scan();

private:
	static const LabelType type_ = LabelType::QRCODE; 
	// shared_ptr<cv::Mat> image_to_scan_;

};

} // "namespace polymechanon_vision"
#endif // QR_SCANNER_H
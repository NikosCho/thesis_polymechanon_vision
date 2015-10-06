# Rescue Robot packages # 
Created in the name of [Robotics Club](http://robotics.mech.upatras.gr/), for [Polymechanon](http://robotics.mech.upatras.gr/club/index.php/projects/polymechanon) project. 

All packages need `ROS` and `OpenCV` to be installed. 

##label_detector pkg##

Label detection and localization.
The main function of this package are the scanners implemented. Each scanner has a responsibilitty of
detecting and translating a certain label.

Currently implemented scanners:

Type | Name | Fully_implemented/or not | Other dependecies
:---: | :---: | ---: | ---: 
Qr code | qr_scanner | almost | `zbar-tools`
Hazardous label | hzl_scanner | not | No other dependecies

###Basic form of scanner-class###

All scanner classes must be implemented *this* way.

```c++

#ifndef NEW_SCANNER_H
#define NEW_SCANNER_H

#include "label_detector/c_scanner.h"

class NewScanner : public Scanner
{	
public:
	NewScanner(shared_ptr<cv::Mat> input_image = nullptr);
	virtual ~NewScanner();

	LabelType getType() const;
	bool scan();
	vector<vector<Point2D> > getDetectedLabels();	
	// Debugging functions //
	bool drawDetectedLabels(shared_ptr<cv::Mat> inputimage);

private:
	static const LabelType _type = LabelType::QRCODE;
	vector<vector<Point2D> > _detected_labels;
}

#endif // QR_SCANNER_H

```


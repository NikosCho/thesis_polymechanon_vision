#include "label_detector/c_label_detector.h"

namespace polymechanon_vision {

LabelDetector::LabelDetector()
{
	_input_image = nullptr;
	ROS_WARN("LabelDetector created!");
}

LabelDetector::LabelDetector(cv::Mat input_image)
{	
	setInputImage(input_image);
}

LabelDetector::~LabelDetector(){ ROS_ERROR("LabelDetector deleted!"); }

void LabelDetector::setInputImage(cv::Mat input_image)
{	
	try  {
		_input_image = make_shared<cv::Mat>(input_image);
	}
	catch ( std::bad_alloc& e)  {
		ROS_ERROR("LabelDetector::setCameraInput(): bad_alloc exception while setting camera input.");
	}
}

void LabelDetector::setSettings(DetectorSettings& settings)
{
	_settings = settings;
}

bool LabelDetector::detect()
{
	cv::namedWindow("Debugging-label_detector",1);
	_labels_contours.clear();	


	setupScanners();
	if(!_scanners.empty())
	for ( auto scanner:_scanners )
	{	

		scanner->setImageToScan(_input_image);
		if ( scanner->scan() ) {
			if (_settings.DEBUGGING )    scanner->drawDetectedLabels(_input_image);
			std::vector<vector<Point2D> > detected_labels_contours = scanner->getDetectedLabels();


			_labels_contours.insert(end(_labels_contours), begin(detected_labels_contours), end(detected_labels_contours));


		}


		cv::imshow("Debugging-label_detector", *_input_image);

	}

	std::cout << " [LabelDetector] - Total labels detected = " << _labels_contours.size() << std::endl;

	return true;

}

// Setup scanners according to detector's settings.
// For each scanner's type, create one, if its type is enabled.
// If scanner's type is disabled, udate scanners accordingly ( delete existing disabled scanners).
void LabelDetector::setupScanners()
{	
	// Qrscanner's setup
	if ( _settings.QR_ENABLED && checkScannerbyType(LabelType::QRCODE) ) {
		Scanner* new_scanner = new QrScanner();
		_scanners.push_back(new_scanner);
	} else if ( !_settings.QR_ENABLED ) {
		removeScannerbyType(LabelType::QRCODE);
	}
	///////////////////////////////////////////////
	//  Any other scanner must be placed here!!  //
	///////////////////////////////////////////////
}

// Check if there is a scanner of this label's type
bool LabelDetector::checkScannerbyType(const LabelType& label)
{
	bool type_is_absent = none_of(begin(_scanners), end(_scanners), [&label](const Scanner* sc) {
				return sc->getType() == label;
			});
	return type_is_absent;
}

// Remove scaner of this label's type. 
void LabelDetector::removeScannerbyType(const LabelType& label)
{	
	// Way 1 ////
	auto removeDisabled = remove_if(begin(_scanners), end(_scanners), [&label] (Scanner* sc) {	
				if ( sc->getType() == label ) {
					delete sc;
					return true;
				}
				return false;
			});

	_scanners.erase(removeDisabled, end(_scanners));

	// Way 2 ////
	// for ( auto it = begin(_scanners); it != end(_scanners); )
	// {
	// 	if ((*it)->getType() == label)
	// 	{	
	// 		delete *it;
	// 		it = _scanners.erase(it);
	// 	} else {
	//       ++it;
	//    }
	// }
}


} // "namespace polymechanon_vision"
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
		ROS_ERROR("LabelDetector - setCameraInput(): bad_alloc exception while setting camera input.");
	}
}

void LabelDetector::setSettings(DetectorSettings& settings)
{
	_settings = settings;
}

bool LabelDetector::detect()
{

	setupScanners();
	if(!scanners_.empty())
	for ( auto scanner:scanners_ )
	{
		scanner->setImageToScan(_input_image);
		scanner->scan();
	}


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
		scanners_.push_back(new_scanner);
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
	bool type_is_absent = none_of(begin(scanners_), end(scanners_), [&label](const Scanner* sc) {
				return sc->getType() == label;
			});
	return type_is_absent;
}

// Remove scaner of this label's type. 
void LabelDetector::removeScannerbyType(const LabelType& label)
{	
	// Way 1 ////
	auto removeDisabled = remove_if(begin(scanners_), end(scanners_), [&label] (Scanner* sc) {	
				if ( sc->getType() == label ) {
					delete sc;
					return true;
				}
				return false;
			});

	scanners_.erase(removeDisabled, end(scanners_));

	// Way 2 ////
	// for ( auto it = begin(scanners_); it != end(scanners_); )
	// {
	// 	if ((*it)->getType() == label)
	// 	{	
	// 		delete *it;
	// 		it = scanners_.erase(it);
	// 	} else {
	//       ++it;
	//    }
	// }
}


} // "namespace polymechanon_vision"
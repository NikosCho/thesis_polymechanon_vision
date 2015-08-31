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

LabelDetector::~LabelDetector(){ 
	ROS_ERROR("LabelDetector deleted!"); 
}

void LabelDetector::setSettings(DetectorSettings& settings)
{
	_settings = settings;

	for ( auto scanner:_scanners )
	{	
		if ( scanner-> getType() == LabelType::QRCODE) {
			if ( _settings.QR_ENABLED ) {
				scanner->setParameters(_settings.QR_DBG_ENABLED, _settings.QR_CANNY_PAR1, _settings.QR_CANNY_PAR2);
			}
		}
		if ( scanner-> getType() == LabelType::HZL) {
			if ( _settings.HZL_ENABLED ) {
				scanner->setParameters(_settings.HZL_DBG_ENABLED, _settings.HZL_CANNY_PAR1, _settings.HZL_CANNY_PAR2, 150, _settings.HZL_MATCHING_METHOD, _settings.HZL_TEMPLATE_MATCHING_METHOD, 10000, _settings.ENABLE_COLOR_MATCHING);
			}
		}

	}

}

bool LabelDetector::setInputImage(cv::Mat input_image)
{	
	if ( input_image.empty() ) {
	 	ROS_WARN("[LabelDetector]-setInputImage() : input image is empty.");
	 	return false;
	}

	try  {
		_input_image = make_shared<cv::Mat>(input_image);
	}
	catch ( std::bad_alloc& e)  {
		ROS_ERROR("[LabelDetector]-setInputImage(): bad_alloc exception while setting camera input.");
	 	return false;
	}

	return true;
}

bool LabelDetector::detect()
{	
	if (_input_image == nullptr ) throw std::runtime_error("[LabelDetector]-scan() : image's pointer is nullptr (use 'setInputImage()').");

	static const std::string debugging_window = "[LabelDetector]-Debugging";
	if ( _settings.DEBUGGING ) {
		cv::namedWindow(debugging_window, cv::WINDOW_AUTOSIZE);  // WINDOW_NORMAL,  WINDOW_AUTOSIZE,  WINDOW_OPENGL
	}

	_labels_contours.clear();	

	setupScanners();
	if(!_scanners.empty())
	for ( auto scanner:_scanners )
	{	

		scanner->setImageToScan(_input_image);
		// scanner->setParameters(_settings);
		if ( scanner->scan() ) {
			if (_settings.DEBUGGING )    scanner->drawDetectedLabels(_input_image);
			std::vector<vector<Point2D> > detected_labels_contours = scanner->getDetectedLabels();
			cout << "WUT WUT " << detected_labels_contours.size();

			// _labels_contours.insert(end(_labels_contours), begin(detected_labels_contours), end(detected_labels_contours));
			_labels_contours.insert(end(_labels_contours), begin(detected_labels_contours), end(detected_labels_contours));


		}



	}


	if ( _settings.DEBUGGING ) {
		drawInfo();
		cv::imshow(debugging_window, *_input_image);  // WINDOW_AUTOSIZE, WINDOW_OPENGL
	} else {
		cv::destroyAllWindows();
	}

	////////////////////////////////////////////////////////////////////////////////////////
	// DRAW XYZ - - - http://docs.opencv.org/master/d7/d53/tutorial_py_pose.html#gsc.tab=0
	////////////////////////////////////////////////////////////////////////////////////////
	if ( _labels_contours.size() <1 )    return false;

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

	// Qrscanner's setup
	if ( _settings.HZL_ENABLED && checkScannerbyType(LabelType::HZL) ) {
		Scanner* new_scanner = new HzlScanner();
		_scanners.push_back(new_scanner);
	} else if ( !_settings.HZL_ENABLED ) {
		removeScannerbyType(LabelType::HZL);
	}
	///////////////////////////////////////////////////////////////////////////////
	//                  Any other scanner must be placed here!!                  //
	///////////////////////////////////////////////////////////////////////////////
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


////////// Debugging //////////

void LabelDetector::drawInfo()
{	
	if (_input_image != nullptr ) {
		std::ostringstream text;
		text << "Labels detected: " << _labels_contours.size();	
		// cv::putText(*_input_image, "geia", cv::Point(0, (*_input_image).rows), cv::FONT_HERSHEY_PLAIN, 5, cv::Scalar(0,0,255), 2);
		cv::rectangle(*_input_image, cv::Point(0, (*_input_image).rows), cv::Point((*_input_image).cols, (*_input_image).rows-25), cv::Scalar(50,50,50), CV_FILLED );
		// cv::rectangle(*_input_image, cv::Point(0, (*_input_image).rows), cv::Point((*_input_image).cols, (*_input_image).rows-25), cv::Scalar(0,0,0), CV_FILLED );
		cv::line(*_input_image, cv::Point(0, (*_input_image).rows-27), cv::Point((*_input_image).cols, (*_input_image).rows-27), cv::Scalar(0,0,0), 3 );

		// cv::putText(*_input_image, text.str(), cv::Point(0, (*_input_image).rows-5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,0), 4);
		cv::putText(*_input_image, text.str(), cv::Point(0, (*_input_image).rows-7), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255), 1);
	

	}
}

////////// Test //////////

cv::Mat LabelDetector::getImageOutput()
{
	cv::Mat image_output = (*_input_image).clone();
	return image_output;
}

int LabelDetector::getNumberOfDetectedLabels()
{
	return _labels_contours.size();
}





} // "namespace polymechanon_vision"

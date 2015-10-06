#include "label_detector/c_label_detector.h"

using std::vector;
using std::for_each;
using std::shared_ptr;
using std::make_shared;
using std::none_of;
using std::remove_if;

using std::string;
using std::cout;
using std::endl;

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

	_locator.setParameters( _settings.LOC_DBG_ENABLED, _settings.LOCALIZING_METHOD);

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

	// vector<Label> detected_labels;
	// _labels_contours.clear();	
	_labels.clear();

	setupScanners();
	if(!_scanners.empty())
	for ( auto scanner:_scanners )
	{	

		scanner->setImageToScan(_input_image);
		// scanner->setParameters(_settings);
		if ( scanner->scan() ) {
			if (_settings.DEBUGGING )    
				scanner->drawDetectedLabels(_input_image);
			
			vector<polymechanon_vision::Label> detected_labels = scanner->getDetectedLabels();
			_labels.insert(end(_labels), begin(detected_labels), end(detected_labels));


			// vector<vector<Point2D> > detected_labels_contours = scanner->getDetectedLabels();
			// // _labels_contours.insert(end(_labels_contours), begin(detected_labels_contours), end(detected_labels_contours));
			// _labels_contours.insert(end(_labels_contours), begin(detected_labels_contours), end(detected_labels_contours));

			// vector<Label> scanned_labels = scanner->getDetectedLabels();
			// detected_labels.insert(end(detected_labels), begin(detected_labels_contours), end(detected_labels_contours));

		}



	}

	for (int i = 0; i < _labels.size(); i++)
		find3DCoordinates(_labels[i]);


	// for_each( begin(_labels), end(_labels), [] (const polymechanon_vision::Label& label) {
	// 		cout << label.getText() << endl;
	// 		find3DCoordinates(label);

	// });	

	// for_each( begin(_labels), end(_labels), [] (const polymechanon_vision::Label& label) {
	// 		cout << label.getText() << endl;
	// });

	if ( _settings.DEBUGGING ) {
		drawInfo();
		cv::imshow(debugging_window, *_input_image);  // WINDOW_AUTOSIZE, WINDOW_OPENGL
	} else {
		cv::destroyAllWindows();
	}



	////////////////////////////////////////////////////////////////////////////////////////
	// DRAW XYZ - - - http://docs.opencv.org/master/d7/d53/tutorial_py_pose.html#gsc.tab=0
	////////////////////////////////////////////////////////////////////////////////////////
	if ( _labels_contours.size() < 1 )    return false;

	return true;
}


vector< Point3D > LabelDetector::find3DCoordinates(Label& label)
{	
	
	
	vector<Point2D> points2d = label.get2DPoints();
	vector<Point3D> points3d;
	switch(label.getType())
	{	
		case LabelType::QRCODE: 
			points3d = _locator.calculate3DPosition(	points2d,
														vector<Point3D>{
															Point3D(0., 0., 0.),
															Point3D(_settings.QRSIDE_LENGTH,0.,0.),
															Point3D(_settings.QRSIDE_LENGTH,_settings.QRSIDE_LENGTH,0.),
															Point3D(0.,_settings.QRSIDE_LENGTH,0.)
														}
			);
			break;
		case LabelType::HZL: 
			points3d =  _locator.calculate3DPosition(	points2d,
														vector<Point3D>{
															Point3D(0., 0., 0.),
															Point3D(_settings.HZLSIDE_LENGTH,0.,0.),
															Point3D(_settings.HZLSIDE_LENGTH,_settings.HZLSIDE_LENGTH,0.),
															Point3D(0.,_settings.HZLSIDE_LENGTH,0.)
														}
			);
			break;
		default: 
			ROS_ERROR("PEOS");


	}
	label.set3DPoints(points3d);
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

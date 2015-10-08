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
	static const std::string topic_to_subscribe = loadTopic(_node);
	image_transport::ImageTransport in_(_node);
	_subscriber_to_img_node = in_.subscribe( topic_to_subscribe, 1, &LabelDetector::imageCallback, this);

	loadDetectorSettings(_node);
	_dyn_rec_server.setCallback(boost::bind(&LabelDetector::dynRecCallback, this, _1, _2));

	_3D_points_pub = _node.advertise<visualization_msgs::Marker>("label_detector/label3Dpoints", 1000);

	ROS_WARN("LabelDetector created!");
}



LabelDetector::~LabelDetector(){ 
	ROS_ERROR("LabelDetector deleted!"); 
}

/// Callback function on every new frame.
void LabelDetector::imageCallback(const sensor_msgs::ImageConstPtr& img_sub_msg)
{	
	cv::Mat camera_input;

	try  {
		cv_bridge::CvImageConstPtr cv_ptr;
		cv_ptr = cv_bridge::toCvShare(img_sub_msg,"bgr8");
		camera_input = cv_ptr->image;
		cvWaitKey(30);
	}
	catch (cv_bridge::Exception& e)  {
		ROS_ERROR("[LabelDetector]-imageCallback() : Could not convert (image message) from '%s' to 'bgr8'.", img_sub_msg->encoding.c_str());
	}

	/////////////////////////////////////////////////////
	setInputImage(camera_input);
	detect();
	/////////////////////////////////////////////////////

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

	_locator.setTestImage(_input_image);

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

	// for (int i = 0; i < _labels.size(); i++)
		// calculate3DCoordinates(_labels[i]);


	for_each( begin(_labels), end(_labels), [&] (polymechanon_vision::Label& label) {
			cout << label.getText() << endl;
			calculate3DCoordinates(label);
	});	
	for_each( begin(_labels), end(_labels), [&] (polymechanon_vision::Label& label) {
			auto test2D = label.get2DPoints();
			auto test3D = label.get3DPoints();

			cout << " ======== POINTS " << endl << std::setprecision(4)
				 << " -------[2D]" << endl 
				 << "(" << test2D[0].x << "," << test2D[0].y << ")" << endl
				 << "(" << test2D[1].x << "," << test2D[1].y << ")" << endl
				 << "(" << test2D[2].x << "," << test2D[2].y << ")" << endl
				 << "(" << test2D[3].x << "," << test2D[3].y << ")" << endl
				 << " -------[3D]" << endl
				 << "(" << test3D[0].x << "," << test3D[0].y << "," << test3D[0].z << ")" << endl
				 << "(" << test3D[1].x << "," << test3D[1].y << "," << test3D[1].z << ")" << endl
				 << "(" << test3D[2].x << "," << test3D[2].y << "," << test3D[2].z << ")" << endl
				 << "(" << test3D[3].x << "," << test3D[3].y << "," << test3D[3].z << ")" << endl
				 << " ========================" << endl;

		publish3DCoordinates(label);
	});	

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
	// if ( _labels_contours.size() < 1 )    return false;

	return true;
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


void LabelDetector::calculate3DCoordinates(Label& label)
{	
	vector<Point2D> points2d = label.get2DPoints();
	// cout << points2d.size() << endl;
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
			label.set3DPoints(points3d);
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
			label.set3DPoints(points3d);
			break;
		default: 
			ROS_ERROR("PEOS");


	}
}

void LabelDetector::publish3DCoordinates(Label& label)
{
	vector<Point3D> points = label.get3DPoints();
	visualization_msgs::Marker marker_points, marker_line_strip;


	marker_points.header.frame_id = marker_line_strip.header.frame_id = "/camera_frame";
	marker_points.header.stamp = marker_line_strip.header.stamp = ros::Time::now();
	marker_points.ns = "points_and_lines";
	marker_points.action = marker_line_strip.action = visualization_msgs::Marker::ADD;
	marker_points.pose.orientation.w = 1.0;

	marker_points.id = 0;
	marker_line_strip.id = 1;

	// points.type = visualization_msgs::Marker::POINTS;
	marker_points.type = visualization_msgs::Marker::POINTS;
	marker_line_strip.type = visualization_msgs::Marker::LINE_STRIP;


	// POINTS markers use x and y scale for width/height respectively
	marker_points.scale.x = 0.1;
	marker_points.scale.y = 0.1;

	marker_line_strip.scale.x = 0.1;

	marker_line_strip.color.b = 1.0;
	marker_line_strip.color.a = 1.0;


	// Points are green
	marker_points.color.r = 0.2;
	marker_points.color.a = 0.2;

	geometry_msgs::Point p0;
	geometry_msgs::Point p1;
	geometry_msgs::Point p2;
	geometry_msgs::Point p3;

	p0.x=points[0].x*0.01;
	p0.y=points[0].z*0.01;
	p0.z=points[0].y*0.01;
	p1.x=points[1].x*0.01;
	p1.y=points[1].z*0.01;
	p1.z=points[1].y*0.01;
	p2.x=points[2].x*0.01;
	p2.y=points[2].z*0.01;
	p2.z=points[2].y*0.01;
	p3.x=points[3].x*0.01;
	p3.y=points[3].z*0.01;
	p3.z=points[3].y*0.01;

	marker_line_strip.points.push_back(p0);
	marker_line_strip.points.push_back(p1);
	marker_line_strip.points.push_back(p2);
	marker_line_strip.points.push_back(p3);
	marker_line_strip.points.push_back(p0);

	marker_points.points.push_back(p0);
	marker_points.points.push_back(p1);
	marker_points.points.push_back(p2);
	marker_points.points.push_back(p3);

	std_msgs::ColorRGBA red, blue, green, mix;

	red.r = 1.0;
	red.a = 1.0;
	blue.b = 1.0;
	blue.a = 1.0;
	green.g = 1.0;
	green.a = 1.0;
	mix.r = 1.0;
	mix.g = 1.0;
	mix.a = 1.0;

	marker_points.colors.push_back(red);
	marker_points.colors.push_back(green);
	marker_points.colors.push_back(blue);
	marker_points.colors.push_back(mix);


	_3D_points_pub.publish(marker_points);
	_3D_points_pub.publish(marker_line_strip);

}



// Load image input topic's name from parameter server.
// If there is no topic's name on server, take the function's paramteter.
std::string LabelDetector::loadTopic(const ros::NodeHandle& node, std::string topic_name /* = "camera/image_raw" */ ) 
{	
	std::string topic_param_name("label_detector/input_image_topic");

	auto gettopic = [&] (const std::string& name_of_topic_variable) {
		std::string topic_param;
		node.getParam(name_of_topic_variable, topic_param);
		return topic_param;
	};

	std::string topic = node.hasParam(topic_param_name) ? gettopic(topic_param_name) : topic_name;
	ROS_INFO("LabelDetectorNodeHandler: subscribed for image input on \"%s\" ", topic.c_str());
	return topic;
}

// Load parameters and settings from parameter server or set them to default.
// If settings aren't on parameter server, set them.
void LabelDetector::loadDetectorSettings(const ros::NodeHandle& node)
{	
	DetectorSettings settings;
	// DetectorSettings default values: 
	// DEBUGGING(false), QR_ENABLED(true), HZL_ENABLED(true), QRSIDE_LENGTH(182)
	bool other_settings_loaded = false;

	if (node.getParam("label_detector/Debugging", settings.DEBUGGING))  other_settings_loaded = true;
    else  node.setParam("label_detector/Debugging", settings.DEBUGGING);
    ROS_INFO("LabelDetectorNodeHandler: [DEBUGGING]-%s ", settings.DEBUGGING?"ON":"OFF" );

    // QR CONFIGURATION ////////////////////////////////////////////////////////
	if (!node.getParam("label_detector/QR_Switch", settings.QR_ENABLED))  other_settings_loaded = true;
    else  node.setParam("label_detector/QR_Switch", settings.QR_ENABLED);
	ROS_INFO("LabelDetectorNodeHandler: [QR DETECTION]-%s ", settings.QR_ENABLED?"ON":"OFF" );

    if (node.getParam("label_detector/QR_Debugging", settings.QR_DBG_ENABLED))  other_settings_loaded = true;
    else  node.setParam("label_detector/QR_Debugging", settings.QR_DBG_ENABLED);
    ROS_INFO("LabelDetectorNodeHandler: [QR DEBUGGING]-%s ", settings.QR_DBG_ENABLED?"ON":"OFF" );

	if (node.getParam("label_detector/QR_Canny_par1", settings.QR_CANNY_PAR1))  other_settings_loaded = true;
    else  node.setParam("label_detector/QR_Canny_par1", settings.QR_CANNY_PAR1);
    ROS_INFO("LabelDetectorNodeHandler: [QR_CANNY_PAR1]-%d ", settings.QR_CANNY_PAR1);

	if (node.getParam("label_detector/QR_Canny_par2", settings.QR_CANNY_PAR2))  other_settings_loaded = true;
    else  node.setParam("label_detector/QR_Canny_par2", settings.QR_CANNY_PAR2);
    ROS_INFO("LabelDetectorNodeHandler: [QR_CANNY_PAR2]-%d ", settings.QR_CANNY_PAR2);

    // HZL CONFIGURATION ////////////////////////////////////////////////////////
	if (node.getParam("label_detector/HZL_Switch", settings.HZL_ENABLED))  other_settings_loaded = true;
    else  node.setParam("label_detector/HZL_Switch", settings.HZL_ENABLED);
    ROS_INFO("LabelDetectorNodeHandler: [HZL DETECTION]-%s ", settings.HZL_ENABLED?"ON":"OFF" );

    if (node.getParam("label_detector/HZL_Debugging", settings.HZL_DBG_ENABLED))  other_settings_loaded = true;
    else  node.setParam("label_detector/HZL_Debugging", settings.HZL_DBG_ENABLED);
    ROS_INFO("LabelDetectorNodeHandler: [HZL DEBUGGING]-%s ", settings.HZL_DBG_ENABLED?"ON":"OFF" );

	if (node.getParam("label_detector/Hzl_Canny_par1", settings.HZL_CANNY_PAR1))  other_settings_loaded = true;
    else  node.setParam("label_detector/Hzl_Canny_par1", settings.HZL_CANNY_PAR1);
    ROS_INFO("LabelDetectorNodeHandler: [HZL_CANNY_PAR1]-%d ", settings.HZL_CANNY_PAR1);

	if (node.getParam("label_detector/Hzl_Canny_par2", settings.HZL_CANNY_PAR2))  other_settings_loaded = true;
    else  node.setParam("label_detector/Hzl_Canny_par2", settings.HZL_CANNY_PAR2);
    ROS_INFO("LabelDetectorNodeHandler: [HZL_CANNY_PAR2]-%d ", settings.HZL_CANNY_PAR2);
    
	if (node.getParam("label_detector/Hzl_matching_method", settings.HZL_MATCHING_METHOD))  other_settings_loaded = true;
    else  node.setParam("label_detector/Hzl_matching_method", settings.HZL_MATCHING_METHOD);
    ROS_INFO("LabelDetectorNodeHandler: [HZL_MATCHING_METHOD]-%d ", settings.HZL_MATCHING_METHOD);

	if (node.getParam("label_detector/Hzl_template_matching_method", settings.HZL_TEMPLATE_MATCHING_METHOD))  other_settings_loaded = true;
    else  node.setParam("label_detector/Hzl_template_matching_method", settings.HZL_TEMPLATE_MATCHING_METHOD);
    ROS_INFO("LabelDetectorNodeHandler: [HZL_TEMPLATE_MATCHING_METHOD]-%d ", settings.HZL_TEMPLATE_MATCHING_METHOD);

	if (node.getParam("label_detector/Hzl_enable_color_matching", settings.ENABLE_COLOR_MATCHING))  other_settings_loaded = true;
    else  node.setParam("label_detector/Hzl_enable_color_matching", settings.ENABLE_COLOR_MATCHING);
    ROS_INFO("LabelDetectorNodeHandler: [COLOR_MATCHING]-%s ", settings.ENABLE_COLOR_MATCHING?"ON":"OFF" );

    if (node.getParam("label_detector/Loc_Debugging", settings.ENABLE_COLOR_MATCHING))  other_settings_loaded = true;
    else  node.setParam("label_detector/Loc_Debugging", settings.ENABLE_COLOR_MATCHING);
    ROS_INFO("LabelDetectorNodeHandler: [COLOR_MATCHING]-%s ", settings.ENABLE_COLOR_MATCHING?"ON":"OFF" );

    if (node.getParam("label_detector/Loc_Debugging", settings.LOC_DBG_ENABLED))  other_settings_loaded = true;
    else  node.setParam("label_detector/Loc_Debugging", settings.LOC_DBG_ENABLED);
    ROS_INFO("LabelDetectorNodeHandler: [LOCALIZING DEBUGGING]-%s ", settings.LOC_DBG_ENABLED?"ON":"OFF" );

    if (node.getParam("label_detector/QR_side_legth", settings.QRSIDE_LENGTH))  other_settings_loaded = true;
    else  node.setParam("label_detector/QR_side_legth", settings.QRSIDE_LENGTH);
    ROS_INFO("LabelDetectorNodeHandler: [QR_SIDE_LENGTH]-%d ", settings.QRSIDE_LENGTH);

    if (node.getParam("label_detector/HZL_side_legth", settings.HZLSIDE_LENGTH))  other_settings_loaded = true;
    else  node.setParam("label_detector/HZL_side_legth", settings.HZLSIDE_LENGTH);
    ROS_INFO("LabelDetectorNodeHandler: [HZL_SIDE_LENGTH]-%d ", settings.HZLSIDE_LENGTH);

    if (other_settings_loaded)  setSettings(settings);
}

// Dynamic reconfigure using command line reconfiguration or rqt-plugin
void LabelDetector::dynRecCallback(label_detector::LabelDetectorConfig &config, uint32_t level)
{	
	DetectorSettings settings;
    ROS_INFO("LabelDetector -- Reconfigure Request: \n\t[DEBUGGING]-%s \n\t[QR DETECTION]-%s \n\t[QR DEBUGGING]-%s  \n\t[QR_CANNY_PAR1]-%d \n\t[QR_CANNY_PAR2]-%d \n\t[HZL DETECTION]-%s \n\t[HZL DEBUGGING]-%s \n\t[HZL_CANNY_PAR1]-%d \n\t[HZL_CANNY_PAR2]-%d \n\t[HZL_MATCHING_METHOD]-%d \n\t[HZL_TEMPLATE_MATCHING_METHOD]-%d \n\t[COLOR_MATCHING]-%s \n\t[LOC DEBUGGING]-%s \n\t[LOC_METHOD]-%d \n\t[QR SIDE LENGTH]-%d \n\t[HZL SIDE LENGTH]-%d ",
                config.Debugging?"ON":"OFF",
                config.QR_Switch?"ON":"OFF",
                config.QR_Debugging?"ON":"OFF",
                config.QR_Canny_par1,
                config.QR_Canny_par2,
                config.HZL_Switch?"ON":"OFF",
                config.HZL_Debugging?"ON":"OFF",
                config.Hzl_Canny_par1,
                config.Hzl_Canny_par2,
                config.Hzl_matching_method,
                config.Hzl_template_matching_method,
                config.Hzl_enable_color_matching?"ON":"OFF",
                config.Loc_Debugging?"ON":"OFF",
                config.Localizing_method,
                config.QR_side_legth,
                config.HZL_side_legth
                );

    settings.DEBUGGING = config.Debugging;
    // QR CONFIGURATION
    settings.QR_ENABLED = config.QR_Switch;
    settings.QR_DBG_ENABLED = config.QR_Debugging;
    settings.QR_CANNY_PAR1 = config.QR_Canny_par1;
    settings.QR_CANNY_PAR2 = config.QR_Canny_par2;

    // HZL CONFIGURATION
    settings.HZL_ENABLED = config.HZL_Switch;
    settings.HZL_DBG_ENABLED = config.HZL_Debugging;
    settings.HZL_CANNY_PAR1 = config.Hzl_Canny_par1;
    settings.HZL_CANNY_PAR2 = config.Hzl_Canny_par2;
    settings.HZL_MATCHING_METHOD = config.Hzl_matching_method;
    settings.HZL_TEMPLATE_MATCHING_METHOD = config.Hzl_template_matching_method;
    settings.ENABLE_COLOR_MATCHING = config.Hzl_enable_color_matching;

    // LOCALIZING CONFIGURATION
    settings.LOC_DBG_ENABLED = config.Loc_Debugging;
    settings.LOCALIZING_METHOD = config.Localizing_method;
    settings.QRSIDE_LENGTH = config.QR_side_legth;
    settings.HZLSIDE_LENGTH = config.HZL_side_legth;

    setSettings(settings);
}

////////// Debugging //////////

void LabelDetector::drawInfo()
{	
	if (_input_image != nullptr ) {
		std::ostringstream text;
		text << "Labels detected: " << _labels.size();	
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

} // "namespace polymechanon_vision"
#include "victim_detector/c_object_detector.h"

using std::string;

namespace polymechanon_vision {

ObjectDetector::ObjectDetector()
{
	static const std::string topic_to_subscribe = loadTopic(_node);
	image_transport::ImageTransport in_(_node);
	_subscriber_to_img_node = in_.subscribe( topic_to_subscribe, 1, &ObjectDetector::imageCallback, this);

	loadDetectorSettings(_node);
	_dyn_rec_server.setCallback(boost::bind(&ObjectDetector::dynRecCallback, this, _1, _2));

	loadCascadeClassifier("/include/haarcascades/haarcascade_frontalface_alt.xml");

	ROS_WARN("ObjectDetector created!");
}

ObjectDetector::~ObjectDetector(){ 
	ROS_ERROR("ObjectDetector deleted!"); 
}


/// Callback function on every new frame.
void ObjectDetector::imageCallback(const sensor_msgs::ImageConstPtr& img_sub_msg)
{	
	cv::Mat camera_input;

	try  {
		cv_bridge::CvImageConstPtr cv_ptr;
		cv_ptr = cv_bridge::toCvShare(img_sub_msg,"bgr8");
		camera_input = cv_ptr->image;
		cvWaitKey(30);
	}
	catch (cv_bridge::Exception& e)  {
		ROS_ERROR("[ObjectDetector]-imageCallback() : Could not convert (image message) from '%s' to 'bgr8'.", img_sub_msg->encoding.c_str());
	}

	detectObject(camera_input, _object_cascade);

	cv::imshow("face_detection", camera_input);

}

 void ObjectDetector::detectObject(cv::Mat& image_to_scan, cv::CascadeClassifier& classifier)
{
    std::vector<cv::Rect> objects;
    cv::Mat input_image_gray;

    cvtColor( image_to_scan, input_image_gray, CV_BGR2GRAY );
    equalizeHist( input_image_gray, input_image_gray );

    /* detectMultiScale
    Parameters: 
          x-  cascade – Haar classifier cascade (OpenCV 1.x API only). It can be loaded from XML or YAML file using Load(). When the cascade is not needed anymore, release it using cvReleaseHaarClassifierCascade(&cascade).
        image – Matrix of the type CV_8U containing an image where objects are detected.
        objects – Vector of rectangles where each rectangle contains the detected object.
        scaleFactor – Parameter specifying how much the image size is reduced at each image scale.
        minNeighbors – Parameter specifying how many neighbors each candidate rectangle should have to retain it.
        flags – Parameter with the same meaning for an old cascade as in the function cvHaarDetectObjects. It is not used for a new cascade.
        minSize – Minimum possible object size. Objects smaller than that are ignored.
        maxSize – Maximum possible object size. Objects larger than that are ignored.
    */

    //-- Detect faces
    classifier.detectMultiScale( input_image_gray, objects, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(_minimum_object_size, _minimum_object_size), cv::Size(_maximum_object_size, _maximum_object_size) );
    // face_cascade_.detectMultiScale( frame_gray, faces, 1.1, 3, CV_HAAR_FIND_BIGGEST_OBJECT, cv::Size(min_face_size_, min_face_size_), cv::Size(max_face_size_, max_face_size_) );
    // face_cascade_.detectMultiScale( frame_gray, faces, 1.1, 3, CV_HAAR_DO_CANNY_PRUNING, cv::Size(min_face_size_, min_face_size_), cv::Size(max_face_size_, max_face_size_) );

    for( size_t i = 0; i < objects.size(); i++ )
    {
        cv::Point center( objects[i].x + objects[i].width*0.5, objects[i].y + objects[i].height*0.5 );
        cv::ellipse( image_to_scan, center, cv::Size( objects[i].width*0.5, objects[i].height*0.5), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4, 8, 0 );

        // cv::Mat faceROI = frame_gray( objects[i] );
        
        // std::vector<cv::Rect> eyes;

        //-- In each face, detect eyes	
        // eyes_cascade_.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );

        // for( size_t j = 0; j < eyes.size(); j++ )
        // {
        //     cv::Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
        //     int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
        //     cv::circle( input_frame, center, radius, cv::Scalar( 255, 0, 0 ), 4, 8, 0 );
        // }
    }
    //-- Show what you got
    // imshow( window_name, frame );
 }


// Load image input topic's name from parameter server.
// If there is no topic's name on server, take the function's paramteter.
std::string ObjectDetector::loadTopic(const ros::NodeHandle& node, std::string topic_name /* = "camera/image_raw" */ ) 
{	
	std::string topic_param_name("object_detector/input_image_topic");

	auto gettopic = [&] (const std::string& name_of_topic_variable) {
		std::string topic_param;
		node.getParam(name_of_topic_variable, topic_param);
		return topic_param;
	};

	std::string topic = node.hasParam(topic_param_name) ? gettopic(topic_param_name) : topic_name;
	ROS_INFO("ObjectDetector: subscribed for image input on \"%s\" ", topic.c_str());
	return topic;
}


// Load parameters and settings from parameter server or set them to default.
// If settings aren't on parameter server, set them.
void ObjectDetector::loadDetectorSettings(const ros::NodeHandle& node)
{	
	// DetectorSettings default values: 
	// DEBUGGING(false), QR_ENABLED(true), HZL_ENABLED(true), QRSIDE_LENGTH(182)
	bool other_settings_loaded = false;

	if (node.getParam("object_detector/Debugging", _debugging))  other_settings_loaded = true;
    else  node.setParam("object_detector/Debugging", _debugging);
    ROS_INFO("ObjectDetector: [DEBUGGING]-%s ", _debugging?"ON":"OFF" );

    if (node.getParam("object_detector/MIN_object_size", _minimum_object_size))  other_settings_loaded = true;
    else  node.setParam("object_detector/MIN_object_size", _minimum_object_size);
    ROS_INFO("ObjectDetector: [MIN_OBJECT_SIZE]-%d ", _minimum_object_size);

    if (node.getParam("object_detector/MAX_object_size", _maximum_object_size))  other_settings_loaded = true;
    else  node.setParam("object_detector/MAX_object_size", _maximum_object_size);
    ROS_INFO("ObjectDetector: [MAX_OBJECT_SIZE]-%d ", _maximum_object_size);

    // if (other_settings_loaded)  setSettings(settings);
}

// void ObjectDetector::loadCascadeClassifier(cv::CascadeClassifier &object_detector)
void ObjectDetector::loadCascadeClassifier(const string& path)
{	
	string file_path;
	file_path.append(PACKAGE_PATH);
	file_path.append(path);

	try{
		_object_cascade.load( file_path );
	} catch(...) {
		throw std::runtime_error("[ObjectDetector]-loadCascadeClassifier() : unable to load classification file.");
	}
}

// Dynamic reconfigure using command line reconfiguration or rqt-plugin
void ObjectDetector::dynRecCallback(object_detector::ObjectDetectorConfig &config, uint32_t level)
{	
    ROS_INFO("ObjectDetector -- Reconfigure Request: \n\t[DEBUGGING]-%s \n\t[QR SIDE MIN_OBJECT_SIZE]-%d \n\t[MAX_OBJECT_SIZE SIDE LENGTH]-%d ",
                config.Debugging?"ON":"OFF",
                config.MIN_object_size,
                config.MAX_object_size
                );


    _debugging = config.Debugging;
    _minimum_object_size = config.MIN_object_size;
    _maximum_object_size = config.MAX_object_size;

}

} // "namespace polymechanon_vision"



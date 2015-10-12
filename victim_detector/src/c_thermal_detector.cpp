#include "victim_detector/c_thermal_detector.h"

namespace polymechanon_vision {

ThermalDetector::ThermalDetector()
{
	static const std::string topic_to_subscribe = loadTopic(_node);
	image_transport::ImageTransport in_(_node);
	_subscriber_to_img_node = in_.subscribe( topic_to_subscribe, 1, &ThermalDetector::imageCallback, this);

	loadDetectorSettings(_node);
	_dyn_rec_server.setCallback(boost::bind(&ThermalDetector::dynRecCallback, this, _1, _2));

	setupDetector();

	ROS_WARN("ThermalDetector created!");
}



ThermalDetector::~ThermalDetector(){ 
	ROS_ERROR("ThermalDetector deleted!"); 
}


/// Callback function on every new frame.
void ThermalDetector::imageCallback(const sensor_msgs::ImageConstPtr& img_sub_msg)
{	
	cv::Mat camera_input;

	try  {
		cv_bridge::CvImageConstPtr cv_ptr;
		cv_ptr = cv_bridge::toCvShare(img_sub_msg,"bgr8");
		camera_input = cv_ptr->image;
		cvWaitKey(30);
	}
	catch (cv_bridge::Exception& e)  {
		ROS_ERROR("[ThermalDetector]-imageCallback() : Could not convert (image message) from '%s' to 'bgr8'.", img_sub_msg->encoding.c_str());
	}

	detectBlobs(camera_input);

	cv::imshow("blob_detection", camera_input);


}

void ThermalDetector::setupDetector()
{
	// Setup SimpleBlobDetector parameters.
	cv::SimpleBlobDetector::Params params;
	 
	// Change thresholds
	params.minThreshold = 10;
	params.maxThreshold = 200;
	 
	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 1500;
	 
	// Filter by Circularity
	params.filterByCircularity = true;
	params.minCircularity = 0.1;
	 
	// Filter by Convexity
	params.filterByConvexity = true;
	params.minConvexity = 0.87;
	 
	// Filter by Inertia
	params.filterByInertia = true;
	params.minInertiaRatio = 0.01;

	// Set up detector with params
	_detector = cv::SimpleBlobDetector(params);

}

void ThermalDetector::detectBlobs(cv::Mat& image_to_scan)
{
	std::vector<cv::KeyPoint> keypoints;
	_detector.detect( image_to_scan, keypoints);
	 
	cv::Mat image_with_keypoints;
	drawKeypoints( image_to_scan, keypoints, image_to_scan, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
 


}




// Load image input topic's name from parameter server.
// If there is no topic's name on server, take the function's paramteter.
std::string ThermalDetector::loadTopic(const ros::NodeHandle& node, std::string topic_name /* = "camera/image_raw" */ ) 
{	
	std::string topic_param_name("thermal_detector/input_image_topic");

	auto gettopic = [&] (const std::string& name_of_topic_variable) {
		std::string topic_param;
		node.getParam(name_of_topic_variable, topic_param);
		return topic_param;
	};

	std::string topic = node.hasParam(topic_param_name) ? gettopic(topic_param_name) : topic_name;
	ROS_INFO("ThermalDetector: subscribed for image input on \"%s\" ", topic.c_str());
	return topic;
}


// Load parameters and settings from parameter server or set them to default.
// If settings aren't on parameter server, set them.
void ThermalDetector::loadDetectorSettings(const ros::NodeHandle& node)
{	
	// DetectorSettings default values: 
	// DEBUGGING(false), QR_ENABLED(true), HZL_ENABLED(true), QRSIDE_LENGTH(182)
	bool other_settings_loaded = false;

	if (node.getParam("victim_detector/Debugging", _debugging))  other_settings_loaded = true;
    else  node.setParam("victim_detector/Debugging", _debugging);
    ROS_INFO("ThermalDetector: [DEBUGGING]-%s ", _debugging?"ON":"OFF" );

    if (node.getParam("victim_detector/MIN_blob_size", _minimum_blob_size))  other_settings_loaded = true;
    else  node.setParam("victim_detector/MIN_blob_size", _minimum_blob_size);
    ROS_INFO("ThermalDetector: [MIN_BLOB_SIZE]-%d ", _minimum_blob_size);

    if (node.getParam("victim_detector/MAX_blob_size", _maximum_blob_size))  other_settings_loaded = true;
    else  node.setParam("victim_detector/MAX_blob_size", _maximum_blob_size);
    ROS_INFO("ThermalDetector: [MAX_BLOB_SIZE]-%d ", _maximum_blob_size);

    // if (other_settings_loaded)  setSettings(settings);
}


// Dynamic reconfigure using command line reconfiguration or rqt-plugin
void ThermalDetector::dynRecCallback(thermal_detector::ThermalDetectorConfig &config, uint32_t level)
{	
    ROS_INFO("ThermalDetector -- Reconfigure Request: \n\t[DEBUGGING]-%s \n\t[MIN_OBJECT_SIZE]-%d \n\t[MAX_OBJECT_SIZE]-%d ",
                config.Debugging?"ON":"OFF",
                config.MIN_blob_size,
                config.MAX_blob_size
                );

    _debugging = config.Debugging;
    _minimum_blob_size = config.MIN_blob_size;
    _maximum_blob_size = config.MAX_blob_size;

}

} // "namespace polymechanon_vision"



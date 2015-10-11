#include "victim_detector/c_motion_detector.h"

namespace polymechanon_vision {


MotionDetector::MotionDetector()
{
	static const std::string topic_to_subscribe = loadTopic(_node);
	image_transport::ImageTransport in_(_node);
	_subscriber_to_img_node = in_.subscribe( topic_to_subscribe, 1, &MotionDetector::imageCallback, this);

	loadDetectorSettings(_node);
	_dyn_rec_server.setCallback(boost::bind(&MotionDetector::dynRecCallback, this, _1, _2));


	ROS_WARN("MotionDetector created!");
}



MotionDetector::~MotionDetector(){ 
	ROS_ERROR("MotionDetector deleted!"); 
}


/// Callback function on every new frame.
void MotionDetector::imageCallback(const sensor_msgs::ImageConstPtr& img_sub_msg)
{	
	cv::Mat camera_input;

	try  {
		cv_bridge::CvImageConstPtr cv_ptr;
		cv_ptr = cv_bridge::toCvShare(img_sub_msg,"bgr8");
		camera_input = cv_ptr->image;
		cvWaitKey(30);
	}
	catch (cv_bridge::Exception& e)  {
		ROS_ERROR("[MotionDetector]-imageCallback() : Could not convert (image message) from '%s' to 'bgr8'.", img_sub_msg->encoding.c_str());
	}

}


// Load image input topic's name from parameter server.
// If there is no topic's name on server, take the function's paramteter.
std::string MotionDetector::loadTopic(const ros::NodeHandle& node, std::string topic_name /* = "camera/image_raw" */ ) 
{	
	std::string topic_param_name("victim_detector/input_image_topic");

	auto gettopic = [&] (const std::string& name_of_topic_variable) {
		std::string topic_param;
		node.getParam(name_of_topic_variable, topic_param);
		return topic_param;
	};

	std::string topic = node.hasParam(topic_param_name) ? gettopic(topic_param_name) : topic_name;
	ROS_INFO("MotionDetector: subscribed for image input on \"%s\" ", topic.c_str());
	return topic;
}


// Load parameters and settings from parameter server or set them to default.
// If settings aren't on parameter server, set them.
void MotionDetector::loadDetectorSettings(const ros::NodeHandle& node)
{	
	bool other_settings_loaded = false;

	if (node.getParam("motion_detector/Debugging", _debugging))  other_settings_loaded = true;
    else  node.setParam("motion_detector/Debugging", _debugging);
    ROS_INFO("MotionDetector: [DEBUGGING]-%s ", _debugging?"ON":"OFF" );

    if (node.getParam("motion_detector/Detection_mode", _detection_mode))  other_settings_loaded = true;
    else  node.setParam("motion_detector/Detection_mode", _detection_mode);
    ROS_INFO("MotionDetector: [DETECTION_MODE]-%d ", _detection_mode);

}


// Dynamic reconfigure using command line reconfiguration or rqt-plugin
void MotionDetector::dynRecCallback(motion_detector::MotionDetectorConfig &config, uint32_t level)
{	
    ROS_INFO("MotionDetector -- Reconfigure Request: \n\t[DEBUGGING]-%s \n\t[DETECTION_MODE]-%d ",
                config.Debugging?"ON":"OFF",
                config.Detection_mode
                );


    _debugging = config.Debugging;
    _detection_mode = config.Detection_mode;

}

} // "namespace polymechanon_vision"



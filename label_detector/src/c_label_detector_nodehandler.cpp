#include "label_detector/c_label_detector_nodehandler.h"

////////////////////
#include <label_detector/config_PP.h>  //PACKAGE_PATH 
/////////////////////

namespace polymechanon_vision {

LabelDetectorNodeHandler::LabelDetectorNodeHandler()
{	
	static const std::string topic_to_subscribe = loadTopic(_node);
	image_transport::ImageTransport in_(_node);
	_subscriber_to_img_node = in_.subscribe( topic_to_subscribe, 1, &LabelDetectorNodeHandler::imageCallback, this);

	loadDetectorSettings(_node);
	_dyn_rec_server.setCallback(boost::bind(&LabelDetectorNodeHandler::dynRecCallback, this, _1, _2));
}

LabelDetectorNodeHandler::~LabelDetectorNodeHandler(){}

void LabelDetectorNodeHandler::imageCallback(const sensor_msgs::ImageConstPtr& img_sub_msg)
{	
	cv::Mat camera_input;

	try  {
		cv_bridge::CvImageConstPtr cv_ptr;
		cv_ptr = cv_bridge::toCvShare(img_sub_msg,"bgr8");
		camera_input = cv_ptr->image;
		cvWaitKey(30);
	}
	catch (cv_bridge::Exception& e)  {
		ROS_ERROR("LabelDetectorNodeHandler::imageCallback() : Could not convert (image message) from '%s' to 'bgr8'.", img_sub_msg->encoding.c_str());
	}


	/////////////////////////////////////////////////////
	std::string image_file_path(PACKAGE_PATH);
	// std::string rest_path("/test/samples/qr_pic2.png");
	std::string rest_path("/test/samples/qr_pic.jpg");
	// std::string rest_path("/test/samples/qr_pic2.jpg");
	// std::string rest_path("/test/samples/qr_pic4.jpg");
	image_file_path.append(rest_path);

	cv::Mat image_to_test;
	image_to_test = cv::imread(image_file_path);

	_detector.setInputImage(image_to_test);
	_detector.detect();
	/////////////////////////////////////////////////////
	// _detector.setInputImage(camera_input);
	// _detector.detect();
	/////////////////////////////////////////////////////









}

// Load image input topic's name from parameter server.
// If there is no topic's name on server, take the function's paramteter.
std::string LabelDetectorNodeHandler::loadTopic(const ros::NodeHandle& node, std::string topic_name /* = "camera/image_raw" */ ) 
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
void LabelDetectorNodeHandler::loadDetectorSettings(const ros::NodeHandle& node)
{	
	DetectorSettings settings;
	// DetectorSettings default values: 
	// DEBUGGING(false), QR_ENABLED(true), HZL_ENABLED(true), QRSIDE_LENGTH(182)
	bool other_settings_loaded = false;

	if (node.getParam("label_detector/Debugging", settings.DEBUGGING))  other_settings_loaded = true;
    else  node.setParam("label_detector/Debugging", settings.DEBUGGING);
    ROS_INFO("LabelDetectorNodeHandler: [DEBUGGING]-%s ", settings.DEBUGGING?"ON":"OFF" );

	if (!node.getParam("label_detector/QR_Switch", settings.QR_ENABLED))  other_settings_loaded = true;
    else  node.setParam("label_detector/QR_Switch", settings.QR_ENABLED);
	ROS_INFO("LabelDetectorNodeHandler: [QR DETECTION]-%s ", settings.QR_ENABLED?"ON":"OFF" );

	if (node.getParam("label_detector/HZL_Switch", settings.HZL_ENABLED))  other_settings_loaded = true;
    else  node.setParam("label_detector/QR_Switch", settings.HZL_ENABLED);
    ROS_INFO("LabelDetectorNodeHandler: [HZL DETECTION]-%s ", settings.HZL_ENABLED?"ON":"OFF" );

    if (other_settings_loaded)  _detector.setSettings(settings);
}

// Dynamic reconfigure using command line reconfiguration or rqt-plugin
void LabelDetectorNodeHandler::dynRecCallback(label_detector::LabelDetectorConfig &config, uint32_t level)
{	
	DetectorSettings settings;
    ROS_INFO("LabelDetector -- Reconfigure Request: \n\t[DEBUGGING]-%s \n\t[QR DETECTION]-%s \n\t[HZL DETECTION]-%s \n\t[QR-WIDTH] x (mm) ",
                config.Debugging?"ON":"OFF",
                config.QR_Switch?"ON":"OFF",
                config.HZL_Switch?"ON":"OFF");

    settings.DEBUGGING = config.Debugging;
    settings.QR_ENABLED = config.QR_Switch;
    settings.HZL_ENABLED = config.HZL_Switch;

    _detector.setSettings(settings);
}

} // "namespace polymechanon_vision"
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
	_detector.setInputImage(camera_input);
	_detector.detect();
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

    if (other_settings_loaded)  _detector.setSettings(settings);
}

// Dynamic reconfigure using command line reconfiguration or rqt-plugin
void LabelDetectorNodeHandler::dynRecCallback(label_detector::LabelDetectorConfig &config, uint32_t level)
{	

    // ROS_INFO("LabelDetector -- Reconfigure Request: 
    //                     \n\t[DEBUGGING]-%s 
    //                     \n\t[QR DETECTION]-%s 
    //                     \n\t[QR DEBUGGING]-%s 
    //                     \n\t[QR_CANNY_PAR1]-%d 
    //                     \n\t[QR_CANNY_PAR2]-%d 
    //                     \n\t[HZL DETECTION]-%s 
    //                     \n\t[HZL DEBUGGING]-%s 
    //                     \n\t[HZL_CANNY_PAR1]-%d 
    //                     \n\t[HZL_CANNY_PAR2]-%d 
    //                     \n\t[HZL_MATCHING_METHOD]-%d 
    //                     \n\t[HZL_TEMPLATE_MATCHING_METHOD]-%d 
    //                     \n\t[COLOR_MATCHING]-%s 
    //                     \n\t[LOC DEBUGGING]-%s 
    //                     \n\t[LOC_METHOD]-%d ",


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
    settings.LOC_DBG_ENABLED = config.Loc_Debugging;
    settings.LOCALIZING_METHOD = config.Localizing_method;
    settings.QRSIDE_LENGTH = config.QR_side_legth;
    settings.HZLSIDE_LENGTH = config.HZL_side_legth;

    _detector.setSettings(settings);
}

} // "namespace polymechanon_vision"
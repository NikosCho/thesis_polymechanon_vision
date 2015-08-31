#include "label_detector/TEST_c_label_detector_nodehandler.h"
////////////////////
#include <label_detector/config_PP.h>  //PACKAGE_PATH 
/////////////////////
namespace polymechanon_vision {

TestLabelDetectorNodeHandler::TestLabelDetectorNodeHandler(std::string filepath, std::string type)
{	
	static const std::string topic_to_subscribe = loadTopic(_node);
	// image_transport::ImageTransport in_(_node);

	loadDetectorSettings(_node);
	_dyn_rec_server.setCallback(boost::bind(&TestLabelDetectorNodeHandler::dynRecCallback, this, _1, _2));


	if (type == "video" ) _image_video_switch = true;
	else if (type == "picture" ) _image_video_switch = false;

	_file_path = filepath;	
	ROS_ERROR("%s",_file_path.c_str());
}

TestLabelDetectorNodeHandler::~TestLabelDetectorNodeHandler(){}

void TestLabelDetectorNodeHandler::start()
{	
	if ( _image_video_switch ) {

		// std::cout << _file_path << std::endl;
		
		// std::cout << _file_path << std::endl;

		// cv::namedWindow("VIDEO",CV_WINDOW_AUTOSIZE);

		cv::Mat frame;
		cv::VideoCapture capture;
		capture.open( _file_path );

		double fps = capture.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
		capture.set(CV_CAP_PROP_POS_MSEC, 300); //start the video at 300ms

		int video_frames = capture.get(CV_CAP_PROP_FRAME_COUNT);
		// std::cout << video_frames << std::endl;
		ROS_ERROR("------------------------------------------------");
		ROS_ERROR("%d",video_frames);


		static int frame_counter = 1;

		bool bSuccess = false;

		// while( frame_counter < video_frames )
		// {
		while(1) {
			// capture >> frame;

			bSuccess = capture.read(frame); // read a new frame from video
			if ( bSuccess ) {
				frame_counter++;

				// imshow("VIDEO", frame);
				_detector.setInputImage(frame);
				_detector.detect();	


				if(cv::waitKey(70) == 27) {
					frame_counter = video_frames;
					break;
				}
				 // std::cout << "esc key is pressed by user" << std::endl; 
			} else {
				capture.release();
				break;

			}
		} 


		
	}
	else {
		// cv::namedWindow("IMAGE",1);
		cv::Mat image_to_test;
		image_to_test = cv::imread(_file_path);

		// imshow("IMAGE", image_to_test);
		_detector.setInputImage(image_to_test);
		_detector.detect();
		cv::waitKey();
	}


	cv::destroyAllWindows();

}

// Load image input topic's name from parameter server.
// If there is no topic's name on server, take the function's paramteter.
std::string TestLabelDetectorNodeHandler::loadTopic(const ros::NodeHandle& node, std::string topic_name /* = "camera/image_raw" */ ) 
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
void TestLabelDetectorNodeHandler::loadDetectorSettings(const ros::NodeHandle& node)
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

    if (other_settings_loaded)  _detector.setSettings(settings);
}

// Dynamic reconfigure using command line reconfiguration or rqt-plugin
void TestLabelDetectorNodeHandler::dynRecCallback(label_detector::LabelDetectorConfig &config, uint32_t level)
{	
	DetectorSettings settings;
    ROS_INFO("LabelDetector -- Reconfigure Request: \n\t[DEBUGGING]-%s \n\t[QR DETECTION]-%s \n\t[QR_CANNY_PAR1]-%d \n\t[QR_CANNY_PAR2]-%d \n\t[HZL DETECTION]-%s \n\t[HZL_CANNY_PAR1]-%d \n\t[HZL_CANNY_PAR2]-%d \n\t[HZL_MATCHING_METHOD]-%d \n\t[HZL_TEMPLATE_MATCHING_METHOD]-%d \n\t[COLOR_MATCHING]-%s ",
                config.Debugging?"ON":"OFF",
                config.QR_Switch?"ON":"OFF",
                config.QR_Canny_par1,
                config.QR_Canny_par2,
                config.HZL_Switch?"ON":"OFF",
                config.Hzl_Canny_par1,
                config.Hzl_Canny_par2,
                config.Hzl_matching_method,
                config.Hzl_template_matching_method,
                config.Hzl_enable_color_matching?"ON":"OFF"
                );

    settings.DEBUGGING = config.Debugging;
    // QR CONFIGURATION
    settings.QR_ENABLED = config.QR_Switch;
    settings.QR_CANNY_PAR1 = config.QR_Canny_par1;
    settings.QR_CANNY_PAR2 = config.QR_Canny_par2;

    // HZL CONFIGURATION
    settings.HZL_ENABLED = config.HZL_Switch;
    settings.HZL_CANNY_PAR1 = config.Hzl_Canny_par1;
    settings.HZL_CANNY_PAR2 = config.Hzl_Canny_par2;
    settings.HZL_MATCHING_METHOD = config.Hzl_matching_method;
    settings.HZL_TEMPLATE_MATCHING_METHOD = config.Hzl_template_matching_method;
    settings.ENABLE_COLOR_MATCHING = config.Hzl_enable_color_matching;

    _detector.setSettings(settings);
}

} // "namespace polymechanon_vision"
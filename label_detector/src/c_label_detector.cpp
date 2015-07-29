#include "label_detector/c_label_detector.h"

namespace polymechanonvision {

LabelDetector::LabelDetector()
{
	loadSettings(n_, settings_);		// load settings from parameter server (or get some default values)

	image_transport::ImageTransport it_(n_);
	sub_to_imag_ = it_.subscribe(settings_.TOPIC_TO_SUBSCRIBE, 1, &LabelDetector::imageCallback, this);

	dyn_rec_server_.setCallback(boost::bind(&LabelDetector::dynRecCallback, this, _1, _2));
}

LabelDetector::~LabelDetector(){}

void LabelDetector::imageCallback(const sensor_msgs::ImageConstPtr& img_sub_msg)
{	
	try  {
		cv_bridge::CvImageConstPtr cv_ptr;
		cv_ptr = cv_bridge::toCvShare(img_sub_msg,"bgr8");
		setCameraInput(cv_ptr->image);
		// camera_input_ = cv_ptr->image;
		cvWaitKey(30);
	}
	catch (cv_bridge::Exception& e)  {
		ROS_ERROR("LabelDetector: Could not convert (image message) from '%s' to 'bgr8'.", img_sub_msg->encoding.c_str());
	}

	this->detect(sp_camera_input_);
}

void LabelDetector::detect(cv::Mat& camera_input)
{
    static const cv::String input_video_window = "LabelDetector: camera_input_";
    if (settings_.DEBUGGING)  {
        cv::namedWindow(input_video_window,1);
    }

    



	if (settings_.DEBUGGING)  {
		if (!sp_camera_input_->empty())  cv::imshow(input_video_window, *sp_camera_input_);
	}  else  {
		cv::destroyAllWindows();
	}
}

void LabelDetector::setCameraInput(cv::Mat input)
{	
	try  {
		sp_camera_input_ = boost::make_shared<cv::Mat>(input);
	}
	catch ( std::bad_alloc& e)  {
		ROS_ERROR("LabelDetector - setCameraInput(): bad_alloc exception while setting camera input.");
	}
}

// Load parameters and settings from parameter server or set them to default.
void LabelDetector::loadSettings(const ros::NodeHandle& node, VisionSettings& settings)
{	
	if (node.getParam("label_detector/sub_image_topic", settings.TOPIC_TO_SUBSCRIBE)) 
        ROS_DEBUG("LabelDetector: Topic to subscribe (selected from launch file) -> \"%s\" ", settings.TOPIC_TO_SUBSCRIBE.c_str());
    else  settings.TOPIC_TO_SUBSCRIBE = "camera/image_raw";

	if (node.getParam("label_detector/Debugging", settings.DEBUGGING))  
		ROS_DEBUG("LabelDetector: [DEBUGGING]-%s ", settings.DEBUGGING?"ON":"OFF" );
	else  settings.DEBUGGING = false;

	if (node.getParam("label_detector/QR_Switch", settings.QR_ENABLED))  
		ROS_DEBUG("LabelDetector: [QR DETECTION]-%s ", settings.QR_ENABLED?"ON":"OFF" );
	else  settings.QR_ENABLED = false;

	if (node.getParam("label_detector/HZL_Switch", settings.HZL_ENABLED))  
		ROS_DEBUG("LabelDetector: [HZL DETECTION]-%s ", settings.HZL_ENABLED?"ON":"OFF" );
	else  settings.HZL_ENABLED = false;
}

// Dynamic reconfigure using command line reconfiguration or rqt-plugin
void LabelDetector::dynRecCallback(label_detector::LabelDetectorConfig &config, uint32_t level) 
{
    ROS_DEBUG("\n\tLabelDetector -- Reconfigure Request: \n\t\t[DEBUGGING]-%s [QR DETECTION]-%s [HZL DETECTION]-%s \n\t\t[QR-WIDTH] x (mm) ", 
                config.Debugging?"ON":"OFF",
                config.QR_Switch?"ON":"OFF",
                config.HZL_Switch?"ON":"OFF");

    settings_.DEBUGGING = config.Debugging;
    settings_.QR_ENABLED = config.QR_Switch;
    settings_.HZL_ENABLED = config.HZL_Switch;
}

} // "namespace polymechanonvision"
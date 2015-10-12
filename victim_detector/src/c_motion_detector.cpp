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

    static bool FRAMES_CAPTURED = false;
    static bool FRAMES_MESSAGES = true;
    // capture the desired amount of frames (3)
    // ALWAYS keeps the last 3 frames captured, in the right order : oldest, ..., newest (frame)
    cv::cvtColor(camera_input, camera_input, cv::COLOR_BGR2GRAY);
    if ( vectorSequence(camera_input, _frames_sequence, 3) )  {
        FRAMES_CAPTURED = true;
        if (!FRAMES_MESSAGES)  {   
            ROS_INFO("MotionDetection: Enough frames captured..");
            FRAMES_CAPTURED = true; 
            FRAMES_MESSAGES = true;
        }
    } else  {
        FRAMES_CAPTURED = false;
        if (FRAMES_MESSAGES)  {   
            ROS_WARN("MotionDetection: Waiting to capture enough frames..");
            FRAMES_MESSAGES = false;
        }
    }   

    if ( FRAMES_CAPTURED)  {
		detectMotion();
	}
}


void MotionDetector::detectMotion()
{
	switch(_detection_mode)
	{
		case 0: // Stable mode
			stableCameraDetection();
			break;
		case 1: // Moving mode
			movingCameraDetection();
			break;
		default: 
			ROS_ERROR("[MotionDetector]-detectMotion() : unknown mode selected.");
	}

}

void MotionDetector::stableCameraDetection()
{
    cv::Mat d1,d2;
    cv::Mat absdiff_mask,motion_mask;


	cv::absdiff(_frames_sequence[0], _frames_sequence[1], d1);
	cv::absdiff(_frames_sequence[0], _frames_sequence[2], d2);
	cv::bitwise_and(d1, d2, absdiff_mask);

	cv::imshow("d1", d1);
	cv::imshow("d2", d2);
	cv::imshow("absdiff_mask", absdiff_mask);


	switch (_thres_method)  {
		case 0: // SIMPLE:
		cv::threshold(d1, d1, _thres_value, 255,cv::THRESH_BINARY);
		cv::threshold(d2, d2, _thres_value, 255,cv::THRESH_BINARY);
		cv::threshold(absdiff_mask, absdiff_mask, _thres_value, 255,cv::THRESH_BINARY);
	break;
	case 1: // ADAPTIVE_GAUSSIAN:
		cv::adaptiveThreshold(d1, d1, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY , 5,0);
		cv::adaptiveThreshold(d2, d2, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY , 5,0);
		cv::adaptiveThreshold(absdiff_mask, absdiff_mask, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY , 5,0);
		break;
	case 2: // ADAPTIVE_MEAN:
		cv::adaptiveThreshold(d1, d1, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 5,0);
		cv::adaptiveThreshold(d2, d2, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 5,0);
		cv::adaptiveThreshold(absdiff_mask, absdiff_mask, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 5,0);
		break;
	case 3: // OTSU:
		cv::threshold(d1, d1, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
		cv::threshold(d2, d2, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
		cv::threshold(absdiff_mask, absdiff_mask, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
		break;
	default:
		cv::threshold(d1, d1, _thres_value, 255,cv::THRESH_BINARY);
		cv::threshold(d2, d2, _thres_value, 255,cv::THRESH_BINARY);
		cv::threshold(absdiff_mask, absdiff_mask, _thres_value, 255,cv::THRESH_BINARY);
	}
	
    cv::bitwise_and(d1, d2, motion_mask);
	cv::imshow("motion_mask", motion_mask);


    cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 3, 3 ) );
	cv::erode( motion_mask, motion_mask, element );
	cv::dilate( motion_mask, motion_mask, element ); 

	cv::imshow("Thresholded d1", d1);
	cv::imshow("Thresholded d2", d2);
	cv::imshow("Thresholded absdiff_mask", absdiff_mask);
	cv::imshow("MORPHOPENED motion_mask", motion_mask);


}


void MotionDetector::movingCameraDetection()
{

	
}

template<typename T>
void MotionDetector::pop_front(std::vector<T>& vec)
{
	assert(!vec.empty());
	vec.erase(vec.begin());
}

template<typename Ta>
bool MotionDetector::vectorSequence(Ta &input_item, std::vector<Ta> &memory, int multitude)
{
    if(!input_item.empty())  {
        // switch(true)  {
        //     case memory.size()==0:
        //         memory.push_back(input_item);
        //         break;
        //     case memory.size() < multitude:
        //         memory.push_back(input_item);
        //         break;
        //     case memory.size() == multitude:
        //         pop_front(memory);
        //         memory.push_back(input_item);
        //         break;
        //     default:
        //         memory.clear();
        //         memory.push_back(input_item);
        // }

        if (memory.size()==0)  {
            memory.push_back(input_item);
        }  else if (memory.size() < multitude)  {
            memory.push_back(input_item);
        }  else if (memory.size() == multitude)  {
            pop_front(memory);
            memory.push_back(input_item);
        }  else  {
            memory.clear();
            memory.push_back(input_item);
        }
    }
    // return memory.size();

    if(memory.size() < multitude )
    	return false;
    else
    	return true;
}

// Load image input topic's name from parameter server.
// If there is no topic's name on server, take the function's paramteter.
std::string MotionDetector::loadTopic(const ros::NodeHandle& node, std::string topic_name /* = "camera/image_raw" */ ) 
{	
	std::string topic_param_name("motion_detector/input_image_topic");

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

    if (node.getParam("motion_detector/Thres_method", _thres_method))  other_settings_loaded = true;
    else  node.setParam("motion_detector/Thres_method", _thres_method);
    ROS_INFO("MotionDetector: [THRESHOLD_METHOD]-%d ", _thres_method);

    if (node.getParam("motion_detector/Thres_value", _thres_value))  other_settings_loaded = true;
    else  node.setParam("motion_detector/Thres_value", _thres_value);
    ROS_INFO("MotionDetector: [THRESHOLD_METHOD]-%d ", _thres_value);

}


// Dynamic reconfigure using command line reconfiguration or rqt-plugin
void MotionDetector::dynRecCallback(motion_detector::MotionDetectorConfig &config, uint32_t level)
{	
    ROS_INFO("MotionDetector -- Reconfigure Request: \n\t[DEBUGGING]-%s \n\t[DETECTION_MODE]-%d \n\t[THRESHOLD_METHOD]-%d n\t[THRESHOLD_VALUE]-%d ",
                config.Debugging?"ON":"OFF",
                config.Detection_mode,
                config.Thres_method,
                config.Thres_value
                );


    _debugging = config.Debugging;
    _detection_mode = config.Detection_mode;
    _thres_method = config.Thres_method;
    _thres_value = config.Thres_value;
}

} // "namespace polymechanon_vision"



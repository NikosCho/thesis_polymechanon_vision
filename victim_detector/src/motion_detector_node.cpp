#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "label_detector/c_motion_detector.h"

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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motion_detector");
	
	ros::NodeHandle node;
	image_transport::ImageTransport in(node);
	image_transport::Subscriber subscriber_to_img_node;

	subscriber_to_img_node = in.subscribe( "/usb_camera/image_raw", 1, &imageCallback, this);

	ros::spin();


}
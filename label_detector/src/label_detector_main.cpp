#include "ros/ros.h"
#include "label_detector/c_label_detector_nodehandler.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "label_detector");

	polymechanon_vision::LabelDetectorNodeHandler my_detector;

	ros::spin();
}
#include <ros/ros.h>
#include "victim_detector/c_object_detector.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "object_detector");
	polymechanon_vision::ObjectDetector my_detector;
	ros::spin();
}
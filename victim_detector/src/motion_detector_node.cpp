#include <ros/ros.h>
#include "victim_detector/c_motion_detector.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motion_detector");
	polymechanon_vision::MotionDetector my_detector;
	ros::spin();
}
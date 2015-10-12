#include "ros/ros.h"
#include "res_rob_vision/c_circle_detection.h"

int main(int argc, char **argv)
{ 
	ros::init(argc, argv, "circle_detection");

	CircleDetection configurator;

	ros::spin();
}


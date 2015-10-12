#include "ros/ros.h"
#include "res_rob_vision/c_borders_detection.h"

int main(int argc, char **argv)
{ 
	ros::init(argc, argv, "area_borders");

	BordersDetection configurator;

	ros::spin();
}


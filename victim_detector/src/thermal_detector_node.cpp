#include <ros/ros.h>
#include "victim_detector/c_thermal_detector.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "thermal_detector");
	polymechanon_vision::ThermalDetector my_detector;
	ros::spin();
}
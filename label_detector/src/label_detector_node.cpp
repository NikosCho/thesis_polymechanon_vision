#include "ros/ros.h"
#include "label_detector/c_label_detector.h"

int main(int argc, char **argv)
{ 
	ros::init(argc, argv, "label_detector");

	polymechanonvision::LabelDetector mydetector;

	ros::spin();
}

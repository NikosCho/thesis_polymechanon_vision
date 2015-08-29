#include "ros/ros.h"
#include "label_detector/c_label_detector_nodehandler.h"


//////////////////// TEST
#include "label_detector/TEST_c_label_detector_nodehandler.h"
#include <string>
#include <label_detector/config_PP.h>  //PACKAGE_PATH 
/////////////////////

int main(int argc, char **argv)
{
	ros::init(argc, argv, "label_detector");

	polymechanon_vision::LabelDetectorNodeHandler my_detector;



	//////////////////////////////////////////////////////////////////////////






	// std::string filepath = PACKAGE_PATH;
	// // VIDEOS //////////////////////////////////

	// // std::string rest_path("/test/samples/videos/vid1.avi");
	// // std::string rest_path("/test/samples/videos/vid2.avi");
	// // std::string rest_path("/test/samples/videos/vid3.avi");
	// // std::string rest_path("/test/samples/videos/isia.avi");
<<<<<<< HEAD
	// // std::string rest_path("/test/samples/videos/gyrw2.avi");
=======
	// std::string rest_path("/test/samples/videos/gyrw2.avi");
>>>>>>> 38c51a172c71bb43ad2a43765df732668b7b1668
	// // std::string rest_path("/test/samples/videos/gyrw.avi");
	// // std::string rest_path("/test/samples/videos/error.avi");


	// // PICTURES ////////////////////////////////

	// // std::string rest_path("/test/samples/pictures/PLAIN_QR.png");
	// // std::string rest_path("/test/samples/pictures/qr_pic.jpg");
	// // std::string rest_path("/test/samples/pictures/qr_pic2.jpg");
	// // std::string rest_path("/test/samples/pictures/qr_pic4.jpg");
	// // std::string rest_path("/test/samples/pictures/qr_pic5.jpg");
<<<<<<< HEAD
	// // std::string rest_path("/test/samples/pictures/hzl_1.jpg");
=======
>>>>>>> 38c51a172c71bb43ad2a43765df732668b7b1668

	// ////////////////////////////////////////////

	// // COLOR  PICTURES ////////////////////////////////

<<<<<<< HEAD
	// // std::string rest_path("/test/samples/pictures/colors/red.png");
	// // std::string rest_path("/test/samples/pictures/colors/orange.png");
	// // std::string rest_path("/test/samples/pictures/colors/blue.png");
	// // std::string rest_path("/test/samples/pictures/colors/yellow.png");
	// std::string rest_path("/test/samples/pictures/colors/green.png");

	// ////////////////////////////////////////////


	// filepath.append(rest_path);	
	// ROS_ERROR("%s",filepath.c_str());

	// // polymechanon_vision::TestLabelDetectorNodeHandler my_detector(filepath,"video");
	// polymechanon_vision::TestLabelDetectorNodeHandler my_detector(filepath,"picture");
=======
	// filepath.append(rest_path);	
	// ROS_ERROR("%s",filepath.c_str());

	// polymechanon_vision::TestLabelDetectorNodeHandler my_detector(filepath,"video");
	// // polymechanon_vision::TestLabelDetectorNodeHandler my_detector(filepath,"picture");
>>>>>>> 38c51a172c71bb43ad2a43765df732668b7b1668
	// my_detector.start();







	/////////////////////////////////////////////////////////////////////////

	ros::spin();
	// ros::spinOnce();
}
/*********************************************************************
* test_something.cpp
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, University of Patras
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of University of Patras nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Nikos Cholis
*********************************************************************/

#include "ros/ros.h"
#include <gtest/gtest.h>
#include <vector>
#include "label_detector/c_label.h"
#include "label_detector/c_label_detector.h"
#include <label_detector/config_PP.h>  //PACKAGE_PATH 

// $ rostest label_detector run_label_detector_node.launch 

// $ catkin_make run_tests_label_detector


static cv::String getFilePath(const std::string& file, int type = 0) // 0-picture 1-video
{
	std::string return_path(PACKAGE_PATH);
	return_path.append("/test/samples/");

	if ( type == 0)    return_path.append("pictures/tests/");
	else if ( type == 1)    return_path.append("videos/tests/");
	return_path.append(file);
	return return_path;
}

static cv::String getTestResultsPath(const std::string& file, int type = 0) // 0-picture 1-video
{
	std::string return_path(PACKAGE_PATH);
	return_path.append("/test/samples/");
	if ( type == 0)    return_path.append("pictures/results/");
	else if ( type == 1)    return_path.append("videos/results/");
	return_path.append("RESULT_OF_");
	return_path.append(file);
	return return_path;
}

// TEST(Label, setID) {
// 	polymechanon_vision::Label label;
// 	EXPECT_EQ(1,label.setID());
// }

// TEST(LabelDetector, detect1) {

// 	std::string file_name = "test1.jpg";
// 	std::string file_path  = getFilePath(file_name);

// 	cv::Mat image_to_test;
// 	image_to_test = cv::imread(file_path);

// 	polymechanon_vision::LabelDetector detector(image_to_test);

// 	EXPECT_EQ(true,detector.detect());
// 	EXPECT_EQ(1,detector.getNumberOfDetectedLabels());

// 	cv::Mat test_result = detector.getImageOutput();
// 	cv::imwrite(getTestResultsPath(file_name), test_result);
// }

// TEST(LabelDetector, detect2) {

// 	std::string file_name = "test2.jpg";
// 	std::string file_path  = getFilePath(file_name);

// 	cv::Mat image_to_test;
// 	image_to_test = cv::imread(file_path);

// 	polymechanon_vision::LabelDetector detector(image_to_test);

// 	EXPECT_EQ(true,detector.detect());
// 	EXPECT_EQ(1,detector.getNumberOfDetectedLabels());

// 	cv::Mat test_result = detector.getImageOutput();
// 	cv::imwrite(getTestResultsPath(file_name), test_result);
// }

// TEST(LabelDetector, detect3) {

// 	std::string file_name = "test3.jpg";
// 	std::string file_path  = getFilePath(file_name);

// 	cv::Mat image_to_test;
// 	image_to_test = cv::imread(file_path);

// 	polymechanon_vision::LabelDetector detector(image_to_test);

// 	EXPECT_EQ(true,detector.detect());
// 	EXPECT_EQ(1,detector.getNumberOfDetectedLabels());

// 	cv::Mat test_result = detector.getImageOutput();
// 	cv::imwrite(getTestResultsPath(file_name), test_result);
// }

// TEST(LabelDetector, detect4) {

// 	std::string file_name = "test4.jpg";
// 	std::string file_path  = getFilePath(file_name);

// 	cv::Mat image_to_test;
// 	image_to_test = cv::imread(file_path);

// 	polymechanon_vision::LabelDetector detector(image_to_test);

// 	EXPECT_EQ(true,detector.detect());
// 	EXPECT_EQ(1,detector.getNumberOfDetectedLabels());

// 	cv::Mat test_result = detector.getImageOutput();
// 	cv::imwrite(getTestResultsPath(file_name), test_result);
// }


TEST(LabelDetector, detect) {

	std::vector<std::string> file_names = 
	{
		"test1.jpg",
		"test2.jpg",
		"test3.jpg",
		"test4.jpg"
		// "test5.jpg",
		// "test6.jpg",
		// "test7.jpg",
		// "test8.jpg"
	};

	for ( size_t i=0; i < file_names.size(); i++) {

		std::string file_path  = getFilePath(file_names[i]);

		cv::Mat image_to_test;
		image_to_test = cv::imread(file_path);

		polymechanon_vision::LabelDetector detector(image_to_test);

		EXPECT_EQ(true,detector.detect());
		EXPECT_EQ(1,detector.getNumberOfDetectedLabels());

		cv::Mat test_result = detector.getImageOutput();
		cv::imwrite(getTestResultsPath(file_names[i]), test_result);
	}
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "google_test_something");
  return RUN_ALL_TESTS();
}

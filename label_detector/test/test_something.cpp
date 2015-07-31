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
#include "label_detector/c_label.h"
#include "label_detector/c_label_detector.h"
#include <label_detector/config_PP.h>  //PACKAGE_PATH 

static cv::String appendPackagePath(const std::string filepath)
	{
	  std::string return_path(PACKAGE_PATH);
	  return_path.append(filepath);
	  return return_path;
	}

TEST(Label, setID) {
	polymechanonvision::Label label;
	EXPECT_EQ(1,label.setID());
}

TEST(LabelDetector, detect) {


	std::string image_file_path(PACKAGE_PATH);
	std::string rest_path("/test/samples/random_pic.png");
	image_file_path.append(rest_path);

	cv::Mat image_to_test;
	image_to_test = cv::imread(image_file_path);

	polymechanonvision::LabelDetector detector(image_to_test);

	EXPECT_EQ(true,detector.detect());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "google_test_something");
  return RUN_ALL_TESTS();
}

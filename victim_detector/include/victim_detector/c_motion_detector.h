#ifndef MOTION_DETECTOR_H
#define MOTION_DETECTOR_H

#include <iostream>
#include <string>


#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

// Dynamic Reconfigure Headers
#include <dynamic_reconfigure/server.h>
#include <victim_detector/MotionDetectorConfig.h>


namespace polymechanon_vision {

// enum class ThreholdType {
// 	SIMPLE, 
// 	ADAPTIVE_GAUSSIAN, 
// 	ADAPTIVE_MEAN, 
// 	OTSU

// };


class MotionDetector
{
public:
	MotionDetector();
	~MotionDetector();


private:
	// ROS node's setup
	ros::NodeHandle _node;
	image_transport::Subscriber _subscriber_to_img_node;
	dynamic_reconfigure::Server<victim_detector::MotionDetectorConfig> _dyn_rec_server;

	std::vector<cv::Mat> _frames_sequence;

	bool _debugging;
	int _detection_mode;
	int _thres_method;
	int _thres_value;

	// Node's setup
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	std::string loadTopic(const ros::NodeHandle& node, std::string topic_name = "/camera/image_raw");	
	void loadDetectorSettings(const ros::NodeHandle& node);
	// void dynRecCallback(victim_detector::VictimDetectorConfig &config, uint32_t level);
	void dynRecCallback(victim_detector::MotionDetectorConfig &config, uint32_t level);

	void detectMotion(cv::Mat& image_to_draw);
	void stableCameraDetection(cv::Mat& image_to_draw);
	void movingCameraDetection(cv::Mat& image_to_draw);


	template<typename T>
	void pop_front(std::vector<T>& vec);

	template<typename Ta>
	bool vectorSequence(Ta &input_item, std::vector<Ta> &memory, int multitude);



	void drawArrow(cv::Mat &image, cv::Point p, cv::Point q, cv::Scalar color, int arrowMagnitude, int thickness, int line_type, int shift);
	int angleCategorize(const double &value, const double &x1, const double &x2);
	bool calculateLenAng(const std::vector<cv::Point2f> &start,const std::vector<cv::Point2f> &end, std::vector<double> &length, std::vector<double> &angle);
	std::vector< std::vector<cv::Point2f> > DBSCAN_points(std::vector<cv::Point2f> &points, float eps, int minPts);
	std::vector<int> regionQuery(std::vector<cv::Point2f> &points, cv::Point2f &point, float eps);
	void phaseCorrelation(cv::Mat &frame, double &radius, double &angle, double &shift_x);







};

} // "namespace polymechanon_vision"
#endif // MOTION_DETECTOR_H
















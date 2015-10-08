#ifndef LOCATOR_H
#define LOCATOR_H

#include <iostream>
#include <string>
#include <stdexcept> // std::runtime_error

#include "ros/ros.h"
#include <opencv2/opencv.hpp> // cv::Point_

#include <label_detector/config_PP.h>

typedef cv::Point2f Point2D;
typedef cv::Point3f Point3D;
using std::shared_ptr;

namespace polymechanon_vision {

class Locator
{
public:
	Locator();
	~Locator();

	bool setParameters(bool debugging, int loc_method);

	std::vector< Point3D > calculate3DPosition(const std::vector<Point2D>& object2Dpoints, const std::vector<Point3D>& real3Dpoints);

	void set2DPoints(std::vector< Point2D >& points);
	std::vector< Point2D > get2DPoints() const;
	void set3DPoints(std::vector< Point3D >& points);
	std::vector< Point3D > get3DPoints() const;

	 void setTestImage(const shared_ptr<cv::Mat> input_image); 


private:
	cv::Mat _camera_matrix;
	cv::Mat _distortion_matrix;
	bool _calibration_loaded;

	cv::Mat _test_image;

	bool _debugging;
	int _localizing_method;

	std::vector< Point2D > _points_2D;
	std::vector< Point3D > _points_3D;



	void loadCalibration(const std::string& filename);
	// std::vector< Point3D > calculate3DPosition(const std::vector<Point2D>& object2Dpoints, const std::vector<Point3D>& real3Dpoints);
	void translateCoordinates(const std::vector<Point2D>& object2Dpoints, const std::vector<Point3D>& real3Dpoints, std::vector<Point3D>& object3Dpoints);

	void calibrateImage(cv::Mat& image);

	double findDiagonal(double width,double height);
	float cv_distance(cv::Point2f P, cv::Point2f Q);

	template<typename Ta, typename Tb>
	double calculateDistance(const Ta& point1, const Tb& point2);
	template<typename T>
	double calculateSlope(const T& pointA, const T& pointB);

	// Debugging 
	void draw3Daxes(cv::Mat& image, std::vector<Point3D>& corners, std::vector<Point3D>& corners_axis);

};

} // "namespace polymechanon_vision"
#endif // LOCATOR_H

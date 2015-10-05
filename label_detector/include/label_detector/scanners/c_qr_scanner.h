#ifndef QR_SCANNER_H
#define QR_SCANNER_H

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <memory>
#include <algorithm>

#include <opencv2/opencv.hpp>

#include <zbar.h> //for zbar

/// Angle calculation //
#include <math.h>
#define _USE_MATH_DEFINES
/////////////////////////

// Other class dependecies
#include "label_detector/c_scanner.h"
#include "label_detector/c_label.h"
// #include "label_detector/c_label_detector.h"


typedef cv::Point ContourPoint;
// typedef cv::Point2f Point2D;

namespace polymechanon_vision {

enum class QrOrientation
{
	NORTH,
	WEST, 
	SOUTH,
	EAST
};

struct QrMarker 
{
	std::vector<Point2D> contour;
	Point2D mass_center;
	QrMarker() {}
	QrMarker(const std::vector<Point2D> contour, const Point2D mass_center): contour(contour), mass_center(mass_center) {}
};


class QrScanner : public Scanner
{	

public:
	QrScanner(shared_ptr<cv::Mat> input_image = nullptr);
	virtual ~QrScanner();

	LabelType getType() const;
	/* void setImageToScan(const shared_ptr<cv::Mat> input_image); */
	bool scan();
	// std::vector<std::vector<Point2D> > getDetectedLabels();
	// std::vector<std::pair<std::vector<Point2D>, std::string> > getDetectedLabels();
	std::vector<polymechanon_vision::Label> getDetectedLabels();

	bool setParameters(bool debugging, int par1, int par2);

	///////////////////// Debugging Functions /////////////////////
	bool drawDetectedLabels(shared_ptr<cv::Mat> inputimage);
	bool drawDetectedLabels(cv::Mat &inputimage);
	cv::Mat getImageOfDetectedLabels(const cv::Mat &inputimage);


private:
	/* shared_ptr<cv::Mat> _image_to_scan */ 
	static const LabelType _type = LabelType::QRCODE;
	// std::vector<std::vector<Point2D> > _detected_labels;
	// std::vector<std::pair<std::vector<Point2D>, std::string> > _detected_labels;
	// std::vector<Label> _labels_detected;
	std::vector<polymechanon_vision::Label> _labels_detected;

	zbar::ImageScanner scanner_;

	int _canny_param1;
	int _canny_param2;
	bool _debugging;

	std::vector<QrMarker> qr_markers_;

	cv::Mat _testing_image;

	// Scanning functions
	std::vector<std::vector<ContourPoint> > findAlignmentMarkers(const cv::Mat& image);
	std::vector<Point2D> findContoursMassCenters(const std::vector<std::vector<ContourPoint> >& contours_vector);
	std::vector<std::vector<int> > clusterMarkers(const std::vector<Point2D>& contours_mass_centers);
	std::vector<Point2D> getMassCenters(const std::vector<int>& contours_by_id, const std::vector<Point2D>& mass_centers);
	std::vector<std::vector<ContourPoint> > getContours(const std::vector<int>& contours_by_id, const std::vector<std::vector<ContourPoint> >& contours);
	std::vector<QrMarker> getMarkers(const std::vector<int>& contours_by_id, const std::vector<std::vector<ContourPoint> >& contours, const std::vector<Point2D>& mass_centers);
	bool sortMarkers(std::vector<QrMarker>& markers);
	int individualizeTopLeftMarker(std::vector<QrMarker>& markers);
	int individualizeTopLeftMarkerbyDiagonal(std::vector<QrMarker>& markers);
	int individualizeTopLeftMarkerbyAngle(std::vector<QrMarker>& markers);
	void sortMarkersVertices(std::vector<QrMarker>& markers);
	std::vector<Point2D> findSquare(const std::vector<QrMarker>& markers);
	// std::string translate(const cv::Mat &gray_input_image,const std::vector<Point2D>& qr_vertices);
	bool translate(const cv::Mat &gray_input_image,const std::vector<Point2D>& qr_vertices, std::string& message);

	template<typename Ta, typename Tb>
	double calculateDistance(const Ta& point1, const Tb& point2);
	template<typename Ta, typename Tb>
	Ta findMiddlePoint(const Ta& pointA, const Tb& pointB);
	template<typename T>
	double calculateSlope(const T& pointA, const T& pointB);
	template<typename T>
	double calculateSlope(const std::vector<T>& line);
	template<typename T>
	double calculateAngle(const T& pointA, const T& pointB, const T& pointC);
	template<typename T>
	T findIntersection(const std::vector<T>& lineA, const std::vector<T>& lineB);
	template<typename T, typename Tc>
	void calculateCoefficients(const T& pointA, const T& pointB, Tc &a, Tc &b, Tc &c);
	template<typename T, typename Tc>
	void calculateCoefficients(const std::vector<T>& line, Tc &a, Tc &b, Tc &c);
	template<typename T>
	T findProjection(const T& point, const T& point_of_lineA, const T& point_of_lineB);
	template<typename T>
	bool pointLiesBetweenLines(const T& point, std::vector<T> lineA, std::vector<T> lineB);

	// TEST ////////////

	cv::Scalar randomColor();
	void drawContours(cv::Mat &inputimage, const std::vector<std::vector<ContourPoint> >& contours );
	void drawMassCenters(cv::Mat &inputimage, const std::vector<Point2D>& mass_centers );
	void drawMarkerVertices(cv::Mat &inputimage, const std::vector<QrMarker>& markers );
	void drawVertices(cv::Mat &inputimage, const std::vector<Point2D>& vertices );
	void drawLines(cv::Mat &inputimage, const std::vector<QrMarker>& markers );
	void drawLine(shared_ptr<cv::Mat> inputimage, const std::vector<Point2D>& line, cv::Scalar color );
	void drawLine(cv::Mat &inputimage, const std::vector<Point2D>& line, cv::Scalar color );
	void drawSquare(cv::Mat &inputimage, const std::vector<Point2D>& vertices, const std::vector<ContourPoint>& top_marker);
	///////////////////
};

} // "namespace polymechanon_vision"
#endif // QR_SCANNER_H
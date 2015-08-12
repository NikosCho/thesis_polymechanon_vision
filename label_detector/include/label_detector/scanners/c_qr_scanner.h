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


using std::vector;
using std::pair; //?
using std::make_pair; //?
using std::shared_ptr;
using std::all_of;
using std::for_each;
using std::distance;
using std::rotate;
using std::max_element;


using std::cout;
using std::endl;

typedef cv::Point ContourPoint;
// typedef cv::Point2f Point2D;
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
	vector<ContourPoint> contour;
	Point2D mass_center;
	QrMarker(const vector<ContourPoint> contour, const Point2D mass_center): contour(contour), mass_center(mass_center) {}
};


class QrScanner : public Scanner
{	

public:
	QrScanner(shared_ptr<cv::Mat> input_image = nullptr);
	virtual ~QrScanner();

	LabelType getType() const;
	/* void setImageToScan(const shared_ptr<cv::Mat> input_image); */
	bool scan();
	vector<vector<Point2D> > getDetectedLabels();

	bool setParameters(int par1, int par2);

	///////////////////// Debugging Functions /////////////////////
	bool drawDetectedLabels(shared_ptr<cv::Mat> inputimage);
	bool drawDetectedLabels(cv::Mat &inputimage);
	cv::Mat getImageOfDetectedLabels(const cv::Mat &inputimage);


private:
	/* shared_ptr<cv::Mat> _image_to_scan */ 
	static const LabelType _type = LabelType::QRCODE;
	vector<vector<Point2D> > _detected_labels;

	zbar::ImageScanner scanner_;

	int _canny_param1;
	int _canny_param2;

	vector<QrMarker> qr_markers_;

	// Scanning functions
	vector<vector<ContourPoint> > findAlignmentMarkers(const cv::Mat& image);
	vector<Point2D> findContoursMassCenters(const vector<vector<ContourPoint> >& contours_vector);
	vector<vector<int> > clusterMarkers(const vector<Point2D>& contours_mass_centers);
	vector<Point2D> getMassCenters(const vector<int>& contours_by_id, const vector<Point2D>& mass_centers);
	vector<vector<ContourPoint> > getContours(const vector<int>& contours_by_id, const vector<vector<ContourPoint> >& contours);
	vector<QrMarker> getMarkers(const vector<int>& contours_by_id, const vector<vector<ContourPoint> >& contours, const vector<Point2D>& mass_centers);
	void sortMarkers(vector<QrMarker>& markers);
	int individualizeTopLeftMarkerbyDiagonal(vector<QrMarker>& markers);
	int individualizeTopLeftMarkerbyAngle(vector<QrMarker>& markers);
	void sortMarkersVertices(vector<QrMarker>& markers);
	vector<Point2D> findSquare(const vector<QrMarker>& markers);
	// std::string translate(const cv::Mat &gray_input_image,const vector<Point2D>& qr_vertices);
	bool translate(const cv::Mat &gray_input_image,const vector<Point2D>& qr_vertices, std::string& message);

	template<typename Ta, typename Tb>
	double calculateDistance(const Ta& point1, const Tb& point2);
	template<typename Ta, typename Tb>
	Ta findMiddlePoint(const Ta& pointA, const Tb& pointB);
	template<typename T>
	double calculateSlope(const T& pointA, const T& pointB);
	template<typename T>
	double calculateSlope(const vector<T>& line);
	template<typename T>
	double calculateAngle(const T& pointA, const T& pointB, const T& pointC);
	template<typename T>
	T findIntersection(const vector<T>& lineA, const vector<T>& lineB);
	template<typename T, typename Tc>
	void calculateCoefficients(const T& pointA, const T& pointB, Tc &a, Tc &b, Tc &c);
	template<typename T, typename Tc>
	void calculateCoefficients(const vector<T>& line, Tc &a, Tc &b, Tc &c);
	template<typename T>
	T findProjection(const T& point, const T& point_of_lineA, const T& point_of_lineB);


	// TEST ////////////

	cv::Scalar randomColor();
	void drawContours(cv::Mat &inputimage, const vector<vector<ContourPoint> >& contours );
	void drawMassCenters(cv::Mat &inputimage, const vector<Point2D>& mass_centers );
	void drawMarkerVertices(cv::Mat &inputimage, const vector<QrMarker>& markers );
	void drawVertices(cv::Mat &inputimage, const vector<Point2D>& vertices );
	void drawLines(cv::Mat &inputimage, const vector<QrMarker>& markers );
void drawLine(shared_ptr<cv::Mat> inputimage, const vector<Point2D>& line, cv::Scalar color );
	void drawSquare(cv::Mat &inputimage, const vector<Point2D>& vertices, const vector<ContourPoint>& top_marker);
	///////////////////
};

} // "namespace polymechanon_vision"
#endif // QR_SCANNER_H
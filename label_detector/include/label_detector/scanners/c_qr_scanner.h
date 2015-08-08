#ifndef QR_SCANNER_H
#define QR_SCANNER_H

#include <iostream>
#include <string>
#include <memory>
#include <algorithm>

#include <opencv2/opencv.hpp>

// Other class dependecies
#include "label_detector/c_scanner.h"
#include "label_detector/c_label.h"

#include "ros/ros.h"

using std::vector;
using std::pair; //?
using std::shared_ptr;
using std::all_of;
using std::for_each;
using std::distance;
using std::rotate;
using std::max_element;
////////////////
using std::cout;
using std::endl;
//////////

typedef cv::Point ContourPoint;
typedef cv::Point2f Point2D;

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
	QrScanner();
	virtual ~QrScanner();

	LabelType getType() const;
	/* void setImageToScan(const shared_ptr<cv::Mat> input_image); */
	bool scan();


private:
	static const LabelType _type = LabelType::QRCODE;
	/* shared_ptr<cv::Mat> _image_to_scan */

	// Scanning functions
	vector<vector<ContourPoint> > findAlignmentMarkers(const cv::Mat& image);
	vector<Point2D> findContoursMassCenters(const vector<vector<ContourPoint> >& contours_vector);
	vector<vector<int> > clusterMarkers(const vector<Point2D>& contours_mass_centers);
	vector<Point2D> getMassCenters(const vector<int>& contours_by_id, const vector<Point2D>& mass_centers);
	vector<vector<ContourPoint> > getContours(const vector<int>& contours_by_id, const vector<vector<ContourPoint> >& contours);
	vector<QrMarker> getMarkers(const vector<int>& contours_by_id, const vector<vector<ContourPoint> >& contours, const vector<Point2D>& mass_centers);
	void sortMarkers(vector<QrMarker>& markers);
	vector<Point2D> findVertices(vector<QrMarker>& markers);

	template<typename Ta, typename Tb>
	double calculateDistance(const Ta& point1, const Tb& point2);
	template<typename T>
	double calculateSlope(const T& pointA, const T& pointB);
	template<typename T>
	T findProjection(const T& point, const T& point_of_lineA, const T& point_of_lineB);
	template<typename Ta, typename Tb>
	Ta findMid(const Ta& pointA, const Tb& pointB);
	// template<typename T>
	// T findIntersection(const T& point1A, const T& point1B, const T& point2A, const T& point2B);





	// TEST ////////////
	cv::Scalar randomColor();
	void drawContours(cv::Mat &inputimage, const vector<vector<ContourPoint> >& contours );
	void drawMassCenters(cv::Mat &inputimage, const vector<Point2D>& mass_centers );
	// void drawVertices(cv::Mat &inputimage, const vector<vector<ContourPoint> >& contours );
	///////////////////
};

} // "namespace polymechanon_vision"
#endif // QR_SCANNER_H
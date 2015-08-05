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
using std::pair;
using std::set;
using std::shared_ptr;
// using std::for_each;

typedef cv::Point ContourPoints;
typedef cv::Point2f Point2D;

namespace polymechanon_vision {

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
	vector<vector<ContourPoints> > findAlignmentMarkers(const cv::Mat& image);
	vector<Point2D> findContoursMassCenters(const vector<vector<ContourPoints> >& contours_vector);
	vector<vector<int> > clusterMarkers(const vector<Point2D>& contours_mass_centers);
	vector<vector<vector<ContourPoints> > > getContours(const vector<vector<int> >& contours_by_id, const vector<vector<ContourPoints> >& contours);

	template<typename T>
	double calculateDistance(const T& point1, const T& point2);







	// TEST ////////////
	void drawContours(cv::Mat &inputimage, const vector<vector<ContourPoints> >& contours );
	void drawMassCenters(cv::Mat &inputimage, const vector<Point2D>& mass_centers );
	///////////////////
};

} // "namespace polymechanon_vision"
#endif // QR_SCANNER_H
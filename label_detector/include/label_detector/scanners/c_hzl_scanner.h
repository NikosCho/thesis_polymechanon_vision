#ifndef HZL_SCANNER_H
#define HZL_SCANNER_H

#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp>

#include <label_detector/config_PP.h>  //PACKAGE_PATH

// Other class dependecies
#include "label_detector/c_scanner.h"
#include "label_detector/c_label.h"

using std::shared_ptr;
using std::vector;

using std::cout;
using std::endl;

typedef cv::Point ContourPoint;
// typedef cv::Point2f Point2D;

namespace polymechanon_vision {

struct HzLabelTemplate {
	std::string name;
	cv::Mat image;
	cv::Mat template_image;
	cv::Mat gray_template_image;
	cv::Mat thumbnail;
	HzLabelTemplate() {}
	HzLabelTemplate(const std::string& name , const cv::Mat& image, const cv::Mat& thumbnail): name(name), image(image), thumbnail(thumbnail){}
};

struct HzlLabel {
	vector<Point2D> contour;
	cv::Mat image;
	cv::Mat gray_image;
	bool matched;
	int match;
	double confidence;
};

class HzlScanner : public Scanner
{	

public:
	HzlScanner(shared_ptr<cv::Mat> input_image = nullptr);
	virtual ~HzlScanner();

	LabelType getType() const;
	/* void setImageToScan(const shared_ptr<cv::Mat> input_image); */
	bool scan();
	vector<vector<Point2D> > getDetectedLabels();

	bool setParameters(int par1, int par2, int side_length, int match_mehod, int contour_area_thres);

	///////////////////// Debugging Functions /////////////////////
	bool drawDetectedLabels(shared_ptr<cv::Mat> inputimage);
	bool drawDetectedLabels(cv::Mat &inputimage);
	cv::Mat getImageOfDetectedLabels(const cv::Mat &inputimage);



private:
	/* shared_ptr<cv::Mat> _image_to_scan */ 
	static const LabelType _type = LabelType::HZL;
	vector<vector<Point2D> > _detected_labels;



	// Parameters
	int _canny_param1;
	int _canny_param2;
	int _label_side_length;
	int _match_method;
	int _contour_area_thres;

	vector<HzLabelTemplate> _labels_templates;
	vector<Point2D> _perspective_square;

	cv::Mat _testing_image;


	bool loadLabels();
	vector< vector<Point2D> > findLabelContours(const cv::Mat& canny_image);
	vector<HzlLabel> getHazardousLabels(shared_ptr<cv::Mat> image, const vector<vector<Point2D> >& contours);
	// bool matchHazardousLabel(const cv::Mat& image);
	// bool getHueHistgram();

	// void rotate_image_90n(const cv::Mat& src, const cv::Mat& dst, int angle);
	bool matchALabel(HzlLabel& label);
	double templateMatching(const cv::Mat &image_to_match, const cv::Mat &template_image, int whatmethod);


	template<typename T>
	vector<Point2D> convertToPoint2D(const vector<T> input_vector);
	template<typename T>
	vector<ContourPoint> convertToContourPoint(const vector<T> input_vector);

	void calculateHistogram(const cv::Mat& image);
	cv::Scalar getBGRfromHUE(const int& value);


	void drawContours(cv::Mat &inputimage, const vector<vector<ContourPoint> >& contours );
	void drawMatches(cv::Mat &inputimage, vector<HzlLabel> &labels);


};

} // "namespace polymechanon_vision"
#endif // HZL_SCANNER_H
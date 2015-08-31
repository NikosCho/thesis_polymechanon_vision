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
	cv::Mat hue_histogram;
	cv::Mat thumbnail;
	HzLabelTemplate() {}
	HzLabelTemplate(const std::string& name , const cv::Mat& image, const cv::Mat& thumbnail): name(name), image(image), thumbnail(thumbnail){}
};

struct HzlLabel {
	vector<Point2D> contour;
	cv::Mat image;
	cv::Mat gray_image;
	cv::Mat hue_histogram;
	int threshold_value;
	bool matched;
	int match;
	double confidence;
};
typedef cv::Point2f Point2D;


class HzlScanner : public Scanner
{	

public:
	HzlScanner(shared_ptr<cv::Mat> input_image = nullptr);
	virtual ~HzlScanner();

	LabelType getType() const;
	/* void setImageToScan(const shared_ptr<cv::Mat> input_image); */
	bool scan();
	vector<vector<Point2D> > getDetectedLabels();

	bool setParameters(int par1, int par2, int side_length, int match_mehod, int template_match_mehod, int contour_area_thres, bool enable_color_match);

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
	int _template_match_method;
	int _contour_area_thres;
	bool _color_match_enabled;

	vector<HzLabelTemplate> _labels_templates;
	vector<Point2D> _perspective_square;

	bool loadLabels();
	vector< vector<Point2D> > findLabelContours(const cv::Mat& canny_image);
	vector<HzlLabel> getHazardousLabels(shared_ptr<cv::Mat> image, const cv::Mat& gray_image, const vector<vector<Point2D> >& contours);
	bool matchLabel(HzlLabel& label);
	
	double templateMatching(HzlLabel& label, HzLabelTemplate& template_label, int whatmethod);
	double customMatching(HzlLabel& label, HzLabelTemplate& template_label);
	
	cv::Mat getHueHistogram(const cv::Mat& image);
	void showHueHistogram(const cv::Mat& image, const std::string& window_name);
	cv::Scalar getBGRfromHUE(const int& hue_value);

	template<typename T>
	vector<Point2D> convertToPoint2D(const vector<T> input_vector);
	template<typename T>
	vector<ContourPoint> convertToContourPoint(const vector<T> input_vector);
	template<typename T>
	void normalizeVector(vector<T>& vector);
	template<typename Ta, typename Tb>
	Ta findMiddlePoint(const Ta& pointA, const Tb& pointB);

	void drawMatches(cv::Mat &inputimage, vector<HzlLabel> &labels);
	void drawMatch(cv::Mat &inputimage, const HzlLabel& label);


};

} // "namespace polymechanon_vision"
#endif // HZL_SCANNER_H
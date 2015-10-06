#ifndef SCANNER_H
#define SCANNER_H

#include <iostream>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>

// Other class dependecies
#include "label_detector/c_scanner.h"
#include "label_detector/c_label.h"

#include "ros/ros.h"

using std::shared_ptr;
using std::vector;

typedef cv::Point2f Point2D;


namespace polymechanon_vision {

class Scanner
{

public:
	Scanner(shared_ptr<cv::Mat> input_image = nullptr);
	virtual ~Scanner();

	virtual LabelType getType() const;
	void setImageToScan(const shared_ptr<cv::Mat> input_image);
	virtual bool scan();
	// virtual vector<vector<Point2D> > getDetectedLabels();	
	virtual std::vector<polymechanon_vision::Label> getDetectedLabels();	

	virtual bool setParameters(bool debugging, int par1, int par2);
	virtual bool setParameters(bool debugging, int par1, int par2, int side_length, int match_mehod, int template_match_mehod, int contour_area_thres, bool enable_color_match);


	///////////////////// Debugging Functions /////////////////////
	virtual bool drawDetectedLabels(shared_ptr<cv::Mat> inputimage);
	virtual bool drawDetectedLabels(cv::Mat &inputimage);
	virtual cv::Mat getImageOfDetectedLabels(const cv::Mat &inputimage);
	
protected:
	shared_ptr<cv::Mat> _image_to_scan;
	
private:
	LabelType _type;
	// vector<vector<Point2D> > _detected_labels;
	// vector<polymechanon_vision::Label> _detected_labels;

};

} // "namespace polymechanon_vision"
#endif // SCANNER_H
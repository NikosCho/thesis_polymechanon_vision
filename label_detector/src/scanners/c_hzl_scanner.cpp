#include "label_detector/scanners/c_hzl_scanner.h"

namespace polymechanon_vision {

HzlScanner::HzlScanner(shared_ptr<cv::Mat> input_image /* = nullptr */)
{	
	// if ( input_image != nullptr)    setImageToScan(input_image);
	if ( input_image != nullptr)    Scanner::setImageToScan(input_image);
	ROS_WARN("Hzl-Scanner created!");

	_canny_param1 = 100;
	_canny_param2 = 200;
	_label_side_length = 150;
	_match_method = 0;
	_template_match_method = 0;
	_contour_area_thres = 10000;
	_color_match_enabled = true;

	_perspective_square = {
		Point2D(_label_side_length/2, 0),
		Point2D(_label_side_length, _label_side_length/2),
		Point2D(_label_side_length/2, _label_side_length),
		Point2D(0, _label_side_length/2)
	};

	loadLabels();

	ROS_WARN("HZL-Scanner created!");
}

HzlScanner::~HzlScanner() { 
	ROS_ERROR("HZL-Scanner deleted!"); 
}

/// Get the type of scanner's detection
/**
*	Virtual method of base class 'Scanner' 
*/
LabelType HzlScanner::getType() const
{
	return _type;
}

/// 'Main' function of scanner.
/**
*	Implementation of whole scanner process.
*	
*	\returns true/false accοording to the progress of scanning (true - if anything was detected  false - if not).
*	\exception std::runtime_error - if camera input's pointer is 'nullptr'.
*/
bool HzlScanner::scan()
{	
	if (_image_to_scan == nullptr ) throw std::runtime_error("[HzlScanner]-scan() : image's pointer is nullptr (use 'setImageToScan()'). ");

	// Check if any labels are loaded
	static bool error_handler = true;
	if ( _labels_templates.size() == 0 )
	{	
		if ( error_handler ) {
			ROS_ERROR("[HzlScanner]-scan() : no labels' templates are loaded.");
			error_handler = false;
		}
		return false;
	} else if ( !error_handler ) 
		error_handler = true;

	cv::Mat caminput_gray(_image_to_scan->size(), CV_MAKETYPE(_image_to_scan->depth(), 1));    // To hold Grayscale Image
	cv::Mat caminput_edges(caminput_gray.size(), CV_MAKETYPE(caminput_gray.depth(), 1));    // To hold Grayscale Image

	cv::cvtColor(*_image_to_scan, caminput_gray, CV_RGB2GRAY);    // Convert Image captured from Image Input to GrayScale 
	cv::Canny(caminput_gray, caminput_edges, _canny_param1 , _canny_param2, 3);    // Apply Canny edge detection on the img_gray image
	vector<vector<Point2D> > label_contours = findLabelContours(caminput_edges);

	// In case we didn't find any frame of a possible label.
	if ( label_contours.size() == 0 )    return false;

	vector<HzlLabel> labels = getHazardousLabels(_image_to_scan, caminput_gray, label_contours);

	for_each(begin(labels), end(labels), [&](HzlLabel& label){
		matchALabel(label);
	});






	return true;
}



vector<vector<Point2D> > HzlScanner::getDetectedLabels()
{	
	vector<vector<Point2D> > detected_labels;
	return detected_labels;
}

bool HzlScanner::setParameters(int par1, int par2, int side_length, int match_mehod, int template_match_mehod, int contour_area_thres, bool enable_color_match)
{	
	_canny_param1 = par1;
	_canny_param2 = par2;
	_label_side_length = side_length;
	_match_method = match_mehod;
	_template_match_method = template_match_mehod;
	_contour_area_thres = contour_area_thres;
	_color_match_enabled = enable_color_match;
	return true;
}

/// Load hazardous labels.
/**
 *	Read the hazardous' labels names from file and 
 *	create an template for each label.
 */
bool HzlScanner::loadLabels()
{   
	std::stringstream path;
	path << PACKAGE_PATH 
		 << "/config/hazardous_labels_images/";

	std::string labels_path = path.str();

	std::string labels_file = path.str();
	labels_file.append("labels_names.xml");

	vector< std::string > labels_names;

	// Read file and get labels' names
	cv::FileStorage fs;
	try	{
		if ( fs.open(labels_file, cv::FileStorage::READ) ) {
			cv::FileNode names_node = fs["hazardous_labels"];
			cv::FileNodeIterator it = names_node.begin(), it_end = names_node.end();

			for( ; it != it_end; ++it ) 
				labels_names.push_back((*it));
		} else {
			ROS_ERROR("[HzlScanner]-loadLabels() : Missing xml file.");
		}
	} catch (const cv::Exception &e)	{
		ROS_ERROR("%s", e.what());
	}
	fs.release();

	// Create a template for each label
	_labels_templates.reserve(labels_names.size());

	for_each(begin(labels_names), end(labels_names),[&](const std::string& name){
		HzLabelTemplate new_label_template;
		// Set label's name
		new_label_template.name = name;
		// Set label's image
		std::string file_path(labels_path);
		file_path.append(name);
		file_path.append(".jpg");
		try {
			new_label_template.image = cv::imread(file_path);
		} catch (const cv::Exception &e)	{
			ROS_ERROR("[HzlScanner]-loadLabels() : missing %s file. (Update xml file or add the file)", name.c_str());	
		} 
		// Set label's template
		new_label_template.template_image = new_label_template.image;
		cv::resize(new_label_template.template_image, new_label_template.template_image, cv::Size( _label_side_length, _label_side_length));
		cv::flip(new_label_template.template_image, new_label_template.template_image, 1);
		// Set label's gray template
		cv::cvtColor(new_label_template.template_image, new_label_template.gray_template_image, CV_BGR2GRAY);    // Convert Image captured from Image Input to GrayScale 
		new_label_template.template_image = new_label_template.image;
		// Set label's hue's histgram
		new_label_template.hue_histogram = getHueHistogram(new_label_template.template_image);
		// Set label's thumbnail
		new_label_template.thumbnail = new_label_template.image;
		cv::resize(new_label_template.thumbnail, new_label_template.thumbnail, cv::Size( 50, 50));

		_labels_templates.push_back(new_label_template);
	});

	ROS_WARN("[HzlScanner]-loadLabels() : labels loaded = %zu", _labels_templates.size() );
    return true;
}

/// Finds contours that meet the conditions of an alignment marker.
vector<vector<Point2D> > HzlScanner::findLabelContours(const cv::Mat& canny_image)
{
	vector<vector<ContourPoint> > contours;
	vector<vector<ContourPoint> > approximated_contours;
	vector<bool> contours_convexity;
	vector<cv::Vec4i> hierarchy;   
	cv::findContours( canny_image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);    // Find contours with full hierarchy

	for_each(begin(contours), end(contours), [&](const vector<ContourPoint>& contour) {
		vector<ContourPoint> approximated_contour;
		cv::approxPolyDP(contour, approximated_contour, cv::arcLength(contour, true)*0.05, true);  
		approximated_contours.push_back(approximated_contour);
		contours_convexity.push_back( isContourConvex(approximated_contour) );
	});

	vector<vector<Point2D> > label_frames;    // To hold contours that meet the conditions of an alignment marker
	vector<Point2D> temp_vector;    

	if ( contours.size() == approximated_contours.size() ) {   // Just in case..
		for ( size_t k = 0; k < approximated_contours.size(); k++) {
			
			bool is_possible_label = approximated_contours[k].size() == 4 &&    // It is quadrilateral
									 contours_convexity[k] &&    // It is convex
									 hierarchy[k][2] != -1 &&    // It is enclosing another contour
									 approximated_contours[ hierarchy[k][2] ].size() == 4 &&    // The enclosing contour is quadrilateral
									 contours_convexity[hierarchy[k][2]] &&    // The enclosing contour is convex
									 contourArea(approximated_contours[hierarchy[k][2]]) > (contourArea(approximated_contours[k]) * 0.7) &&
									 contourArea(approximated_contours[hierarchy[k][2]]) > _contour_area_thres;    // Threshold of aeras

			if ( is_possible_label ) 
				label_frames.push_back( convertToPoint2D(approximated_contours[k]) );    // Contours that meet the conditions of a hazardous label frame are saved seperate
		}
	}
	return label_frames;
}

/// Finds contours that meet the conditions of an alignment marker. (in addition calculates a mean/threshold value)
vector<HzlLabel> HzlScanner::getHazardousLabels(shared_ptr<cv::Mat> image, const cv::Mat& gray_image, const vector<vector<Point2D> >& contours)
{
	vector<HzlLabel> labels_to_return;

	for_each(begin(contours), end(contours), [&](const vector<Point2D>& contour){
		cv::Mat cropped;
		cropped = cv::Mat((*image).rows, (*image).cols, (*image).type());
		cropped.setTo(cv::Scalar(255,255,255));

		vector< vector< Point2D > > contour_vec = { contour };
		vector< vector< ContourPoint > > cntrpt_vec = { convertToContourPoint(contour) };

		cv::Mat mask_to_crop = cv::Mat::zeros((*image).rows, (*image).cols, CV_8UC1);
		cv::drawContours(mask_to_crop, cntrpt_vec, -1, cv::Scalar(255), CV_FILLED);

		HzlLabel new_label;

		if (_match_method == 1) {    // Selected method: customMatching
	    	cv::Scalar mean;
			mean = cv::mean(gray_image, mask_to_crop);
			new_label.threshold_value = mean[0];				
		}

	    cv::Mat warp_matrix = getPerspectiveTransform(contour_vec[0], _perspective_square);

	    cv::Mat label_raw = cv::Mat::zeros(_label_side_length, _label_side_length, CV_8UC3 );
		label_raw.setTo(cv::Scalar(255,255,255));

		(*image).copyTo(cropped, mask_to_crop);
	    warpPerspective(cropped, label_raw, warp_matrix, cv::Size(label_raw.cols, label_raw.rows));

		new_label.contour = contour;	    
		new_label.image = label_raw;	
		cv::Mat gray_label_raw;
		cv::cvtColor(label_raw, gray_label_raw, CV_BGR2GRAY);    // Convert Image captured from Image Input to GrayScale 
		new_label.gray_image = gray_label_raw;	    

		labels_to_return.push_back(new_label);
	});

	return labels_to_return;
}


bool HzlScanner::matchALabel(HzlLabel& label)
{	
	if ( _labels_templates.size() == 0 )    throw std::runtime_error( "[HzlScanner]-matchALabel() : no labels' templates are loaded" );	

    vector<double> matching_values;
    for( int i = 0; i < _labels_templates.size(); i++ )  {
    	if (_match_method == 0) 
        	matching_values.push_back( templateMatching(label.gray_image, _labels_templates[i].gray_template_image, _template_match_method) );
    	else if (_match_method == 1) 
        	matching_values.push_back( customMatching(label.gray_image, _labels_templates[i].gray_template_image) );
    }

    int match_id;
    double match_value;
	if (_match_method == 0) {
		auto max_it = std::max_element(begin(matching_values), end(matching_values));
		match_id = std::distance( begin(matching_values), max_it )
		match_value = *max_it;

		if ( match_value > .0) {    // NOT_IMPLEMENTED
			label.matched = true;
    		label.match = match_id;
    		label.confidence = match_value;    // NOT_IMPLEMENTED
		} else    label.matched = false;

	} else if (_match_method == 1) {
		auto min_it = std::min_element(begin(matching_values), end(matching_values));
		match_id = std::distance( begin(matching_values), min_it )
		match_value = *min_it;

		if ( match_value > 0 ) {    // NOT_IMPLEMENTED
			label.matched = true;
    		label.match = match_id;
    		label.confidence = match_value;    // NOT_IMPLEMENTED
		} else    label.matched = false;		
	}



	label.matched = false;
    return false;
}

double templateMatching(HzlLabel& label, HzLabelTemplate& template_label, int whatmethod)
{
	double minVal; 
    double maxVal; 

    cv::Mat temp_image;
    copyMakeBorder( label.gray_image, temp_image, 40, 40, 40, 40,cv::BORDER_CONSTANT, cv::Scalar(255,255,255) );

    // cv::Mat result= temp_image.clone();
    /// Create the result matrix
    int result_cols =  temp_image.cols - template_label.gray_template_image.cols + 1;
    int result_rows = temp_image.rows - template_label.gray_template_image.rows + 1;

    result.create( result_rows, result_cols, CV_32FC1 );

    matchTemplate( temp_image, template_label.gray_template_image, result, whatmethod );
    normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

    cv::Scalar mean = cv::mean(result);
    double value_to_return = mean[0];
    if ( whatmethod  == cv::TM_SQDIFF || whatmethod == cv::TM_SQDIFF_NORMED )    value_to_return =  1 - value_to_return; 

    return value_to_return;

}

double HzlScanner::customMatching(const cv::Mat &input_image,const cv::Mat &template_image)
{
    cv::Mat dif;
	cv::Mat image_thres = cv::Mat::zeros(100, 100, CV_8UC1);
    cv::Scalar image_mean, image_stddev;
    cv::meanStdDev(input_image, image_mean, image_stddev);
	threshold(input_image, image_thres, (int)image_mean[0], 255, CV_THRESH_BINARY);
	// cv::adaptiveThreshold(input_image, image_thres, 255, cv::ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 3, 1);
	cv::Mat template_thres = cv::Mat::zeros(100, 100, CV_8UC1);
	threshold(template_image, template_thres, 0, 255, CV_THRESH_OTSU);

	// imshow("input_image", image_thres);
	// imshow("template_image", template_thres);


    cv::absdiff(template_thres,image_thres,dif);

    // cv::absdiff(input_image,template_image,dif);
    // cv::threshold(dif, dif, 35, 255,cv::THRESH_BINARY);
    cv::Scalar mean, stddev;
    cv::meanStdDev(dif, mean, stddev);


	// imshow("KAKAKA", lb_thres);

    // cout << " mean[0] = " << mean[0] << endl;
    // cout << " stddev[0] = " << stddev[0] << endl;

    return mean[0];
}

/// Calculate and return the image's hue's histogram.
cv::Mat HzlScanner::getHueHistogram(const cv::Mat& image)
{	
	cv::Mat hsv_image;
	cv::cvtColor(image, hsv_image, CV_BGR2HSV);

	vector<cv::Mat> hsv_planes;
	cv::split(hsv_image, hsv_planes);

	// Establish the number of bins
	int histSize = 181;
	// Set the range ( channel's size= 8bit, so H's range is 0-180 )
	float range[] = { 0, 181 } ;
	const float* histRange = { range };

	bool uniform = true; bool accumulate = false;

	cv::Mat hue_hist;
	calcHist( &hsv_planes[0], 1, 0, cv::Mat(), hue_hist, 1, &histSize, &histRange, uniform, accumulate );

	return hue_hist;
}

/// Draw an image's histogram.
void HzlScanner::showHueHistogram(const cv::Mat& image, const std::string& window_name)
{
	cv::Mat hsv_image;
	cv::cvtColor(image, hsv_image, CV_BGR2HSV);

	vector<cv::Mat> hsv_planes;
	cv::split(hsv_image, hsv_planes);

	// Establish the number of bins
	int histSize = 181;
	// Set the ranges ( for B,G,R) )
	float range[] = { 0, 181 } ;
	const float* histRange = { range };

	bool uniform = true; bool accumulate = false;

	cv::Mat hue_hist;
	calcHist( &hsv_planes[0], 1, 0, cv::Mat(), hue_hist, 1, &histSize, &histRange, uniform, accumulate );

	// Draw the histograms for B, G and R
	int hist_w = 512; int hist_h = 400;
	int bin_w = cvRound( (double) hist_w/histSize );

	cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
	cv::normalize(hue_hist, hue_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

	for( int i = 1; i < histSize; i++ )
	{
		cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(hue_hist.at<float>(i-1)) ) ,
							 cv::Point( bin_w*(i), hist_h - cvRound(hue_hist.at<float>(i)) ),
							 getBGRfromHUE(i), 2, 8, 0  );
	}	

	cv::imshow(window_name, histImage );
}

/// Convert hue's value to an rgb color.
cv::Scalar HzlScanner::getBGRfromHUE(const int& hue_value)
{	
	// // MANUALLY ////////////////////////////////////
	// cv::Scalar color_to_return;
	// if ( 0 <= hue_value && hue_value < 12)    		color_to_return = { 0, 0, 255};	// RED
	// else if ( 12 <= hue_value && hue_value < 25)	color_to_return = { 0, 128, 255};	// ORANGE
	// else if ( 25 <= hue_value && hue_value < 35)	color_to_return = { 0, 255, 255};	// YELLOW
	// else if ( 35 <= hue_value && hue_value < 80)	color_to_return = { 0, 255, 0};	// GREEN
	// else if ( 80 <= hue_value && hue_value < 100)	color_to_return = { 255, 255, 0};	// SIEL
	// else if ( 100 <= hue_value && hue_value < 140)	color_to_return = { 255, 0, 0};	// BLUE
	// else if ( 140 <= hue_value && hue_value < 165)	color_to_return = { 255, 0, 255};	// PINK
	// else if ( 165 <= hue_value && hue_value < 180)	color_to_return = { 0, 0, 255};	// RED
	// else color_to_return = { 0, 0, 255};	// Undefined: WHITE

	// Using OPENCV /////////////////////////////////////////////////
	cv::Mat input_color(1,1,CV_8UC3);
	input_color.setTo( cv::Scalar(hue_value,255,255) );
	cv::Mat output_color(1,1,CV_8UC3);
	cv::cvtColor(input_color,output_color, CV_HSV2BGR);
	cv::Vec3b& color = output_color.at<cv::Vec3b>(0,0);
	cv::Scalar color_to_return(color[0],color[1],color[2]);

	return color_to_return;
}












///////////////////////// Debugging Functions /////////////////////////
bool HzlScanner::drawDetectedLabels(shared_ptr<cv::Mat> inputimage)
{	
	return false;
}

bool HzlScanner::drawDetectedLabels(cv::Mat &inputimage)
{
	return false;
}

cv::Mat HzlScanner::getImageOfDetectedLabels(const cv::Mat &inputimage)
{	
	cv::Mat image_to_draw = inputimage.clone();
	return image_to_draw;
}

} // "namespace polymechanon_vision"
#include "label_detector/scanners/c_hzl_scanner.h"

//
/*

contourArea

Calculates a contour area.

C++: double contourArea(InputArray contour, bool oriented=false )

Python: cv2.contourArea(contour[, oriented]) → retval

C: double cvContourArea(const CvArr* contour, CvSlice slice=CV_WHOLE_SEQ, int oriented=0 )

Python: cv.ContourArea(contour, slice=CV_WHOLE_SEQ) → float
    Parameters:	

        contour – Input vector of 2D points (contour vertices), stored in std::vector or Mat.
        oriented – Oriented area flag. If it is true, the function returns a signed area value, depending on the contour orientation (clockwise or counter-clockwise). Using this feature you can determine orientation of a contour by taking the sign of an area. By default, the parameter is false, which means that the absolute value is returned.


*/
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
	_template_match_method = 4;
	_contour_area_thres = 10000;
	_color_match_enabled = true;

	_perspective_square = {
		Point2D(_label_side_length/2, 0),
		Point2D(_label_side_length, _label_side_length/2),
		Point2D(_label_side_length/2, _label_side_length),
		Point2D(0, _label_side_length/2)
	};

	loadLabels();

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

	cv::Mat testing_image = (*_image_to_scan).clone();


	cv::Mat caminput_gray(_image_to_scan->size(), CV_MAKETYPE(_image_to_scan->depth(), 1));    // To hold Grayscale Image
	cv::Mat caminput_edges(caminput_gray.size(), CV_MAKETYPE(caminput_gray.depth(), 1));    // To hold Grayscale Image

	cv::cvtColor(*_image_to_scan, caminput_gray, CV_RGB2GRAY);    // Convert Image captured from Image Input to GrayScale 
	cv::Canny(caminput_gray, caminput_edges, _canny_param1 , _canny_param2, 3);    // Apply Canny edge detection on the img_gray image
	vector<vector<Point2D> > label_contours = findLabelContours(caminput_edges);

	// In case we didn't find any frame of a possible label.
	if ( label_contours.size() == 0 )    return false;

	vector<HzlLabel> labels = getHazardousLabels(_image_to_scan, caminput_gray, label_contours);

	// for (size_t kk = 0; kk <labels.size(); kk++ ) {

	// 	ROS_INFO(" MATCHING A LABEL");
	// 	matchLabel(labels[kk]);

	// 	if( labels[kk].matched ) {
	// 		drawMatch(testing_image, labels[kk]);
	// 		cout << "MAtch " << _labels_templates[labels[kk].match].name << " - " << labels[kk].match<< endl;
	// 	} 
	// 	else 
	// 	ROS_INFO(" ---- NO MATCH");
	// }

	_detected_labels.clear();
	_labels.clear();

	cv::Mat debugging_image(360, 512, CV_8UC3, cv::Scalar( 0,0,0) );
	cv::Mat TEST_TEST;



	for_each(begin(labels), end(labels), [&](HzlLabel& label){
		matchLabel(label);

		// ROS_INFO(" MATCHING A LABEL");
		if( label.matched ) {
			drawMatch(testing_image, label);
			// cout << "MAtch " << _labels_templates[label.match].name << " - " << label.match<< endl;

			if ( _color_match_enabled ) {
				drawHueHistogram(debugging_image, _labels_templates[0].hue_histogram, cv::Scalar(80,80,80), 1);
				drawHueHistogram(debugging_image, _labels_templates[1].hue_histogram, cv::Scalar(80,80,80), 1);
				drawHueHistogram(debugging_image, _labels_templates[2].hue_histogram, cv::Scalar(80,80,80), 1);
				drawHueHistogram(debugging_image, _labels_templates[3].hue_histogram, cv::Scalar(80,80,80), 1);
				drawHueHistogram(debugging_image, _labels_templates[4].hue_histogram, cv::Scalar(80,80,80), 1);

				drawHueHistogram(debugging_image, _labels_templates[label.match].hue_histogram, cv::Scalar(255,255,255), 2);
			}
			drawHueHistogram(debugging_image, label.hue_histogram);

			drawDebugging(TEST_TEST, debugging_image, label, _labels_templates[label.match] );


			_detected_labels.push_back(label.contour);
			_labels.push_back(label);
		} 
		// else 
		// ROS_INFO(" ---- NO MATCH");


	});
	
	static int labels_detected_num;
	if ( _debugging )
	{	
		// std::stringstream debugging_window_name;

		for( size_t i = 0; i < _labels.size(); i++) {
			std::stringstream debugging_window_name;
			debugging_window_name << "[Hzl-Scanner] - Debugging label - " << i;

			cv::Mat histogram_image(360, 512, CV_8UC3, cv::Scalar( 0,0,0) );
			if ( _color_match_enabled ) {
				drawHueHistogram(histogram_image, _labels_templates[0].hue_histogram, cv::Scalar(80,80,80), 1);
				drawHueHistogram(histogram_image, _labels_templates[1].hue_histogram, cv::Scalar(80,80,80), 1);
				drawHueHistogram(histogram_image, _labels_templates[2].hue_histogram, cv::Scalar(80,80,80), 1);
				drawHueHistogram(histogram_image, _labels_templates[3].hue_histogram, cv::Scalar(80,80,80), 1);
				drawHueHistogram(histogram_image, _labels_templates[4].hue_histogram, cv::Scalar(80,80,80), 1);
				drawHueHistogram(histogram_image, _labels_templates[_labels[i].match].hue_histogram, cv::Scalar(255,255,255), 2);
			}
			drawHueHistogram(histogram_image, _labels[i].hue_histogram);
			cv::Mat debugging_image;
			drawDebugging(debugging_image, histogram_image, _labels[i], _labels_templates[_labels[i].match] );

			cv::imshow(debugging_window_name.str(), debugging_image);
		}

		for( size_t i = labels_detected_num; i >= _labels.size(); i--) {
			std::stringstream debugging_window_name;
			debugging_window_name << "[Hzl-Scanner] - Debugging label - " << i;
			cv::destroyWindow(debugging_window_name.str());
		}

	} else {
		for( size_t i = labels_detected_num; i != -1; i--) {
			std::stringstream debugging_window_name;
			debugging_window_name << "[Hzl-Scanner] - Debugging label - " << i;
			cv::destroyWindow(debugging_window_name.str());
		}
	}

	labels_detected_num = _labels.size();


	cv::imshow("wxwxw", testing_image);
	// cv::imshow("debugging_image", debugging_image);
	// cv::imshow("TEST_TEST", TEST_TEST);


	return true;
}



vector<vector<Point2D> > HzlScanner::getDetectedLabels()
{	
	return _detected_labels;
}

bool HzlScanner::setParameters(bool debugging, int par1, int par2, int side_length, int match_mehod, int template_match_mehod, int contour_area_thres, bool enable_color_match)
{	
	_debugging = debugging;
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
		// new_label_template.template_image = new_label_template.image;
		// Set label's hue's histgram
		new_label_template.hue_histogram = getHueHistogram(new_label_template.template_image);
		// Set label's binary ( thresholded ) image
		cv::threshold(new_label_template.gray_template_image, new_label_template.binary_template_image, 0, 255, CV_THRESH_OTSU);
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
									 contourArea(approximated_contours[hierarchy[k][2]]) > ( contourArea(approximated_contours[k])*0.99) &&    // The inner contour area must be at least the 80% percent of outer's aera
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
		new_label.hue_histogram = getHueHistogram(label_raw);	

		cv::Mat gray_label_raw;
		cv::cvtColor(label_raw, gray_label_raw, CV_BGR2GRAY);    // Convert Image captured from Image Input to GrayScale 
		new_label.gray_image = gray_label_raw;	    

		labels_to_return.push_back(new_label);
	});

	return labels_to_return;
}


bool HzlScanner::matchLabel(HzlLabel& label)
{	
	if ( _labels_templates.size() == 0 )    throw std::runtime_error( "[HzlScanner]-matchALabel() : no labels' templates are loaded" );	

    vector<double> matching_values;
    vector<double> histogram_matching_values;
    for( int i = 0; i < _labels_templates.size(); i++ )  {
    	if (_match_method == 0) {    // templateMatching
        	matching_values.push_back( templateMatching(label, _labels_templates[i], _template_match_method) );

        	if ( _color_match_enabled ) {
        		// label.hue_histogram = getHueHistogram(label.image);
				histogram_matching_values.push_back( compareHist( 	label.hue_histogram, 
																	_labels_templates[i].hue_histogram,
																	CV_COMP_INTERSECT));    	
	    	}
	
    	} else if (_match_method == 1) {     // customMatching
        	matching_values.push_back( customMatching(label, _labels_templates[i]) );
        	
        	if ( _color_match_enabled ) {
        		// label.hue_histogram = getHueHistogram(label.image);
				histogram_matching_values.push_back( compareHist( 	label.hue_histogram, 
																	_labels_templates[i].hue_histogram,
																	CV_COMP_INTERSECT));    	
	    	}
    	}
    }


	// cout << "========== COMPARISON ============" << endl;
	// for(size_t m=0; m < histogram_matching_values.size(); m++) {
	// 	if ( _color_match_enabled ) 
	// 		cout << "Compare Histograms' result : " << histogram_matching_values[m] << endl;
	// 	cout << "\t template matching result : " << matching_values[m] << endl;

	// } 


    if ( _color_match_enabled ) {
	    normalizeVector(histogram_matching_values);
	    for(size_t c = 0; c < matching_values.size(); c++) {
			if (_match_method == 0) {
	    		matching_values[c] = matching_values[c] * histogram_matching_values[c];
			} else if (_match_method == 1) {
	    		matching_values[c] = matching_values[c] * ( 1 - histogram_matching_values[c]);
	    	}
		}
	}

	// for(size_t m=0; m < histogram_matching_values.size(); m++) {
	// 	if ( _color_match_enabled ) 
	// 		cout << " -- Normalized Compare Histograms' result : " << histogram_matching_values[m] << endl;
	// 	cout << "\t -- Normalized template matching result : " << matching_values[m] << endl;

	// } 

	// cout << "_match_method - " << _match_method << endl;
	// cout << "_template_match_method - " << _template_match_method << endl;
	// cout << "_color_match_enabled - " << _color_match_enabled << endl;
	// cout << "==============================" << endl;

    int match_id;
    double match_value;
	if (_match_method == 0) {
		auto max_it = std::max_element(begin(matching_values), end(matching_values));
		match_id = std::distance( begin(matching_values), max_it);
		match_value = *max_it;

		// if ( match_value > .0) {    // NOT_IMPLEMENTED
			ROS_WARN(" matched");
			label.matched = true;
    		label.match = match_id;
			ROS_ERROR("MATCH -- %s - %d ", _labels_templates[label.match].name.c_str() ,label.match);
    		label.confidence = match_value;    // NOT_IMPLEMENTED
		// } else    label.matched = false;

	} else if (_match_method == 1) {
		auto min_it = std::min_element(begin(matching_values), end(matching_values));
		match_id = std::distance( begin(matching_values), min_it);
		match_value = *min_it;

		// if ( match_value > 0 ) {    // NOT_IMPLEMENTED
			ROS_WARN(" matched");
			label.matched = true;
    		label.match = match_id;
			ROS_ERROR("MATCH -- %s - %d ", _labels_templates[label.match].name.c_str() ,label.match);
    		label.confidence = match_value;    // NOT_IMPLEMENTED
		// } else    label.matched = false;		
	} else label.matched = false;
  
    return false;
}

double HzlScanner::templateMatching(HzlLabel& label, HzLabelTemplate& template_label, int whatmethod)
{
	double minVal; 
    double maxVal; 

    cv::Mat temp_image;
    copyMakeBorder( label.gray_image, temp_image, 20, 20, 20, 20,cv::BORDER_CONSTANT, cv::Scalar(255,255,255) );

    cv::Mat result= temp_image.clone();
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

double HzlScanner::customMatching(HzlLabel& label, HzLabelTemplate& template_label)
{
    cv::Mat dif;
    cv::Mat label_thres;
    // cv::Mat label_thres, template_label_thres;

	cv::threshold(label.gray_image, label_thres, label.threshold_value, 255, CV_THRESH_BINARY);
	// cv::threshold(template_label.gray_template_image, template_label_thres, 0, 255, CV_THRESH_OTSU);

    // cv::absdiff(label_thres, template_label_thres, dif);
    cv::absdiff(label_thres, template_label.binary_template_image, dif);

	cv::Scalar mean = cv::mean(dif);  
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

	for( int i = 2; i < histSize; i++ )
	{
		cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(hue_hist.at<float>(i-1)) ) ,
							 cv::Point( bin_w*(i), hist_h - cvRound(hue_hist.at<float>(i)) ),
							 getBGRfromHUE(i), 2, 8, 0  );
	}	

	cv::imshow(window_name, histImage );
}

/// Draw an image's histogram.
void HzlScanner::drawHueHistogram(cv::Mat& image, const cv::Mat& histogram, cv::Scalar color /* = {0,0,0} */, int line_thickness /*  = 2 */)
{

	// Establish the number of bins
	int histSize = 181;

	// Draw the histograms for B, G and R
	// int hist_w = 512; int hist_h = 400;
	int hist_w = image.cols; 
	int hist_h = image.rows;
	int bin_w = cvRound( (double) hist_w/histSize );
	cv::Mat normalized_histogram;
	cv::normalize(histogram, normalized_histogram, 0, image.rows, cv::NORM_MINMAX, -1, cv::Mat() );


	for( int i = 2; i < histSize; i++ )
	{	
		if ( color == cv::Scalar(0,0,0) )
			cv::line( image, cv::Point( bin_w*(i-1), hist_h - cvRound(normalized_histogram.at<float>(i-1)) ) ,
								 cv::Point( bin_w*(i), hist_h - cvRound(normalized_histogram.at<float>(i)) ),
								 getBGRfromHUE(i), line_thickness, 8, 0  );
		else 
			cv::line( image, cv::Point( bin_w*(i-1), hist_h - cvRound(normalized_histogram.at<float>(i-1)) ) ,
								 cv::Point( bin_w*(i), hist_h - cvRound(normalized_histogram.at<float>(i)) ),
								 color, line_thickness, 8, 0  );
	}	

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



template<typename T>
vector<Point2D> HzlScanner::convertToPoint2D(const vector<T> input_vector)
{
	vector<Point2D> output_vector;

	for(size_t k = 0; k < input_vector.size(); k++)
	{
		output_vector.push_back(input_vector[k]);
	}

	return output_vector;
}

template<typename T>
vector<ContourPoint> HzlScanner::convertToContourPoint(const vector<T> input_vector)
{
	vector<ContourPoint> output_vector;

	for(size_t k = 0; k < input_vector.size(); k++)
	{
		output_vector.push_back(input_vector[k]);
	}

	return output_vector;
}

template<typename T>
void HzlScanner::normalizeVector(vector<T>& vector)
{
	auto min_max_it = minmax_element(begin(vector), end(vector));

	T min_value = *min_max_it.first;
	T max_value = *min_max_it.second;

	for_each(begin(vector), end(vector), [&max_value, &min_value](T& val){
		val = (val - min_value) / (max_value - min_value);
	});

}


/// Finds the middle point between two points.
template<typename Ta, typename Tb>
Ta HzlScanner::findMiddlePoint(const Ta& pointA, const Tb& pointB)
{
	return Ta((pointA.x+pointB.x)/2, (pointA.y+pointB.y)/2);
}












void HzlScanner::drawMatches(cv::Mat &inputimage, vector<HzlLabel> &labels)
{   

    for(auto label : labels )   {   // For each (of the 3) markers of our identifier
        // cout << "?" << endl;
        cv::Rect roi( label.contour[0], cv::Size( _labels_templates[label.match].thumbnail.cols, _labels_templates[label.match].thumbnail.rows) );
        _labels_templates[label.match].thumbnail.copyTo( inputimage( roi ) );
    }



    // cv::circle( inputimage, mycenter, 2,  cv::Scalar(255,255,0), 2, 8, 0 );
}


void HzlScanner::drawMatch(cv::Mat &inputimage, const HzlLabel& label)
{   
        cv::Rect roi( label.contour[0], cv::Size( _labels_templates[label.match].thumbnail.cols, _labels_templates[label.match].thumbnail.rows) );
        _labels_templates[label.match].thumbnail.copyTo( inputimage( roi ) );
}


bool HzlScanner::drawDebugging(cv::Mat& image_to_draw, cv::Mat& histogram_matching, const HzlLabel& label, const HzLabelTemplate& template_label )
{	
	int temp_int_1 = histogram_matching.rows;
	int temp_int_2 = 2*label.image.rows + 60;

	int rows = temp_int_1 >= temp_int_2? temp_int_1 : temp_int_2;

	// int rows = histogram_matching.rows;
	int cols = histogram_matching.cols + (2*label.image.cols) + 10;
	image_to_draw = cv::Mat(rows, cols,  CV_8UC3);
	image_to_draw.setTo(cv::Scalar(0,0,0));

	// Draw histogram
    cv::Rect roi( cv::Point(0,0), cv::Size(  histogram_matching.cols, histogram_matching.rows) );
    histogram_matching.copyTo( image_to_draw( roi ) );
	// Draw label's colored image
    roi = { cv::Point(histogram_matching.cols + 10, 30), cv::Size(label.image.cols, label.image.rows) };
    label.image.copyTo( image_to_draw( roi ) );
	// Draw template's colored image
    roi = { cv::Point(histogram_matching.cols + 10, label.image.rows + 60), cv::Size(template_label.template_image.cols, template_label.template_image.rows) };
    template_label.template_image.copyTo( image_to_draw( roi ) );

    // Draw label's and template's matching images
    if (_match_method == 0) {    // templateMatching
	    // Label
	    cv::Rect roi_1( cv::Point(histogram_matching.cols + label.image.cols + 10, 30), cv::Size(label.gray_image.cols, label.gray_image.rows) );
	    cv::Mat temp_image1;
		cvtColor(label.gray_image, temp_image1, CV_GRAY2BGR);
	    temp_image1.copyTo( image_to_draw( roi_1 ) );		
	    // Template
	    cv::Rect roi_2( cv::Point(histogram_matching.cols + label.image.cols + 10, label.image.rows + 60), cv::Size(template_label.gray_template_image.cols, template_label.gray_template_image.rows) );
	    cv::Mat temp_image2;
		cvtColor(template_label.gray_template_image, temp_image2, CV_GRAY2BGR);
	    temp_image2.copyTo( image_to_draw( roi_2 ) );
	} else {    // customMatching
	    // Label
	    cv::Rect roi_1( cv::Point(histogram_matching.cols + label.image.cols + 10, 30), cv::Size(label.gray_image.cols, label.gray_image.rows) );
	    cv::Mat temp_image1;
		cv::threshold(label.gray_image, temp_image1, label.threshold_value, 255, CV_THRESH_BINARY);
		cvtColor(temp_image1, temp_image1, CV_GRAY2BGR);
	    temp_image1.copyTo( image_to_draw( roi_1 ) );
	    // Template
	    cv::Rect roi_2( cv::Point(histogram_matching.cols + label.image.cols + 10, label.image.rows + 60), cv::Size(template_label.gray_template_image.cols, template_label.gray_template_image.rows) );
	    cv::Mat temp_image2;
		cv::threshold(template_label.gray_template_image, temp_image2, 0, 255, CV_THRESH_OTSU);
		cvtColor(temp_image2, temp_image2, CV_GRAY2BGR);
	    temp_image2.copyTo( image_to_draw( roi_2 ) );
	}

    // Draw separators
	cv::rectangle(	image_to_draw, 
					cv::Point(histogram_matching.cols, 0), 
					cv::Point(histogram_matching.cols + 10, rows), 
					cv::Scalar(50,50,50), CV_FILLED );
	cv::rectangle(	image_to_draw, 
					cv::Point(histogram_matching.cols + 10, 0), 
					cv::Point(histogram_matching.cols + 10 + 2*label.image.cols, 30), 
					cv::Scalar(50,50,50), CV_FILLED );
	cv::rectangle(	image_to_draw, 
					cv::Point(histogram_matching.cols + 10, label.image.rows + 30 ), 
					cv::Point(histogram_matching.cols + 10 + 2*label.image.cols, label.image.rows + 60), 
					cv::Scalar(50,50,50), CV_FILLED );

	// Draw text
	std::string label_text(" Label: ");
	cv::putText(image_to_draw, label_text, cv::Point(histogram_matching.cols + 10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255), 1);
	std::string _template_label_text(" Template: ");
	_template_label_text.append(template_label.name);
	cv::putText(image_to_draw, _template_label_text, cv::Point(histogram_matching.cols + 10, label.image.rows + 55), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255), 1);
	
	return false;
}


///////////////////////// Debugging Functions /////////////////////////
bool HzlScanner::drawDetectedLabels(shared_ptr<cv::Mat> inputimage)
{	
	if ( _detected_labels.size() == 0)    return false;

	for_each(begin(_detected_labels), end(_detected_labels), [&](const vector<Point2D>& label) {
		cv::line(*inputimage, label[0], label[1], cv::Scalar(0,255,255), 3);
		cv::line(*inputimage, label[1], label[2], cv::Scalar(0,255,255), 3);
		cv::line(*inputimage, label[2], label[3], cv::Scalar(0,255,255), 3);
		cv::line(*inputimage, label[3], label[0], cv::Scalar(0,255,255), 3);

		// cv::line(*inputimage, label[1], findMiddlePoint(label[1], label[0]), cv::Scalar(125,125,255), 4);
		// cv::line(*inputimage, label[1], findMiddlePoint(label[1], label[2]), cv::Scalar(125,125,255), 4);
		cv::line(*inputimage, label[0],findMiddlePoint(label[0], findMiddlePoint(label[0], label[1])), cv::Scalar(0,0,0), 6);
		cv::line(*inputimage, label[0],findMiddlePoint(label[0], findMiddlePoint(label[0], label[3])), cv::Scalar(0,0,0), 6);

		cv::line(*inputimage, label[1],findMiddlePoint(label[1], findMiddlePoint(label[1], label[0])), cv::Scalar(0,0,0), 6);
		cv::line(*inputimage, label[1],findMiddlePoint(label[1], findMiddlePoint(label[1], label[2])), cv::Scalar(0,0,0), 6);

		cv::line(*inputimage, label[2],findMiddlePoint(label[2], findMiddlePoint(label[2], label[1])), cv::Scalar(0,0,0), 6);
		cv::line(*inputimage, label[2],findMiddlePoint(label[2], findMiddlePoint(label[2], label[3])), cv::Scalar(0,0,0), 6);

		cv::line(*inputimage, label[3],findMiddlePoint(label[3], findMiddlePoint(label[3], label[0])), cv::Scalar(0,0,0), 6);
		cv::line(*inputimage, label[3],findMiddlePoint(label[3], findMiddlePoint(label[3], label[2])), cv::Scalar(0,0,0), 6);
	});
	return true;
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
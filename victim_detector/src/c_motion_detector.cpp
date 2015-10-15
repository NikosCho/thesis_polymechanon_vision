#include "victim_detector/c_motion_detector.h"

namespace polymechanon_vision {


MotionDetector::MotionDetector()
{
	static const std::string topic_to_subscribe = loadTopic(_node);
	image_transport::ImageTransport in_(_node);
	_subscriber_to_img_node = in_.subscribe( topic_to_subscribe, 1, &MotionDetector::imageCallback, this);

	loadDetectorSettings(_node);
	_dyn_rec_server.setCallback(boost::bind(&MotionDetector::dynRecCallback, this, _1, _2));


	ROS_WARN("MotionDetector created!");
}



MotionDetector::~MotionDetector(){ 
	ROS_ERROR("MotionDetector deleted!"); 
}


/// Callback function on every new frame.
void MotionDetector::imageCallback(const sensor_msgs::ImageConstPtr& img_sub_msg)
{	
	cv::Mat camera_input, gray_camera_input;
	try  {
		cv_bridge::CvImageConstPtr cv_ptr;
		cv_ptr = cv_bridge::toCvShare(img_sub_msg,"bgr8");
		camera_input = cv_ptr->image;
		cvWaitKey(30);
	}
	catch (cv_bridge::Exception& e)  {
		ROS_ERROR("[MotionDetector]-imageCallback() : Could not convert (image message) from '%s' to 'bgr8'.", img_sub_msg->encoding.c_str());
	}

    static bool FRAMES_CAPTURED = false;
    static bool FRAMES_MESSAGES = true;
    // capture the desired amount of frames (3)
    // ALWAYS keeps the last 3 frames captured, in the right order : oldest, ..., newest (frame)
    cv::cvtColor(camera_input, gray_camera_input, cv::COLOR_BGR2GRAY);
    if ( vectorSequence(gray_camera_input, _frames_sequence, 3) )  {
        FRAMES_CAPTURED = true;
        if (!FRAMES_MESSAGES)  {   
            ROS_INFO("MotionDetection: Enough frames captured..");
            FRAMES_CAPTURED = true; 
            FRAMES_MESSAGES = true;
        }
    } else  {
        FRAMES_CAPTURED = false;
        if (FRAMES_MESSAGES)  {   
            ROS_WARN("MotionDetection: Waiting to capture enough frames..");
            FRAMES_MESSAGES = false;
        }
    }   

    if ( FRAMES_CAPTURED)  {
		detectMotion(camera_input);
	}
	cv::imshow("PhaseCorraletion", camera_input);

}


void MotionDetector::detectMotion(cv::Mat& image_to_draw)
{
	switch(_detection_mode)
	{
		case 0: // Stable mode
			stableCameraDetection(image_to_draw);
			break;
		case 1: // Moving mode
			movingCameraDetection(image_to_draw);

			break;
		default: 
			ROS_ERROR("[MotionDetector]-detectMotion() : unknown mode selected.");
	}

}

void MotionDetector::stableCameraDetection(cv::Mat& image_to_draw)
{
    cv::Mat d1,d2;
    cv::Mat absdiff_mask,motion_mask;

	cv::absdiff(_frames_sequence[0], _frames_sequence[1], d1);
	cv::absdiff(_frames_sequence[0], _frames_sequence[2], d2);

	switch (_thres_method)  {
		case 0: // SIMPLE:
		cv::threshold(d1, d1, _thres_value, 255,cv::THRESH_BINARY);
		cv::threshold(d2, d2, _thres_value, 255,cv::THRESH_BINARY);
	break;
	case 1: // ADAPTIVE_GAUSSIAN:
		cv::adaptiveThreshold(d1, d1, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY , 5,0);
		cv::adaptiveThreshold(d2, d2, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY , 5,0);
		break;
	case 2: // ADAPTIVE_MEAN:
		cv::adaptiveThreshold(d1, d1, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 5,0);
		cv::adaptiveThreshold(d2, d2, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 5,0);
		break;
	case 3: // OTSU:
		cv::threshold(d1, d1, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
		cv::threshold(d2, d2, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
		break;
	default:
		cv::threshold(d1, d1, _thres_value, 255,cv::THRESH_BINARY);
		cv::threshold(d2, d2, _thres_value, 255,cv::THRESH_BINARY);
	}
	
    cv::bitwise_and(d1, d2, motion_mask);
    cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 3, 3 ) );
	cv::erode( motion_mask, motion_mask, element );
	cv::dilate( motion_mask, motion_mask, element ); 

	cv::imshow("Motion_mask", motion_mask);
}


void MotionDetector::movingCameraDetection(cv::Mat& image_to_draw)
{

	cv::Mat trasnformation_mat;
    std::vector<cv::Point2f> prev_corners, curr_corners;
    std::vector<cv::Point2f> matched_prev_corners, matched_curr_corners;
    std::vector<uchar> f_status;
    std::vector<float> f_err;

    std::vector<cv::Point2f> possible_movement_points;
    std::vector<int> point_movement;
    std::vector< std::vector<cv::Point2f> > angle_categorized_moving_points;
    std::vector< std::vector<cv::Point2f> > clusters;

    // const cv::Mat& filtered_transform_vector;

    // cv::goodFeaturesToTrack(_frames_sequence[0], prev_corners, 200, 0.01, 10);
    cv::goodFeaturesToTrack(_frames_sequence[0], prev_corners, 500, 0.01, 10);
    /*
    Parameters: 
        image – Input 8-bit or floating-point 32-bit, single-channel image.
        corners – Output vector of detected corners.
        maxCorners – Maximum number of corners to return. If there are more corners than are found, the strongest of them is returned.
        qualityLevel – Parameter characterizing the minimal accepted quality of image corners. The parameter value is multiplied by the best corner quality measure, which is the minimal eigenvalue (see cornerMinEigenVal() ) or the Harris function response (see cornerHarris() ). The corners with the quality measure less than the product are rejected. For example, if the best corner has the quality measure = 1500, and the qualityLevel=0.01 , then all the corners with the quality measure less than 15 are rejected.
        minDistance – Minimum possible Euclidean distance between the returned corners.
        mask – Optional region of interest. If the image is not empty (it needs to have the type CV_8UC1 and the same size as image ), it specifies the region in which the corners are detected.
        blockSize – Size of an average block for computing a derivative covariation matrix over each pixel neighborhood. See cornerEigenValsAndVecs() .
        useHarrisDetector – Parameter indicating whether to use a Harris detector (see cornerHarris()) or cornerMinEigenVal().
        k – Free parameter of the Harris detector.
    */

    cv::calcOpticalFlowPyrLK(_frames_sequence[0], _frames_sequence[1], prev_corners, curr_corners, f_status, f_err);
    /*
    Parameters: 
        prevImg – first 8-bit input image or pyramid constructed by buildOpticalFlowPyramid().
        nextImg – second input image or pyramid of the same size and the same type as prevImg.
        prevPts – vector of 2D points for which the flow needs to be found; point coordinates must be single-precision floating-point numbers.
        nextPts – output vector of 2D points (with single-precision floating-point coordinates) containing the calculated new positions of input features in the second image; when OPTFLOW_USE_INITIAL_FLOW flag is passed, the vector must have the same size as in the input.
        status – output status vector (of unsigned chars); each element of the vector is set to 1 if the flow for the corresponding features has been found, otherwise, it is set to 0.
        err – output vector of errors; each element of the vector is set to an error for the corresponding feature, type of the error measure can be set in flags parameter; if the flow wasn’t found then the error is not defined (use the status parameter to find such cases).
        winSize – size of the search window at each pyramid level.
        maxLevel – 0-based maximal pyramid level number; if set to 0, pyramids are not used (single level), if set to 1, two levels are used, and so on; if pyramids are passed to input then algorithm will use as many levels as pyramids have but no more than maxLevel.
        criteria – parameter, specifying the termination criteria of the iterative search algorithm (after the specified maximum number of iterations criteria.maxCount or when the search window moves by less than criteria.epsilon.
    */

    matched_prev_corners.clear();   // just to be sure  
    matched_curr_corners.clear();   // just to be sure 
    possible_movement_points.clear();
    point_movement.clear();

    //clean bad matches
    for (size_t i=0; i < f_status.size(); i++)  {
        if ( f_status[i] && f_err[i] < 5.0 )  {
            matched_prev_corners.push_back(prev_corners[i]);
            matched_curr_corners.push_back(curr_corners[i]);

        }
    }


    if (matched_prev_corners.size() > 0 && matched_curr_corners.size() > 0 && matched_prev_corners.size() == matched_curr_corners.size() && matched_curr_corners.size() > 30)  {

        double phCo_radius, phCo_angle, phCo_shift;
        // AngleType phCo_angle_type;
        int phCo_angle_type;

        phaseCorrelation(image_to_draw, phCo_radius, phCo_angle, phCo_shift);

        // phCo_angle_type = angleCategorize(phCo_angle,0,phCo_shift);
        phCo_angle_type = angleCategorize(phCo_angle,0,phCo_shift);
        int phase_angle = static_cast<int>(phCo_angle_type);
        int phase_angle1 = phase_angle + 1;
        int phase_angle2 = phase_angle - 1;
        std::cout << phCo_angle_type << " ------------- " << phase_angle << std::endl;

        // std::cout << static_cast<int>(phCo_angle_type) << std::endl;

        std::vector<double> matching_length, matching_angle;
        calculateLenAng(matched_prev_corners, matched_curr_corners, matching_length, matching_angle);


        if( !image_to_draw.empty() ) {
            // std::cout << matched_prev_corners.size() << "----" << matched_curr_corners.size() << std::endl;
            if (matched_prev_corners.size() == matched_curr_corners.size() &&  matching_length.size() == matching_angle.size() && matched_curr_corners.size() == matching_length.size() ) {
                // Paint cornrers matched

                if(phCo_radius > 5 ) { //&& phCo_radius < 150) {
                    bool ang_status; 
                    bool len_status;
                    // AngleType matched_ang_type;
                    int matched_ang_type;
                    for (size_t k=0; k < matched_curr_corners.size(); k++ ) { 
                        // matched_ang_type = angleCategorize(matching_angle[k], matched_prev_corners[k].x, matched_curr_corners[k].x);
                        matched_ang_type = angleCategorize(matching_angle[k], matched_prev_corners[k].x, matched_curr_corners[k].x);
                        //////////////////////////////////////////////////////////////////////////// WAY 1
                        //  50% -150% length  ---------  50% -150% angle 
                        // if (phCo_radius*0.5 > matching_length[k] || phCo_radius*1.5 < matching_length[k])   len_status =true;
                        // else   len_status = false;
                        // if (phCo_radius > (0.05*camera_input_.cols))  {
                        //     if (phCo_angle*0.5 > matching_angle[k] || phCo_angle*1.5 < matching_angle[k])   ang_status =true;
                        //     else   ang_status = false;
                        // }  else   ang_status = false; 
                        ///////////////////////////////////////////////////////////////////////////////////

                        ////////////////////////////////////////////////////////////////////////// WAY 2
                        //  50% -200% length  
                        if (phCo_radius*0.2 > matching_length[k] || phCo_radius*2.0 < matching_length[k])   len_status = false;
                        else   len_status = false;

                        // if( matching_length[k] > 2.0 ) {
                        //     if (matched_ang_type != phCo_angle_type)  {
                        //         // if (static_cast<int>(phCo_angle_type) + 1 != static_cast<int>(matched_ang_type) || static_cast<int>(phCo_angle_type) - 1 != static_cast<int>(matched_ang_type))
                        //             ang_status = true;
                        //         // ang_status = false;
                        //     }  else   ang_status = false; 
                        // }



                        int point_angle;
                        
                        if( matching_length[k] > 5.0 ) {
                            if (matched_ang_type != phCo_angle_type)  {
                                point_angle = matched_ang_type;

                                if (phase_angle1 == 8 ){
                                    phase_angle1 = 0;
                                }
                                if (phase_angle2 == -1) {
                                    phase_angle2 = 7;
                                }
                                if (phase_angle != point_angle && phase_angle1 != point_angle && phase_angle2 != point_angle) 
                                    ang_status = true;
                                else  
                                    ang_status = false;
                            }  else   ang_status = false; 
                        }

                        ///////////////////////////////////////////////////////////////////////////////////                      


                        if (len_status || ang_status)  {
                            // if (ang_status) std::cout << "(ANGLE uneven) phCo_angle = " << phCo_angle << " ------- matching_angle[k] =" << matching_angle[k] << " -- [ " << phase_angle << " , " << point_angle << " ]" << std::endl; 
                            // if (len_status) std::cout << "(LENGTH uneven) phCo_radius = " << phCo_radius << " ------- matching_length[k] =" << matching_length[k] << std::endl;

                            if (ang_status) std::cout << "(ANGLE uneven) phCo_angle = " << phCo_angle << " ------- matching_angle[k] =" << matching_angle[k] << " -- [ " << phase_angle << " , " << point_angle << " ]" << std::endl; 
                            if (len_status) std::cout << "(LENGTH uneven) phCo_radius = " << phCo_radius << " ------- matching_length[k] =" << matching_length[k] << std::endl;

                            possible_movement_points.push_back(matched_curr_corners[k]);
                            point_movement.push_back(matching_angle[k]);

                            drawArrow( image_to_draw, matched_prev_corners[k], matched_curr_corners[k], cv::Scalar( 0, 0, 255 ), 6, 2, CV_AA, 0);
                        }   else   drawArrow( image_to_draw, matched_prev_corners[k], matched_curr_corners[k], cv::Scalar( 0, 250, 0 ), 4, 1, CV_AA, 0);
                    } //END of 'for: matched_curr_corners.size()'
                }  else  {
                     for (size_t k=0; k < matched_curr_corners.size(); k++ )
                        if (matching_length[k] > 5.0) 
                            drawArrow( image_to_draw, matched_prev_corners[k], matched_curr_corners[k], cv::Scalar( 0, 0, 255 ), 4, 1, CV_AA, 0);
                }
            }//END of 'if (matched_prev_corners.size() == matched_curr_corners.size() &&  matching_length.size() == matching_angle.size() && matched_curr_corners.size() == matching_length.size() )'
        }//END of 'if(output.empty())'
    }//END of 'if (matched_prev_corners.size() > 0 && matched_curr_corners.size() > 0)'
    else    ROS_WARN("Stabilizer2: Unable to calculate trasform. There were no corner matches (in 'calcOpticalFlowPyrLK').");


    for ( int i=0; i<8; i++)
    {
        angle_categorized_moving_points.push_back( std::vector<cv::Point2f>() );
    }
 
    for (int i=0; i < possible_movement_points.size(); i++) {
        if(point_movement[i] >= 0 && point_movement[i] <8)
            angle_categorized_moving_points[point_movement[i]].push_back(possible_movement_points[i]);
        else
            std::cout << "oti nanai" << std::endl;
    }

    cv::RNG rng(12345);

    for (size_t i=0; i < angle_categorized_moving_points.size(); i++) {


        clusters = DBSCAN_points(angle_categorized_moving_points[i], 100.0, 3);
        for(int j=0; j < clusters.size(); j++) {
            cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
            for(int k=0; k < clusters[j].size(); k++) 
                cv::circle( image_to_draw, clusters[j][k], 4, color, -1, 20 );
                // cv::circle( output, clusters[j][k], 4, cv::Scalar( 255, 126, 255 ), -1, 20 );
        }
    }

    std::cout << "SIZES: " << possible_movement_points.size() << " --- " << point_movement.size() << " ============= " << angle_categorized_moving_points.size() << std::endl;
}

void MotionDetector::phaseCorrelation(cv::Mat &frame, double &radius, double &angle, double &shift_x)
{
    // cv::Mat frame, curr, prev, curr64f, prev64f, hann;
    cv::Mat curr, prev, curr64f, prev64f, hann;

    // frame = camera_input_.clone();

    createHanningWindow(hann, _frames_sequence[1].size(), CV_64F);
    /*
    Parameters: 
        dst – Destination array to place Hann coefficients in
        winSize – The window size specifications
        type – Created array type
    */

    _frames_sequence[0].convertTo(prev64f, CV_64F);
    _frames_sequence[1].convertTo(curr64f, CV_64F);

    cv::Point2d shift = phaseCorrelate(prev64f, curr64f, hann);
    /*
    Parameters: 
        src1 – Source floating point array (CV_32FC1 or CV_64FC1)
        src2 – Source floating point array (CV_32FC1 or CV_64FC1)
        window – Floating point array with windowing coefficients to reduce edge effects (optional).
        response – Signal power within the 5x5 centroid around the peak, between 0 and 1 (optional).
    */
    radius = std::sqrt(shift.x*shift.x + shift.y*shift.y);
    if ( shift.x != 0)   angle = -(shift.y / shift.x) ;
    else   angle = 999;
    shift_x = shift.x;

    if(radius > 5)
    {
        // draw a circle and line indicating the shift direction...
        cv::Point center(_frames_sequence[1].cols >> 1, _frames_sequence[1].rows >> 1);
        circle(frame, center, (int)radius, cv::Scalar(125, 125, 125), 3, CV_AA);
        line(frame, center, cv::Point(center.x + (int)shift.x, center.y + (int)shift.y), cv::Scalar(0, 255, 0), 3, CV_AA);
    }
}

void MotionDetector::drawArrow(cv::Mat &image, cv::Point p, cv::Point q, cv::Scalar color, int arrowMagnitude = 9, int thickness=1, int line_type=8, int shift=0) 
{
    //Draw the principle line
    cv::line(image, p, q, color, thickness, line_type, shift);
    const double PI = 3.141592653;
    //compute the angle alpha
    double angle = atan2((double)p.y-q.y, (double)p.x-q.x);
    //compute the coordinates of the first segment
    p.x = (int) ( q.x +  arrowMagnitude * cos(angle + PI/4));
    p.y = (int) ( q.y +  arrowMagnitude * sin(angle + PI/4));
    //Draw the first segment
    cv::line(image, p, q, color, thickness, line_type, shift);
    //compute the coordinates of the second segment
    p.x = (int) ( q.x +  arrowMagnitude * cos(angle - PI/4));
    p.y = (int) ( q.y +  arrowMagnitude * sin(angle - PI/4));
    //Draw the second segment
    cv::line(image, p, q, color, thickness, line_type, shift);
}

bool MotionDetector::calculateLenAng(const std::vector<cv::Point2f> &start,const std::vector<cv::Point2f> &end, std::vector<double> &length, std::vector<double> &angle)
{   
    if( start.size() == 0 || end.size() == 0 )  {
        ROS_WARN("Stabilizer2::calculateLenAng() - empty input vectors.");
        return false;
    } else if ( start.size() != end.size() )  {
        ROS_WARN("Stabilizer2::calculateLenAng() - uneven input vectors.");
        return false; 
    } else {
        length.clear(); angle.clear();
        for (size_t i=0; i < start.size(); i++)  {
            if ( (end[i].x - start[i].x) != 0)   angle.push_back( -((end[i].y - start[i].y ) / ( end[i].x - start[i].x)) );
            else   angle.push_back(999); // some big number
            length.push_back( std::sqrt( std::pow( std::abs(end[i].x - start[i].x), 2) + std::pow( std::abs(end[i].y - start[i].y), 2) ) );
        }
        if (length.size() != angle.size())   return false;
        else   return true;
    }
}

int MotionDetector::angleCategorize(const double &value, const double &x1, const double &x2) // start - end
{   
    // int type_to_return;
    // if ( abs(value) <= 1.0 && value < 0.0 )  {                        // value E [-oo,-1]
    //     if (x1 > x2) type_to_return = 2;
    //     else if (x1 <= x2) type_to_return = 6;
    // }  else if ( 1.0 > abs(value) && value <= 0.0)  {       // value E (-1,0]
    //     if (x1 > x2) type_to_return = 3;
    //     else if (x1 <= x2) type_to_return = 7;
    // }  else if ( 0.0 < value && value <= 1.0 )  {       // value E (0,1]
    //     if (x1 > x2) type_to_return = 4;
    //     else if (x1 <= x2) type_to_return = 0;
    // }  else if ( 1.0 < value )  {                     // value E (1, +oo]
    //     if (x1 > x2) type_to_return = 5;
    //     else if (x1 <= x2) type_to_return = 1;
    // }  else type_to_return = -999;
    // return type_to_return; 
    // std::cout << " PAPARIA " << std::endl;

    int type_to_return;
    if (  value < -1.0 )  {                        // value E [-oo,-1]
        if (x1 > x2) type_to_return = 2;
        else if (x1 <= x2) type_to_return = 6;
    }  else if ( -1.0 < value && value <= 0.0)  {       // value E (-1,0]
        if (x1 > x2) type_to_return = 3;
        else if (x1 <= x2) type_to_return = 7;
    }  else if ( 0.0 < value && value <= 1.0 )  {       // value E (0,1]
        if (x1 > x2) type_to_return = 4;
        else if (x1 <= x2) type_to_return = 0;
    }  else if ( 1.0 < value )  {                     // value E (1, +oo]
        if (x1 > x2) type_to_return = 5;
        else if (x1 <= x2) type_to_return = 1;
    }  else type_to_return = -999;
    return type_to_return; 
    std::cout << " PAPARIA " << std::endl;
}

std::vector< std::vector<cv::Point2f> > MotionDetector::DBSCAN_points(std::vector<cv::Point2f> &points, float eps, int minPts)
{
    std::vector< std::vector<cv::Point2f> > clusters;
    std::vector<bool> clustered, visited;
    std::vector<int> noise;
    std::vector<int> neighborPts, neighborPts_;

    int c;

    int noKeys = points.size();

    //init clustered and visited
    for(int k = 0; k < noKeys; k++)
    {
        clustered.push_back(false);
        visited.push_back(false);
    }

    //C =0;
    c = 0;
    clusters.push_back(std::vector<cv::Point2f>()); //will stay empty?

    //for each unvisted point P in dataset keypoints
    for(int i = 0; i < noKeys; i++)
    {
        if(!visited[i])
        {
            //Mark P as visited
            visited[i] = true;
            neighborPts = regionQuery(points, points[i], eps);
            if(neighborPts.size() < minPts)
                //Mark P as Noise
                noise.push_back(i);
            else
            {
                clusters.push_back(std::vector<cv::Point2f>());
                c++;
                // expand cluster
                // add P to cluster c
                clusters[c].push_back(points[i]);
                //for each point P' in neighborPts
                for(int j = 0; j < neighborPts.size(); j++)
                {
                    //if P' is not visited
                    if(!visited[neighborPts[j]])
                    {
                        //Mark P' as visited
                        visited[neighborPts[j]] = true;
                        neighborPts_ = regionQuery(points, points[neighborPts[j]], eps);
                        if(neighborPts_.size() >= minPts)
                        {
                            neighborPts.insert(neighborPts.end(),neighborPts_.begin(),neighborPts_.end());
                        }
                    }
                    // if P' is not yet a member of any cluster
                    // add P' to cluster c
                    if(!clustered[neighborPts[j]])
                        clusters[c].push_back(points[neighborPts[j]]);
                }
            }

        }
    }
    return clusters;
}

std::vector<int> MotionDetector::regionQuery(std::vector<cv::Point2f> &points, cv::Point2f &point, float eps)
{
    float dist;
    std::vector<int> retKeys;
    for(int i = 0; i< points.size(); i++)
    {
        dist = sqrt(pow((point.x - points[i].x),2)+pow((point.y - points[i].y),2));
        if(dist <= eps && dist != 0.0f)
        {
            retKeys.push_back(i);
        }
    }
    return retKeys;
}

template<typename T>
void MotionDetector::pop_front(std::vector<T>& vec)
{
	assert(!vec.empty());
	vec.erase(vec.begin());
}

template<typename Ta>
bool MotionDetector::vectorSequence(Ta &input_item, std::vector<Ta> &memory, int multitude)
{
    if(!input_item.empty())  {
        // switch(true)  {
        //     case memory.size()==0:
        //         memory.push_back(input_item);
        //         break;
        //     case memory.size() < multitude:
        //         memory.push_back(input_item);
        //         break;
        //     case memory.size() == multitude:
        //         pop_front(memory);
        //         memory.push_back(input_item);
        //         break;
        //     default:
        //         memory.clear();
        //         memory.push_back(input_item);
        // }

        if (memory.size()==0)  {
            memory.push_back(input_item);
        }  else if (memory.size() < multitude)  {
            memory.push_back(input_item);
        }  else if (memory.size() == multitude)  {
            pop_front(memory);
            memory.push_back(input_item);
        }  else  {
            memory.clear();
            memory.push_back(input_item);
        }
    }
    // return memory.size();

    if(memory.size() < multitude )
    	return false;
    else
    	return true;
}

// Load image input topic's name from parameter server.
// If there is no topic's name on server, take the function's paramteter.
std::string MotionDetector::loadTopic(const ros::NodeHandle& node, std::string topic_name /* = "camera/image_raw" */ ) 
{	
	std::string topic_param_name("motion_detector/input_image_topic");

	auto gettopic = [&] (const std::string& name_of_topic_variable) {
		std::string topic_param;
		node.getParam(name_of_topic_variable, topic_param);
		return topic_param;
	};

	std::string topic = node.hasParam(topic_param_name) ? gettopic(topic_param_name) : topic_name;
	ROS_INFO("MotionDetector: subscribed for image input on \"%s\" ", topic.c_str());
	return topic;
}


// Load parameters and settings from parameter server or set them to default.
// If settings aren't on parameter server, set them.
void MotionDetector::loadDetectorSettings(const ros::NodeHandle& node)
{	
	bool other_settings_loaded = false;

	if (node.getParam("motion_detector/Debugging", _debugging))  other_settings_loaded = true;
    else  node.setParam("motion_detector/Debugging", _debugging);
    ROS_INFO("MotionDetector: [DEBUGGING]-%s ", _debugging?"ON":"OFF" );

    if (node.getParam("motion_detector/Detection_mode", _detection_mode))  other_settings_loaded = true;
    else  node.setParam("motion_detector/Detection_mode", _detection_mode);
    ROS_INFO("MotionDetector: [DETECTION_MODE]-%d ", _detection_mode);

    if (node.getParam("motion_detector/Thres_method", _thres_method))  other_settings_loaded = true;
    else  node.setParam("motion_detector/Thres_method", _thres_method);
    ROS_INFO("MotionDetector: [THRESHOLD_METHOD]-%d ", _thres_method);

    if (node.getParam("motion_detector/Thres_value", _thres_value))  other_settings_loaded = true;
    else  node.setParam("motion_detector/Thres_value", _thres_value);
    ROS_INFO("MotionDetector: [THRESHOLD_METHOD]-%d ", _thres_value);

}


// Dynamic reconfigure using command line reconfiguration or rqt-plugin
void MotionDetector::dynRecCallback(victim_detector::MotionDetectorConfig &config, uint32_t level)
{	
    ROS_INFO("MotionDetector -- Reconfigure Request: \n\t[DEBUGGING]-%s \n\t[DETECTION_MODE]-%d \n\t[THRESHOLD_METHOD]-%d n\t[THRESHOLD_VALUE]-%d ",
                config.Debugging?"ON":"OFF",
                config.Detection_mode,
                config.Thres_method,
                config.Thres_value
                );


    _debugging = config.Debugging;
    _detection_mode = config.Detection_mode;
    _thres_method = config.Thres_method;
    _thres_value = config.Thres_value;
}

} // "namespace polymechanon_vision"



#include "res_rob_vision/c_circle_detection.h"

void CircleDetection::imageCallback(const sensor_msgs::ImageConstPtr& img_sub_msg)
{   
    try  {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(img_sub_msg,"bgr8");
        camera_input_ = cv_ptr->image;
        cvWaitKey(30);
    }  catch (cv_bridge::Exception& e)  {
        ROS_ERROR("CircleDetection: Could not convert from '%s' to 'bgr8'.", img_sub_msg->encoding.c_str());
    }

    // Testing if image input is empty
    if(camera_input_.empty()) ROS_ERROR("CircleDetection: Unable to query image from capture device.");
    else  {
        // Debugging's windows names
        static cv::String input_video_window = "CircleDetection: camera input";
        static cv::String processing_video_window = "CircleDetection: processing video";
        static cv::String output_video_window = "CircleDetection: output video";
        // If on DEBUGGING mode, create windows 
        if ( DEBUGGING_ )  {
            cv::namedWindow(input_video_window,1);
            cv::namedWindow(processing_video_window,1);
            cv::namedWindow(output_video_window,1);
        }



        cv::Mat output;
        output = camera_input_.clone();
        cv::Mat hsv_camera_input;
        cv::Mat processing_video;
        // cv::Mat element = getStructuringElement( cv::MORPH_RECT, cv::Size( 2*er_dir_value_ + 1, 2*er_dir_value_+1 ), cv::Point( er_dir_value_, er_dir_value_ ) );
        // cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*er_dir_value_ + 1, 2*er_dir_value_+1 ), cv::Point( er_dir_value_, er_dir_value_ ) );
        cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( er_dir_value_, er_dir_value_ ) );

        cv::Scalar inrange_MIN;
        cv::Scalar inrange_MAX;
        std::vector<cv::Vec3f> VectorCir;
        std::vector<cv::Vec3f>::iterator iterCircles;
        
        static cv::String colors_filename = FreqFunctions::appendPackagePath("/config/area_colors.yml");

        if ( ros::ok() )    {

            loadColors(colors_filename);

            // std::cout <<colors_filename << std::endl;
            inrange_MIN = cv::Scalar(wall_color_hsv_values_[0][0],wall_color_hsv_values_[0][1],wall_color_hsv_values_[0][2]);
            inrange_MAX = cv::Scalar(wall_color_hsv_values_[0][3],wall_color_hsv_values_[0][4],wall_color_hsv_values_[0][5]);

            // cv::equalizeHist(processed_camera_input_,processed_camera_input_);
            // processed_camera_input_ = equalizeIntensity(processed_camera_input_);

            cv::cvtColor(camera_input_,
                            hsv_camera_input,
                            CV_BGR2HSV);
            cv::inRange(hsv_camera_input,
                            inrange_MIN,
                            inrange_MAX,
                            processing_video);


            switch (filter_method_)  {
                case GAUSBLUR_AND_ERODE:
                    cv::GaussianBlur(processing_video,
                            processing_video,
                            cv::Size(9,9),
                            1.5);
                    // Apply the erosion operation
                    erode( processing_video, processing_video, element );
                    break;
                case BLUR_AND_THRSHOLD:
                    cv::blur(processing_video, processing_video, cv::Size(10,10));
                    //threshold again to obtain binary image from blur output
                    cv::threshold(processing_video, processing_video, 20, 255, cv::THRESH_BINARY);
                    break;
                case MORPH_OPENING:
                    cv::bitwise_not(processing_video, processing_video);

                    cv::erode(processing_video, processing_video, element );
                    cv::dilate( processing_video, processing_video, element ); 

                    //morphological closing (fill small holes in the foreground)
                    cv::dilate( processing_video, processing_video, element ); 
                    cv::erode(processing_video, processing_video, element );
                    break;
                default:
                    cv::bitwise_not(processing_video, processing_video);
                    cv::erode(processing_video, processing_video, element );
                    cv::dilate( processing_video, processing_video, element ); 

                    //morphological closing (fill small holes in the foreground)
                    cv::dilate( processing_video, processing_video, element ); 
                    cv::erode(processing_video, processing_video, element );
            }

            cv::HoughCircles(processing_video, //image
                            VectorCir,  //circles
                            CV_HOUGH_GRADIENT, //method
                            2,  //dp 
                            processing_video.rows / HC_minDist_, //minDist
                            HC_param1_, //200,   //param1
                            HC_param2_, //90,    //param2
                            processing_video.rows / HC_minRadius_, //minRadius
                            processing_video.rows / HC_maxRadius_); //maxRadius

            for(iterCircles = VectorCir.begin(); iterCircles != VectorCir.end(); iterCircles++) {
                std::cout << "Ball position x= " << (*iterCircles)[0]
                            << " y = " << (*iterCircles)[1]
                            << " r = " << (*iterCircles)[2] << std::endl;
                cv::circle(output,
                            cv::Point((int)(*iterCircles)[0],(int)(*iterCircles)[1]),
                            3,
                            cv::Scalar(255,0,0),
                            CV_FILLED);
                cv::circle(output,
                            cv::Point((int)(*iterCircles)[0],(int)(*iterCircles)[1]),
                            (int)(*iterCircles)[2],
                            cv::Scalar(0,0,255),
                            3);
            }


            if ( DEBUGGING_ )  {
                if(!camera_input_.empty()) cv::imshow(input_video_window, camera_input_);
                if(!processing_video.empty()) cv::imshow(processing_video_window, processing_video);
                if(!output.empty()) cv::imshow(output_video_window, output);
            }  else  {
                cv::destroyAllWindows();
            }

        }//END of 'while(ros::ok())''
    }//END of else (!camera_input_.empty())
}// END of 'imageCallback' function 


void CircleDetection::loadParameters(const ros::NodeHandle &node, cv::String &topic_to_subscribe, cv::String &topic_to_publish )
{
    if( node.getParam("circle_detection/sub_topic", topic_to_subscribe)) 
        ROS_INFO("Topic to subscribe (selected from launch file): %s ", topic_to_subscribe.c_str());
    
    if( node.getParam("circle_detection/pub_topic", topic_to_publish)) 
        ROS_INFO("Topic to publish (selected from launch file): %s ", topic_to_publish.c_str());

    if ( node.getParam("circle_detection/DEBUGGING", DEBUGGING_) )  {
        if ( !DEBUGGING_ )
            ROS_INFO("Node DEBUGGING: OFF ");
        else if ( DEBUGGING_ )
            ROS_INFO("Node DEBUGGING: ON ");
    }

    int method_to_use;
    if( node.getParam("circle_detection/method", method_to_use))  {
        filter_method_ = static_cast< FilterMethod >(method_to_use);
        ROS_INFO("Threshold method: %s ", getFilterMethodName(filter_method_).c_str());
    }

    if( node.getParam("circle_detection/threshold_value", threshold_value_)) 
        ROS_INFO("Threshold value: %d ", threshold_value_);

    if( node.getParam("circle_detection/erode_dilate_kernel_size", er_dir_value_)) 
        ROS_INFO("Side of erode's kernel: %d ", er_dir_value_);

    if( node.getParam("circle_detection/HC_minDist", HC_minDist_)) 
        ROS_INFO("HC_minDist: %d ", HC_minDist_);
    if( node.getParam("circle_detection/HC_param1", HC_param1_)) 
        ROS_INFO("HC_param1_: %d ", HC_param1_);
    if( node.getParam("circle_detection/HC_param2", HC_param2_)) 
        ROS_INFO("HC_param2_: %d ", HC_param2_);
    if( node.getParam("circle_detection/HC_minRadius", HC_minRadius_)) 
        ROS_INFO("HC_minRadius_: %d ", HC_minRadius_);
    if( node.getParam("circle_detection/HC_maxRadius", HC_maxRadius_)) 
        ROS_INFO("HC_maxRadius_: %d ", HC_maxRadius_);

}

void CircleDetection::dynRecCallback(res_rob_vision::CircleDetectionConfig &config, uint32_t level) {

    ROS_INFO("\n\tCircleDetection -- Reconfigure Request: \n\t\t[DEBUGGING]-%s \n\t\t[METHOD]-%d  [ERODE-SIZE]-%d [THRES-VAL]-%d \n\t\t[HC_minDist]-%d [HC_param1]-%d [HC_param2]-%d [HC_minRadius]-%d [HC_maxRadius]-%d  \n[/INFO]", 
                config.DEBUGGING?"ON":"OFF", 
                config.method, 
                config.threshold_value, 
                config.erode_dilate_kernel_size, 
                config.HC_minDist,
                config.HC_param1,
                config.HC_param2,
                config.HC_minRadius,
                config.HC_maxRadius
                );

    DEBUGGING_ = config.DEBUGGING;
    er_dir_value_ = config.erode_dilate_kernel_size;
    threshold_value_ = config.threshold_value;
    filter_method_ = static_cast<FilterMethod>(config.method);

    HC_minDist_ = config.HC_minDist;
    HC_param1_ = config.HC_param1;
    HC_param2_ = config.HC_param2;
    HC_minRadius_  = config.HC_minRadius;
    HC_maxRadius_ = config.HC_maxRadius;
}

std::string CircleDetection::getFilterMethodName(const FilterMethod what_method)
{
    switch (what_method)  {
        case GAUSBLUR_AND_ERODE:
            return "GAUSBLUR_AND_ERODE";
            break;
        case BLUR_AND_THRSHOLD:
            return "BLUR_AND_THRSHOLD";
            break;
        case MORPH_OPENING:
            return "MORPH_OPENING";
            break;
        default:
            return "GAUSBLUR_AND_ERODE";
    }
}

bool CircleDetection::loadColors(cv::String filename)
{
    cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::READ);

    if (!fs.isOpened())  {   
        ROS_ERROR("CircleDetection: Unable to load color file.");
        return false;
    }

    cv::Mat color_values;
    fs["WALL"] >> color_values;

    for (int i=0; i<6; i++)  {
        wall_color_hsv_values_[0][i] = color_values.at<int>(0,i);
    }

    fs.release();

    // std::cout << "====================== COLORS LOADED!" << std::endl;

    // printColors();
    return true;
}

cv::Mat CircleDetection::equalizeIntensity(const cv::Mat& inputImage)
{
    if(inputImage.channels() >= 3)
    {
        cv::Mat ycrcb;
        cv::cvtColor(inputImage,ycrcb,CV_BGR2YCrCb);

        std::vector<cv::Mat> channels;
        cv::split(ycrcb,channels);

        cv::equalizeHist(channels[0], channels[0]);

        cv::Mat result;
        cv::merge(channels,ycrcb);
        cv::cvtColor(ycrcb,result,CV_YCrCb2BGR);

        return result;
    }

    return cv::Mat();
}

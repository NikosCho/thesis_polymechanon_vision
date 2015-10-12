#include "ros/ros.h" //for ROS

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp> //for OpenCV

#include "std_msgs/String.h" // for Strings

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <vector>

#include <res_rob_vision/config_PP.h>  //PACKAGE_PATH
#include <res_rob_vision/freqfunctions.hpp>

    // Hold windows names
static cv::String original_cam_window = "ConfigureAreaColors: camera_input_";
static cv::String configuration_window = "ConfigureAreaColors: configuration";
static cv::String configured_window = "ConfigureAreaColors: configured camera_input_";

static int erosion_size = 0;

static bool with_threshold = false;

enum BorderColor {YELLOW , ORANGE, RED, WALL};
// yellow: 0,255,255 | orange: BGR-0,122,255 | red: BGR-0,0,255 |

class ConfigureAreaColors
{
public:
	ConfigureAreaColors()
	{	
		// current_color_ = YELLOW;
		// loadColor("/home/nikos/ros_resrob_ws/src/rescrob_cam/include/rescrob_cam/colors_boundaries.yml", current_color_);
		
        // cv::String colors_filename = FreqFunctions::appendPackagePath("/include/res_rob_vision/area_colors.yml");
        cv::String colors_filename = FreqFunctions::appendPackagePath("/config/area_colors.yml");
        loadColors(colors_filename);

		image_transport::ImageTransport it_(n_);
		sub_ = it_.subscribe("camera/image_raw", 1, &ConfigureAreaColors::imageCallback, this);
		pub_ = it_.advertise("camera/image_raw/borders_processed", 1);
		
	}

	void imageCallback(const sensor_msgs::ImageConstPtr& msg);

	void drawColorsOnImage(cv::Mat &input_imag);


private:
	cv::Mat camera_input_;
	cv::Mat hsv_camera_input_;
	cv::Mat configured_camera_input_;
	
	ros::NodeHandle n_; 
	image_transport::Subscriber sub_;		// Subscribing to camera publisher
	image_transport::Publisher pub_;		// Publishing configured image

	BorderColor current_color_;
	int current_color_boundaries_[6];		// [HSV] 0-2: minimum values | 3-5: maximum values
	int color_boundaries_[5][7];

	bool loadColors(cv::String filename);
	bool saveColors(cv::String filename);

	void saveColor(int whatcolor);
	void printColors();

	void updateCurrentColor(cv::Mat inputcolormat);
	void updateTrackbars(int whatcolor);
	cv::String getStringColorName(BorderColor whatcolor);
	// void printCurrentColor();

};//End of class QrExtractor
//__________________________________________________________________________
void ConfigureAreaColors::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{	
    try  {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(msg,"bgr8");
        camera_input_ = cv_ptr->image;
        cvWaitKey(30);
    }  catch (cv_bridge::Exception& e)  {
        ROS_ERROR("qr_extractor: Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    // Testing if image input is empty
    if(camera_input_.empty())  ROS_ERROR("qr_extractor: Unable to query image from capture device.");

    cvtColor(camera_input_,hsv_camera_input_, cv::COLOR_BGR2HSV);
    // memset(current_color_boundaries_, 0, sizeof(current_color_boundaries_));        // intializing color boundaries to zero

    // cv::String colors_filename = "/home/nikos/ros_resrob_ws/src/rescrob_cam/include/rescrob_cam/colors_boundaries.yml";

    cv::String colors_filename = FreqFunctions::appendPackagePath("/config/area_colors.yml");    


    // // Hold windows names
    // cv::String original_cam_window = "ConfigureAreaColors: camera_input_";
    // cv::String configuration_window = "ConfigureAreaColors: configuration";
    // cv::String configured_window = "ConfigureAreaColors: configured camera_input_";
    
    static int DEBUG_MODE=0,current_color=0,save_flag=0,load_flag=0,temp_color=0;




    // Create named windows
    cv::namedWindow(original_cam_window);
    cv::namedWindow(configuration_window);
    cv::namedWindow(configured_window);

    cv::Mat configured_camera_input = camera_input_.clone();
    // Convert and hold camera_input_ to HSV channel
    // cv::Mat hsv_camera_input;
//////////////////////////////////////
    cv::createTrackbar( "Erosion Circle:  ", original_cam_window, &erosion_size, 20);  
    cv::Mat element1 = getStructuringElement( cv::MORPH_ELLIPSE,
                                   cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                   cv::Point( erosion_size, erosion_size ) );

    cv::erode( camera_input_, camera_input_, element1 );
    // processed_camera_input_ = camera_input_.clone();

        // ERODE RECT ELEMENT
    cv::createTrackbar( "Erosion Rect:  ", original_cam_window, &erosion_size, 20);  
    cv::Mat element2 = getStructuringElement( cv::MORPH_RECT,
                                   cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                   cv::Point( erosion_size, erosion_size ) );
    cv::erode( camera_input_, camera_input_, element2 );
    //////
    /////////////////////////////////////////////////////////////////////////////

    
    cvtColor(camera_input_,hsv_camera_input_, cv::COLOR_BGR2HSV);
    
    cv::Mat mask;
    cv::Scalar inrange_MIN_value;
    cv::Scalar inrange_MAX_value;

    std::vector<cv::Vec4i> border_lines;
    std::vector<cv::Point> border_rectangles;
    cv::Vec4i temp_line; //?

    // int temp_color;
    int temp_save_flag;
    int temp_load_flag;

    // Create trackbars of 'configuration_window'
    // Debugger's trackbar [ 0-off | 1-on ]
    // cv::createTrackbar( "DEBUG_mode" , configuration_window , &DEBUG_MODE, 1 );
    cv::createTrackbar( "Color:  ", configuration_window, &current_color, 4);       // [0-NULL 1-YELLOW | 2-ORANGE | 3-RED]
    cv::createTrackbar( "Save color: ", configuration_window, &save_flag, 1);       // [0-off | 1-on]
    cv::createTrackbar( "Load color: ", configuration_window, &load_flag, 1);       // [0-off | 1-on]

    // minValue trackbar
    cv::createTrackbar("min_H", configuration_window, &color_boundaries_[current_color][1], 255);     //49
    cv::createTrackbar("min_S", configuration_window, &color_boundaries_[current_color][2], 255);     //100
    cv::createTrackbar("min_V", configuration_window, &color_boundaries_[current_color][3], 255);     //78
    // maxValue trackbar
    cv::createTrackbar("max_H", configuration_window, &color_boundaries_[current_color][4], 255);     //83
    cv::createTrackbar("max_S", configuration_window, &color_boundaries_[current_color][5], 255);     //146
    cv::createTrackbar("max_V", configuration_window, &color_boundaries_[current_color][6], 255);     //132
            
    if ( ros::ok() )  {
        if( temp_color != current_color )  {
            saveColor(temp_color);
            temp_color = (BorderColor)current_color;
            updateTrackbars(current_color);
        }
        if ( temp_save_flag != save_flag )  {
            if(save_flag)  {
                saveColor(temp_color);
                saveColors(colors_filename);
            }
            temp_save_flag = save_flag;
        }
        if ( temp_load_flag != load_flag )  {
            if(load_flag)  {
                loadColors(colors_filename);
                updateTrackbars(temp_color);
            }
            temp_load_flag = load_flag;
        }

        inrange_MIN_value = cv::Scalar(cv::getTrackbarPos("min_H", configuration_window),cv::getTrackbarPos("min_S", configuration_window),cv::getTrackbarPos("min_V", configuration_window));
        inrange_MAX_value = cv::Scalar(cv::getTrackbarPos("max_H", configuration_window),cv::getTrackbarPos("max_S", configuration_window),cv::getTrackbarPos("max_V", configuration_window));
        cv::inRange(hsv_camera_input_, inrange_MIN_value, inrange_MAX_value, mask); // if the frame has any orange pixel, this will be painted in the mask as white

        // Show windows
        drawColorsOnImage(configured_camera_input);
        cv::imshow(original_cam_window , camera_input_);
        cv::imshow(configuration_window , mask);
        cv::imshow(configured_window , configured_camera_input);

    }//END of 'while(ros::ok())''
}// END of 'imageCallback' function 

//_____________________________________________________________________________
bool ConfigureAreaColors::loadColors(cv::String filename)
{   
    cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::READ);

    if (!fs.isOpened())  {   
        ROS_ERROR("ConfigureAreaColors: Unable to load color file.");
        std::cout << filename << std::endl;
        return false;
    }

    cv::Mat color_values0,color_values1,color_values2,color_values3;
    fs["YELLOW"] >> color_values0;
    fs["ORANGE"] >> color_values1;
    fs["RED"] >> color_values2;
    fs["WALL"] >> color_values3;

    color_boundaries_[1][0] = 0;
    color_boundaries_[2][0] = 1;
    color_boundaries_[3][0] = 2;
    color_boundaries_[4][0] = 3;

    for (int i=1; i<7; i++)  {
        color_boundaries_[1][i] = color_values0.at<int>(0,i-1);
        color_boundaries_[2][i] = color_values1.at<int>(0,i-1);
        color_boundaries_[3][i] = color_values2.at<int>(0,i-1);
        color_boundaries_[4][i] = color_values3.at<int>(0,i-1);
    }

    fs.release();

    std::cout << "====================== COLORS LOADED!" << std::endl;
    return true;
}

//_____________________________________________________________________________
bool ConfigureAreaColors::saveColors(cv::String filename)
{   
    cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::WRITE);

    if (!fs.isOpened())  {   
        ROS_ERROR("ConfigureAreaColors: Unable to save color file.");
        return false;
    }

    cv::Mat color_to_save0 = (cv::Mat_<int>(1,6) << color_boundaries_[1][1], color_boundaries_[1][2], color_boundaries_[1][3], color_boundaries_[1][4], color_boundaries_[1][5], color_boundaries_[1][6]);
    cv::Mat color_to_save1 = (cv::Mat_<int>(1,6) << color_boundaries_[2][1], color_boundaries_[2][2], color_boundaries_[2][3], color_boundaries_[2][4], color_boundaries_[2][5], color_boundaries_[2][6]);
    cv::Mat color_to_save2 = (cv::Mat_<int>(1,6) << color_boundaries_[3][1], color_boundaries_[3][2], color_boundaries_[3][3], color_boundaries_[3][4], color_boundaries_[3][5], color_boundaries_[3][6]);
    cv::Mat color_to_save3 = (cv::Mat_<int>(1,6) << color_boundaries_[4][1], color_boundaries_[4][2], color_boundaries_[4][3], color_boundaries_[4][4], color_boundaries_[4][5], color_boundaries_[4][6]);
    
    fs << getStringColorName((BorderColor)color_boundaries_[1][0]) << color_to_save0
            << getStringColorName((BorderColor)color_boundaries_[2][0]) << color_to_save1
            << getStringColorName((BorderColor)color_boundaries_[3][0]) << color_to_save2
            << getStringColorName((BorderColor)color_boundaries_[4][0]) << color_to_save3;

    // fs << getStringColorName((BorderColor)color_boundaries_[1][0]) << color_to_save0 << getStringColorName((BorderColor)color_boundaries_[2][0]) << color_to_save1 << getStringColorName((BorderColor)color_boundaries_[3][0]) << color_to_save2;

    fs.release();

    std::cout << "====================== COLORS SAVED!" << std::endl;
    return true;
}

void ConfigureAreaColors::saveColor(int whatcolor)
{   
    // std::cout << whatcolor << std::endl;
    color_boundaries_[whatcolor][1] = cv::getTrackbarPos("min_H", configuration_window);     
    color_boundaries_[whatcolor][2] = cv::getTrackbarPos("min_S", configuration_window);     
    color_boundaries_[whatcolor][3] = cv::getTrackbarPos("min_V", configuration_window);     
    color_boundaries_[whatcolor][4] = cv::getTrackbarPos("max_H", configuration_window);     
    color_boundaries_[whatcolor][5] = cv::getTrackbarPos("max_S", configuration_window);     
    color_boundaries_[whatcolor][6] = cv::getTrackbarPos("max_V", configuration_window);

}

void ConfigureAreaColors::updateTrackbars(int whatcolor)
{
    // update trackbars
    cv::setTrackbarPos("min_H", configuration_window, color_boundaries_[whatcolor][1]);     
    cv::setTrackbarPos("min_S", configuration_window, color_boundaries_[whatcolor][2]);     
    cv::setTrackbarPos("min_V", configuration_window, color_boundaries_[whatcolor][3]);     
    cv::setTrackbarPos("max_H", configuration_window, color_boundaries_[whatcolor][4]);     
    cv::setTrackbarPos("max_S", configuration_window, color_boundaries_[whatcolor][5]);     
    cv::setTrackbarPos("max_V", configuration_window, color_boundaries_[whatcolor][6]);
}

cv::String ConfigureAreaColors::getStringColorName(BorderColor whatcolor)
{
    cv::String colorname;
    switch (whatcolor)  {
        case YELLOW:
            return "YELLOW";
            break;
        case ORANGE:
            return "ORANGE";
            break;
        case RED:
            return "RED";
            break;
        case WALL:
            return "WALL";
            break;
        default:
            return "NOCOLOR";
    }
}

void ConfigureAreaColors::printColors()
{   
    std::cout << "===== COLORS =====" << std::endl;
    for (int i=1; i<5; i++)  {
        std::cout << "\tColor: |" << getStringColorName((BorderColor)color_boundaries_[i][0]) << "| ---"
                << "\tMIN-[ " << color_boundaries_[i][1] << ", " << color_boundaries_[i][2] << ", " << color_boundaries_[i][3] << " ]" 
                << "\tMAX-[ " << color_boundaries_[i][4] << ", " << color_boundaries_[i][5] << ", " << color_boundaries_[i][6] << " ]" << std::endl;
    }
}

void ConfigureAreaColors::drawColorsOnImage(cv::Mat &input_image)
{   
    cv::Mat mask;
    cv::Scalar lowerb;
    cv::Scalar upperb;
    for(int i=1; i<5; i++)  {
        lowerb = cv::Scalar(color_boundaries_[i][1],color_boundaries_[i][2],color_boundaries_[i][3]);
        upperb = cv::Scalar(color_boundaries_[i][4],color_boundaries_[i][5],color_boundaries_[i][6]);
        cv::inRange(hsv_camera_input_, lowerb, upperb, mask);

        if ( with_threshold )  {
            cv::blur(mask,mask,cv::Size(10,10));
            //threshold again to obtain binary image from blur output
            cv::threshold(mask,mask,20,255,cv::THRESH_BINARY);
        }

        switch((BorderColor)color_boundaries_[i][0])  {
            case YELLOW:
                input_image.setTo(cv::Scalar(0,255,255),mask);
                break;
            case ORANGE:
                input_image.setTo(cv::Scalar(0,122,255),mask);
                break;
            case RED:
                input_image.setTo(cv::Scalar(0,0,255),mask);
                break;
            case WALL:
                input_image.setTo(cv::Scalar(25,42,73),mask);
                break;
            default:
                std::cout << "xese mesa" << std::endl;    
        }
    }
}




int main(int argc, char **argv)
{ 
	ros::init(argc, argv, "configure_colors");

	ConfigureAreaColors configurator;

	ros::spin();
}



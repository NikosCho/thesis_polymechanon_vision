#include "res_rob_vision/c_borders_detection.h"

#include <string>
using std::string;
using std::stringstream;

void BordersDetection::imageCallback(const sensor_msgs::ImageConstPtr& img_sub_msg)
{   
    try  {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(img_sub_msg,"bgr8");
        camera_input_ = cv_ptr->image;
        cvWaitKey(30);
    }  catch (cv_bridge::Exception& e)  {
        ROS_ERROR("qr_extractor: Could not convert from '%s' to 'bgr8'.", img_sub_msg->encoding.c_str());
    }

    // Testing if image input is empty
    if(camera_input_.empty()) ROS_ERROR("qr_extractor: Unable to query image from capture device.");

    // Debugging's windows names
    static cv::String input_video_window = "BordersDetection: camera input";
    static cv::String output_video_window = "BordersDetection: output video";
    // If on DEBUGGING mode, create windows 
    if ( DEBUGGING_ )  {
        cv::namedWindow(input_video_window,1);
        cv::namedWindow(output_video_window,1);
    }

    cv::Mat output;
    output = camera_input_.clone();
    cv::Mat hsv_camera_input;
    sensor_msgs::ImageConstPtr img_pub_msg;


    static cv::String colors_filename = FreqFunctions::appendPackagePath("/config/area_colors.yml");


    if ( ros::ok() )    {

        // cv::cvtColor(camera_input_, output, CV_BGR2GRAY);

        loadColors(colors_filename);
        cvtColor(camera_input_,hsv_camera_input, cv::COLOR_BGR2HSV);
        // printColors();
        // drawBorders(output,hsv_camera_input);
        drawALLBorders(output,hsv_camera_input);
        

        // std::cout <<colors_filename << std::endl;

        // cv::cvtColor(output, output, CV_GRAY2BGR);
        // Publish output image (borders cut)
        if(!output.empty()) {
            img_pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output).toImageMsg();
            imag_pub_.publish(img_pub_msg);
        }


        if ( DEBUGGING_ )  {
            if(!camera_input_.empty()) cv::imshow(input_video_window, camera_input_);
            if(!output.empty()) cv::imshow(output_video_window, output);
        }  else  {
            cv::destroyAllWindows();
        }

    }//END of 'if(ros::ok())''
}// END of 'imageCallback' function 


void BordersDetection::loadParameters(const ros::NodeHandle &node, cv::String &topic_to_subscribe, cv::String &topic_to_publish )
{
    if( node.getParam("area_borders/sub_topic", topic_to_subscribe)) 
        ROS_INFO("Topic to subscribe (selected from launch file): %s ", topic_to_subscribe.c_str());
    
    if( node.getParam("area_borders/pub_topic", topic_to_publish)) 
        ROS_INFO("Topic to publish (selected from launch file): %s ", topic_to_publish.c_str());

    if ( node.getParam("area_borders/DEBUGGING", DEBUGGING_) )  {
        if ( !DEBUGGING_ )
            ROS_INFO("Node DEBUGGING: OFF ");
        else if ( DEBUGGING_ )
            ROS_INFO("Node DEBUGGING: ON ");
    }

    // Color parameters
    bool show_color;        // temporarilu holds color boolean
    if ( node.getParam("area_borders/color_yellow", show_color) )  {
        show_colors_[0] = show_color;
        if ( !show_colors_[0] )
            ROS_INFO("Color-Yellow: OFF ");
        else if ( show_colors_[0] )
            ROS_INFO("Color-Yellow: ON ");
    }

    if ( node.getParam("area_borders/color_orange", show_color) )  {
        show_colors_[1] = show_color;
        if ( !show_colors_[0] )
            ROS_INFO("Color-Orange: OFF ");
        else if ( show_colors_[0] )
            ROS_INFO("Color-Orange: ON ");
    }

    if ( node.getParam("area_borders/color_red", show_color) )  {
        show_colors_[2] = show_color;
        if ( !show_colors_[0] )
            ROS_INFO("Color-Red: OFF ");
        else if ( show_colors_[0] )
            ROS_INFO("Color-Red: ON ");
    }

    if( node.getParam("area_borders/HL_threshold", houghlinesp_parameters_[0])) 
        ROS_INFO("HoughLinesP: Accumulator threshold parameter: %d ", houghlinesp_parameters_[0]);
    if( node.getParam("area_borders/HL_minLineLength", houghlinesp_parameters_[1])) 
        ROS_INFO("HoughLinesP: Minimum line length. Line segments shorter than that are rejected: %d ", houghlinesp_parameters_[1]);
    if( node.getParam("area_borders/HL_maxLineGap", houghlinesp_parameters_[2])) 
        ROS_INFO("HoughLinesP: Maximum allowed gap between points on the same line to link them: %d ", houghlinesp_parameters_[2]);

}

void BordersDetection::dynRecCallback(res_rob_vision::AreaBordersConfig &config, uint32_t level) 
{
    // Print info message with reconfiguration
    ROS_INFO("\n\tBordersDetection -- Reconfigure Request: \n\t\t[DEBUGGING]-%s \n\t\t[YELLOW]-%s [ORANGE]-%s [RED]-%s \n\t\t[HOUGHLINESP-PARAMETERS] [HL_threshold]-%d [HL_minLineLength]-%d [HL_maxLineGap]-%d  \n[/INFO]",
                config.DEBUGGING?"ON":"OFF",
                config.color_yellow?"ON":"OFF",
                config.color_orange?"ON":"OFF",
                config.color_red?"ON":"OFF",
                config.HL_threshold,
                config.HL_minLineLength,
                config.HL_maxLineGap);

    // Assign the reconfigured parameters
    DEBUGGING_ = config.DEBUGGING;
    show_colors_[0] = config.color_yellow;
    show_colors_[0] = config.color_orange;
    show_colors_[0] = config.color_red;
    houghlinesp_parameters_[0] = config.HL_threshold;
    houghlinesp_parameters_[1] = config.HL_minLineLength;
    houghlinesp_parameters_[2] = config.HL_maxLineGap;
}

bool BordersDetection::loadColors(cv::String filename)
{
    cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::READ);

    if (!fs.isOpened())  {   
        ROS_ERROR("c_cam_processed_publish: Unable to load color file.");
        return false;
    }

    cv::Mat color_values0,color_values1,color_values2,color_values3;
    fs["YELLOW"] >> color_values0;
    fs["ORANGE"] >> color_values1;
    fs["RED"] >> color_values2;
    fs["WALL"] >> color_values3;

    color_hsv_values_[0][0] = 0;
    color_hsv_values_[1][0] = 1;
    color_hsv_values_[2][0] = 2;
    color_hsv_values_[3][0] = 3;

    for (int i=1; i<7; i++)  {
        color_hsv_values_[0][i] = color_values0.at<int>(0,i-1);
        color_hsv_values_[1][i] = color_values1.at<int>(0,i-1);
        color_hsv_values_[2][i] = color_values2.at<int>(0,i-1);
        color_hsv_values_[3][i] = color_values3.at<int>(0,i-1);
    }

    fs.release();

    // std::cout << "====================== COLORS LOADED!" << std::endl;

    // printColors();
    return true;
}

void BordersDetection::drawBorders(cv::Mat &input_image, cv::Mat &hsv_input_image)
{   
    std::vector<cv::Point> border_upper_rectangle;
    std::vector<cv::Vec4i> lines;
    cv::Vec4i templine;

    std::vector<cv::Point2d> line_to_draw;

    cv::Mat mask;
    cv::Scalar inrange_MIN;
    cv::Scalar inrange_MAX;
    for(int i=0; i<3; i++)  {
        inrange_MIN = cv::Scalar(color_hsv_values_[i][1],color_hsv_values_[i][2],color_hsv_values_[i][3]);
        inrange_MAX = cv::Scalar(color_hsv_values_[i][4],color_hsv_values_[i][5],color_hsv_values_[i][6]);
        cv::inRange(hsv_input_image, inrange_MIN, inrange_MAX, mask);
        lines.clear();

        cv::HoughLinesP( mask, lines, 5, 2*CV_PI/180, houghlinesp_parameters_[0], houghlinesp_parameters_[1], houghlinesp_parameters_[2] );

        if (lines.size() > 0)  {

            line_to_draw.clear();
            findPerfectPoints(lines,line_to_draw);

            if (line_to_draw.size() > 0)  {
                border_upper_rectangle.clear();
                border_upper_rectangle.push_back(cv::Point(line_to_draw[0].x, line_to_draw[0].y));
                border_upper_rectangle.push_back(cv::Point(line_to_draw[1].x, line_to_draw[1].y));
                border_upper_rectangle.push_back(cv::Point(line_to_draw[1].x, 0));
                border_upper_rectangle.push_back(cv::Point(line_to_draw[0].x, 0));

                switch((BorderColor)color_hsv_values_[i][0])  {
                    case YELLOW:
                    
                        // std::cout << i << "---" << color_hsv_values_[i][0] <<std::endl;
                            cv::fillConvexPoly(input_image, border_upper_rectangle,cv::Scalar(0,0,0), 8, 0);
                            cv::line( input_image, cv::Point(line_to_draw[0].x, line_to_draw[0].y),
                                        cv::Point(line_to_draw[1].x, line_to_draw[1].y), cv::Scalar(0,255,255), 4);
                        break;
                    case ORANGE:
                    // std::cout << i << "---" << color_hsv_values_[i][0] <<std::endl;
                            cv::fillConvexPoly(input_image, border_upper_rectangle,cv::Scalar(0,0,0), 8, 0);
                            cv::line( input_image, cv::Point(line_to_draw[0].x, line_to_draw[0].y),
                                        cv::Point(line_to_draw[1].x, line_to_draw[1].y), cv::Scalar(0,125,255), 4);
                        break;
                    case RED:
                            cv::fillConvexPoly(input_image, border_upper_rectangle,cv::Scalar(0,0,0), 8, 0);
                            cv::line( input_image, cv::Point(line_to_draw[0].x, line_to_draw[0].y),
                                        cv::Point(line_to_draw[1].x, line_to_draw[1].y), cv::Scalar(0,0,255), 4);   
                        break;
                    default:
                        ROS_ERROR("BordersDetection::drawBorders: invalid color name.");    
                }
            }
        }
    }
}

void BordersDetection::drawALLBorders(cv::Mat &input_image, cv::Mat &hsv_input_image)
{   
    std::vector<cv::Point> border_upper_rectangle;
    std::vector<cv::Vec4i> lines;
    cv::Vec4i templine;

    std::vector<cv::Point2d> lowestline;

    cv::Mat mask;
    cv::Scalar inrange_MIN;
    cv::Scalar inrange_MAX;
    for(int i=0; i<3; i++)  {
        inrange_MIN = cv::Scalar(color_hsv_values_[i][1],color_hsv_values_[i][2],color_hsv_values_[i][3]);
        inrange_MAX = cv::Scalar(color_hsv_values_[i][4],color_hsv_values_[i][5],color_hsv_values_[i][6]);
        cv::inRange(hsv_input_image, inrange_MIN, inrange_MAX, mask);

        // if ( i == 2) {

            stringstream text;
            text << "aretouli_mask" << i;
            string window_name_mask = text.str();
            stringstream text2;
            text2 << "aretouli_edges" << i;
            string window_name_edges = text2.str();

        // }
        /////////////////////////////////////////
        // cv::Mat mask_edges(mask.size(), CV_MAKETYPE(mask.depth(), 1));   // To hold Grayscale Image
        cv::Mat mask_edges;   // To hold Grayscale Image

        Canny(mask, mask_edges, 100 , 200, 3);   // Apply Canny edge detection on the img_gray image
        if ( i == 2) {
            cv::imshow(window_name_edges, mask_edges);
            cv::imshow(window_name_mask, mask);
        }
            // cv::imshow(window_name_mask, mask);

        cv::HoughLinesP( mask, lines, 5, 2*CV_PI/180, houghlinesp_parameters_[0], houghlinesp_parameters_[1], houghlinesp_parameters_[2] );

        // cv::Mat mymask(mask.rows,mask.cols, CV_8UC1  );
        cv::Mat mymask = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC1);

        double x_ = .0;
        double y_ = .0;
        // double xy = .0;

        int points_counter = 0;

        // std::cout << 
        // // mask.at<double>(300,240) 
        // // mask.get(300,240) 
        // mask.at<uchar>(300,240) 
        // << std::endl;

        //         cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 3, 3 ) );
        // cv::dilate( mask, mask, element ); 
        // cv::erode( mask, mask, element );
        //         cv::imshow("geia", mask);

        int y_counter = 0;
        int y_sum = 0;

        // for ( int px = 0; px < mask.cols; px++)
        // {   y_counter = 0;
        //     y_sum = 0;
        //     for ( int py = 0; py < mask.rows; py++)
        //     {
        //         if ( mask.at<uchar>(py,px) != 0) {
        //             y_sum += py;
        //             y_counter++;
        //             // mask.at<uchar>(px,py) = '\0';
        //         }
        //     }
        //     if ( y_counter > 10 )
        //         // std::cout << y_sum / y_counter << std::endl;
        //         mymask.at<uchar>(y_sum / y_counter, px) = 255;
        // }
        // if (i == 2) {
        //     cv::imshow("wraia", mymask);
        // }

        for ( int px = 0; px < mask.cols; px++)
        {
            for ( int py = 0; py < mask.rows; py++)
            {
                if ( mask.at<uchar>(py,px) != 0) {
                    // xy += px * py;
                    x_ += px;
                    y_ += py;
                    points_counter++;
                }
            }
        }

        if ( points_counter != 0 )
        {
            x_ = x_ / points_counter;
            y_ = y_ / points_counter;
        }


        // std::cout << "=== " << i <<" \n" <<
        // countNonZero(mask) << "---" <<
        // points_counter
        // << std::endl;

        // std::cout << "=== " << i <<" \n" <<
        // x_ << "---" << y_
        // << std::endl;

        if (i == 2) {
            double sum = .0;
            double sx2 = .0;

            for ( int px = 0; px < mask.cols; px++)
            {
                for ( int py = 0; py < mask.rows; py++)
                {

                    if ( mask.at<uchar>(py,px) != 0) {

                        sum += (px - x_)*(py - y_);
                        sx2 += pow(px - x_,2);
                        // sum += (static_cast<double>(px) - x_)* (static_cast<int>(py) - y_);
                        // sx2 += pow(static_cast<double>(px) - x_,2);

                    }

                }
            }
            double slope = sum / sx2;

            // double slope = ( points_counter*xy - SumX*SumY ) / ( points_counter*SumX2 -  pow(SumX, 2));            

            // double slope = sum / sx2;
            double c = y_ - slope*x_;

        std::cout << "=== " << i <<" \n" <<
slope
        << std::endl;


            std::cout << "=== " << i <<" \n" <<
            slope 
            << std::endl;        

            Point2D p, q;
            p.x = 0;
            q.x = input_image.cols;
            p.y = slope * p.x + c;
            q.y = slope * q.x + c;


            cv::line(input_image, p, q, cv::Scalar(0,255,0), 10);


        }
        //////////////////////////////////////////

        // cv::HoughLinesP( mask, lines, 5, 2*CV_PI/180, 100, 100, 50 );

        
        // findBiggest(lines,lowestline);

        // findLowestPoints(lines,lowestline);
        // findGoodPoints(lines,lowestline);

        // for ( int j=0; j<lines.size(); j++)  {
        //     templine = lines[j];
        //     // border_upper_rectangle.clear();
        //     // border_upper_rectangle.push_back(cv::Point(templine[0], templine[1]));
        //     // border_upper_rectangle.push_back(cv::Point(templine[2], templine[3]));
        //     // border_upper_rectangle.push_back(cv::Point(templine[2], 0));
        //     // border_upper_rectangle.push_back(cv::Point(templine[0], 0));
        //     border_upper_rectangle.clear();
        //     border_upper_rectangle.push_back(cv::Point(templine[0], templine[1]));
        //     border_upper_rectangle.push_back(cv::Point(templine[2], templine[3]));

        //     border_upper_rectangle.push_back(cv::Point(templine[2], 0));
        //     border_upper_rectangle.push_back(cv::Point(templine[0], 0));

        //     switch((BorderColor)color_hsv_values_[i][0])  {
        //         case YELLOW:
        //             // cv::fillConvexPoly(input_image, border_upper_rectangle,cv::Scalar(0,0,0), 8, 0);
        //             cv::line( input_image, cv::Point(templine[0], templine[1]),
        //                         cv::Point(templine[2], templine[3]), cv::Scalar(0,255,255), 4);
        //             break;
        //         case ORANGE:
        //             // cv::fillConvexPoly(input_image, border_upper_rectangle,cv::Scalar(0,0,0), 8, 0);
        //             cv::line( input_image, cv::Point(templine[0], templine[1]),
        //                         cv::Point(templine[2], templine[3]), cv::Scalar(0,122,255), 4);;
        //             break;
        //         case RED:
        //             // cv::fillConvexPoly(input_image, border_upper_rectangle,cv::Scalar(0,0,0), 8, 0);
        //             cv::line( input_image, cv::Point(templine[0], templine[1]),
        //                         cv::Point(templine[2], templine[3]), cv::Scalar(0,0,255), 4);
        //             break;
        //         default:
        //             std::cout << "xese mesa" << std::endl;    
        //     }
        // }
    }
}

float BordersDetection::findLength(cv::Point2d A, cv::Point2d B)
{
  return sqrt(pow(abs(A.x - B.x),2) + pow(abs(A.y - B.y),2)); 
}

float BordersDetection::findLength(cv::Vec4i line)
{
  return sqrt(pow(abs(line[0] - line[2]),2) + pow(abs(line[1] - line[3]),2)); 
}

void BordersDetection::findPerfectPoints(std::vector<cv::Vec4i> &lines,std::vector<cv::Point2d> &templine)
{

    cv::Point2d pointone;
    cv::Point2d pointtwo;

    findBiggest(lines,templine);
    int length_MAX = findLength(templine[0],templine[1]);

    pointone = templine[0];
    pointtwo = templine[1];

    templine.clear();

    int multitude_counter = 0;

    int myval0=0;
    int myval1=0;
    int myval2=0;
    int myval3=0;
    // pointone.clean(); pointtwo.clear();
    for( size_t i = 0; i < lines.size(); i++ )  {
        if ( findLength(lines[i]) > (length_MAX * 0.8) )  {
            myval0 += lines[i][0];
            myval1 += lines[i][1];
            myval2 += lines[i][2];
            myval3 += lines[i][3];

            multitude_counter++;
        }
    }

    if( multitude_counter>1 )  {

        pointone.y = (double)(myval1 / multitude_counter);
        pointtwo.y = (double)(myval3 / multitude_counter);

        templine.push_back(pointone);
        templine.push_back(pointtwo);
    }   else   {  findBiggest(lines,templine);  }


    // std::cout << "=================================================" << std::endl;
}

void BordersDetection::findBiggest(std::vector<cv::Vec4i> lines,std::vector<cv::Point2d> &templine)
{   
    float maxLength=0.0;
    float templength=0.0;

    cv::Point2d pointone;
    pointone.x = 0.0;
    pointone.y = 0.0;
    cv::Point2d pointtwo;
    pointtwo.x = 0.0;
    pointtwo.y = 0.0;

    cv::Point2d lowest_point_one;
    cv::Point2d lowest_point_two;

    for( size_t i = 0; i < lines.size(); i++ )  {   
        pointone.x=lines[i][0];
        pointone.y=lines[i][1];
        pointtwo.x=lines[i][2];
        pointtwo.y=lines[i][3];

        templength=findLength(pointone,pointtwo);

        // std::cout << "LENGTH   " << templength << std::endl;

        if (maxLength<templength)  {
            maxLength=templength;
            // biggest_one = lines[i];
        
            templine.clear();
            templine.push_back(pointone);
            templine.push_back(pointtwo);
        }
    }
    // templine.push_back(lowest_point_one);
    // templine.push_back(lowest_point_two);
}

void BordersDetection::printColors()
{
    // std::cout << color_hsv_values_[0][0] << std::endl
    //         << color_hsv_values_[1][0] << std::endl
    //         << color_hsv_values_[2][0] << std::endl;

    std::cout << "color values :" << std::endl;
    for (int i=0; i<4; i++)
           std::cout << "Line:" << i << " -color:" << color_hsv_values_[i][0] << " -values: " << color_hsv_values_[i][1] << " | " << color_hsv_values_[i][2] << " | " << color_hsv_values_[i][3] << " | " << color_hsv_values_[i][4] << " | " << color_hsv_values_[i][5] << " | " << color_hsv_values_[i][6] << " | " << std::endl;

}

cv::String BordersDetection::getStringColorName(BorderColor whatcolor)
{
    cv::String colorname;
    switch (whatcolor)  {
        case YELLOW:
            return "YELLOW";  break;
        case ORANGE:
            return "ORANGE";  break;
        case RED:
            return "RED";  break;
        case WALL:
            return "WALL";  break;
        default:
            return "NOCOLOR";
    }
}

// void BordersDetection::drawLine(cv::Mat &inputimage, const vector<Point2D>& line,  cv::Scalar color)
// {   

//         Point2D p, q;
//         Point2D p1 = line[0];
//         Point2D p2 = line[1];
//         // Check if the line is a vertical line because vertical lines don't have slope
//         if (p1.x != p2.x)
//         {
//                 p.x = 0;
//                 q.x = inputimage.cols;
//                 // Slope equation (y1 - y2) / (x1 - x2)
//                 float m = (p1.y - p2.y) / (p1.x - p2.x);
//                 // Line equation:  y = mx + b
//                 float b = p1.y - (m * p1.x);
//                 p.y = m * p.x + b;
//                 q.y = m * q.x + b;
//         }
//         else
//         {
//                 p.x = q.x = p2.x;
//                 p.y = 0;
//                 q.y = inputimage.rows;
//         }

//         cv::line(inputimage, p, q, color, 2);
// }
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> 

bool cmd_output = true;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nl;
    image_transport::ImageTransport it(nh);
    ros::Rate loop_rate(50);
    
    cv::String topic_to_publish = "camera/image_raw";       // Default topic to publish image
    int camera_device = 0;      // Default camera to use

    // Launch parameters (if any)
    if( nl.getParam("camera_publisher/topic", topic_to_publish)) 
         ROS_INFO("Topic to publish (selected from launch file): %s ", topic_to_publish.c_str());
    if( nl.getParam("camera_publisher/camera", camera_device)) 
         ROS_INFO("Camera selected (selected from launch file): %d ", camera_device);

    if ( cmd_output ) 
        std::cout << "=============================================================" << std::endl
                << "Topic : \"" << topic_to_publish << "\" " << std::endl
                << "Camera : " << camera_device << std::endl
                << "=============================================================" << std::endl
                << std::endl;

    // image_transport::Publisher pub = it.advertise(topic_to_publish, 1);


    // if(argv[1] == NULL) { 
    // cv::VideoCapture cap(0);
    cv::VideoCapture cap(camera_device);

    if ( !cap.isOpened() )  ROS_ERROR("Video source can't be accessed.");  
    cv::Mat camera_frame;
    sensor_msgs::ImagePtr msg;

    // cv::namedWindow("camera_input",1);
    if ( ros::ok() )  {
        // cap.read(camera_frame);
        cap >> camera_frame;
        if ( !camera_frame.empty() )  {
            std::cout << "not empty" << std::endl;
            cv::namedWindow("camera_input",1);
            cv::imshow("camera_input", camera_frame);
        }  else  {
            std::cout << "empty!" << std::endl;
            cv::destroyWindow("camera_input");
        }


    }
    loop_rate.sleep();
    ros::spin();

    // VideoCapture stream1(0);   //0 is the id of video device.0 if you have only one camera.
 
    // if (!stream1.isOpened()) { //check if video device has been initialised
    // cout << "cannot open camera";
    // }

    // //unconditional loop
    // while ( ros::ok() ) {
    // Mat cameraFrame;
    // stream1.read(cameraFrame);
    // imshow("cam", cameraFrame);
    // // // if (waitKey(30) >= 0)
    // // break;
    // }
    // return 0;



    // // Check if video device can be opened with the given index
    // if(!cap.isOpened()) { std::cout << "Video source cant be accessed." << std::endl; return 1;  }
    // cv::Mat frame;

    // sensor_msgs::ImagePtr msg;
    // std::cout << "[ Ctr-C to terminate ] " << std::endl;
    // ros::Rate loop_rate(50);
    // while (nh.ok()) {
    // cap >> frame;

    // // Check if grabbed frame is actually full with some content
    // if(!frame.empty()) {

    // msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    // pub.publish(msg);
    // }

    // ros::spinOnce();
    // loop_rate.sleep();
    // } 
    // } else  {
    // std::cout << "Video device [" << argv[1] << "] selected." << std::endl;

    // // Convert the passed as command line parameter index for the video device to an integer
    // std::istringstream video_sourceCmd(argv[1]);
    // int video_source;

    // // Check if it is indeed a number
    // if(!(video_sourceCmd >> video_source)) return 1;
    // cv::VideoCapture cap(video_source);

    // // Check if video device can be opened with the given index
    // if(!cap.isOpened()) { std::cout << "Video source cant be accessed." << std::endl; return 1;  }
    // cv::Mat frame;

    // sensor_msgs::ImagePtr msg;
    // std::cout << "[ Ctr-C to terminate ] " << std::endl;
    // ros::Rate loop_rate(50);
    // while (nh.ok()) {
    // cap >> frame;

    // // Check if grabbed frame is actually full with some content
    // if(!frame.empty()) {
    // msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    // pub.publish(msg);

    // }

    // ros::spin();
    // loop_rate.sleep();
    // }
    // }
}
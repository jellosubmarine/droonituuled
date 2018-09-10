#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/opencv.hpp"

using namespace cv;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualizer");
    ros::NodeHandle n;
    image_transport::ImageTransport it_(n);

    image_transport::Publisher image_pub_ = it_.advertise("/frames_output", 1);

    ros::Rate loop_rate(10);
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat edges;
    namedWindow("edges",1);

    while(ros::ok()){
        Mat frame;
        cap >> frame; // get a new frame from camera
        cvtColor(frame, edges, CV_BGR2GRAY);
        GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        Canny(edges, edges, 0, 30, 3);
        imshow("edges", edges);
        if(waitKey(30) >= 0) break;

        #if 1
            ros::Time time = ros::Time::now();
            cv_ptr->encoding = "bgr8";
            cv_ptr->header.stamp = time;
            cv_ptr->header.frame_id = "/frames_output";
        #endif

            cv_ptr->image = frame;
            image_pub_.publish(cv_ptr->toImageMsg());

            ROS_INFO("ImageMsg Sent.");
            ROS_INFO("Subscribers: %d", image_pub_.getNumSubscribers());


        ros::spinOnce();
        loop_rate.sleep();
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    ros::spin();
    return 0;
}

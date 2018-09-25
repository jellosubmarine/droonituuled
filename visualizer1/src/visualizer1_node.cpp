#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <time.h>
#include <ctime>
#include <stdio.h>
#include <stdlib.h>
#include "geometry_msgs/Point.h"
#include <numeric>

using namespace std;
using namespace cv;

// Currently published images. Need to change is to it publishes average x,y vector of all countour midpoints.
// This info goes to another node which calculates it into understandable commands for the controllers.

int main(int argc, char **argv)
{   
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // OPEN CAMERA

    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat frame; // RGB PIC

    cap >> frame; // get a new frame from camera to get the height and width of frame.

    // IMPORTANT CHANGEABLE PARAMETERS

    int lower = 1000; // Lower limit for contour size
    int upper = 10000; // Upper limit for contour size

        // Define the point which is the 0,0 for coordinates (under the drone, needs calibrating)

    int X_offset = -(frame.cols/2); // X - 0 gets shifted into the middle of the frame
    int Y_offset = 10; // Y - 0  logically a few pixels from the bottom of the frame, can also be 0 or negative
        // if drone accelerates it can actually see more below it.


    // STUFF FOR ROS 

    ros::init(argc, argv, "visualizer");
    ros::NodeHandle n;

    ros::Publisher point_pub = n.advertise<geometry_msgs::Point>("/cam_point_vector", 1);
    geometry_msgs::Point msg;
    ros::Rate loop_rate(10);
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);


    // MOST OF NEEDED DEFINITIONS HERE

    Mat img_gray; // GRAY PIC
    Mat img_bw; // B&W PIC
    Mat img_final; // TRESHED PIC
    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    RNG rng(12345);
    float sum_x, sum_y, X, Y;

    namedWindow( "Contours", WINDOW_AUTOSIZE );
    
    // ROS CYCLES START HERE

    while(ros::ok()){

    // START CLOCK TO GET FPS

        std::clock_t start;
        double duration;
        start = std::clock();

    // GET A NEW FRAME

        cap >> frame; // get a new frame from camera

    // IMAGE PROCESSING HERE

        cvtColor(frame, img_gray, CV_BGR2GRAY); // Turn image to grayscale
        blur( img_gray, img_gray, Size( 5, 5 ), Point(-1,-1) ); // Blur image
        threshold(img_gray, img_bw, 0, 255,THRESH_OTSU); // B&W filter
        
        Canny(img_bw, canny_output, 100, 100, 3); // Find edges
        dilate(canny_output, canny_output, Mat(), Point(-1, -1), 2, 1, 1); // Attempt to get rid of noise
        erode(canny_output, canny_output, Mat(), Point(-1, -1), 2, 1, 1);
        findContours(canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) ); // Get contours

    // GETTING CONTOUR CENTROIDS HERE

        vector<Moments> mu; // Moments for calculating centroids
        mu.reserve(contours.size());

        for( size_t i = 0; i < contours.size(); i++ ) // Look through all contours
        { 
        if(contourArea(contours[i]) > lower and contourArea(contours[i]) < upper){ // Contour size treshold
        mu.push_back(moments( contours[i], false )); }} // Calculate moments
        
        vector<Point2f> mc( mu.size() );
        vector<float> avg_x( mu.size());
        vector<float> avg_y( mu.size());

        for( size_t i = 0; i < mu.size(); i++ ) // Based on moments calculate centroids
        { 
        avg_x[i] = float(mu[i].m10/mu[i].m00);
        avg_y[i] = float(mu[i].m01/mu[i].m00);
        mc[i] = Point2f( static_cast<float>(mu[i].m10/mu[i].m00) , static_cast<float>(mu[i].m01/mu[i].m00) ); // for visuals
        }

    // CALCULATING MIDPOINT OF ALL CENTROIDS HERE

        sum_x = accumulate(avg_x.begin(), avg_x.end(), 0); // Sum all centroids x values
        if(avg_x.size() == 0){ X = -1;}
        else {X = (sum_x/avg_x.size());} // Get average x coordinate.

        sum_y = accumulate(avg_y.begin(), avg_y.end(), 0); // Sum all centroids y values
        if(avg_y.size() == 0){ Y = -1;}
        else {Y = (sum_y/avg_y.size());} // Get average y coordinate.
        
        vector<Point2f> XY(1);
        XY[0] = Point2f( static_cast<float>(X) ,static_cast<float>(Y)); // For visuals

        cout << XY[0];

    // VISUALIZING EVERYTHING HERE

        for( size_t i = 0; i< mc.size(); i++ ) // Visualize the centroids
            {
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            circle( canny_output, mc[i], 4, color, -1, 8, 0 );
            circle( canny_output, XY[0], 10, color, -1,8,0);
            }
        
        imshow( "Contours", canny_output );

        if(waitKey(30) >= 0) break;

    // PUBLISHING STUFF HERE
    
        msg.x = X+X_offset;
        msg.y = Y+Y_offset;
        msg.z = 0;

        point_pub.publish(msg);; // Publish point

        ROS_INFO("Msg Sent.");
        ROS_INFO("Subscribers: %d", point_pub.getNumSubscribers());

    // STOP CLOCK HERE, CALCULATE FPS

        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC; // Get FPS here
        std::cout<<"printf: "<< 1/duration <<'\n';
        ros::spinOnce();
        loop_rate.sleep();
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    ros::spin();
    return 0;
}
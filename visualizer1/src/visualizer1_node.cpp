#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <ctime>
#include <iostream>
#include <numeric>
#include "geometry_msgs/PointStamped.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

class visuals
{
  protected:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    geometry_msgs::PointStamped msg;
    image_transport::Subscriber sub;
    ros::Publisher point_pub;

  private:

    // MOST OF NEEDED DEFINITIONS HERE

    bool image_flag;

    // IMPORTANT CHENAGEABLE PARAMETERS HERE
    const static int width = 640;    // Frame width
    const static int height = 480;   // Frame height
    const static int lower  = 1000;  // Lower limit for contour size
    const static int upper = 10000;  // Upper limit for contour size
    const static int X_offset = -(width / 2); // X gets shifted into the middle of the frame .
    const static int Y_offset = 10; // Y shift from the bottom of the frame.

    Mat frame;
    Mat img_gray;
    Mat img_bw; 
    Mat img_final; 
    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    float sum_x, sum_y, X, Y;

  public:

    visuals(ros::NodeHandle &nh_, ros::NodeHandle &private_) : it_(nh_)
    {
        image_flag = false;
        sub = it_.subscribe("/camera/color/image_raw", 1, &visuals::imgcb, this);
        point_pub = nh_.advertise<geometry_msgs::PointStamped>("/cam_point_vector", 1);
    }

    ~visuals() { destroyWindow("Contours"); }

    void imgcb(const sensor_msgs::ImageConstPtr &msg)
    {
        frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        image_flag = true;
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////

    // MAIN PROCESS STARTS HERE

    void process()
    {
        // START CLOCK TO GET FPS

        std::clock_t start;
        double duration;
        start = std::clock();

        // IMAGE PROCESSING HERE

        cvtColor(frame, img_gray, CV_BGR2GRAY);              // Turn image to grayscale
        blur(img_gray, img_gray, Size(5, 5), Point(-1, -1)); // Blur image
        threshold(img_gray, img_bw, 0, 255, THRESH_OTSU);    // B&W filter

        Canny(img_bw, canny_output, 100, 100, 3); // Find edges
        dilate(canny_output, canny_output, Mat(), Point(-1, -1), 2, 1,1); // Attempt to get rid of noise
        erode(canny_output, canny_output, Mat(), Point(-1, -1), 2, 1, 1);
        findContours(canny_output, contours, hierarchy, RETR_TREE,CHAIN_APPROX_SIMPLE, Point(0, 0)); // Get contours

        // GETTING CONTOUR CENTROIDS HERE

        vector<Moments> mu; // Moments for calculating centroids
        mu.reserve(contours.size());

        for (size_t i = 0; i < contours.size(); i++) // Look through all contours
        {   

            if (contourArea(contours[i]) > lower and contourArea(contours[i]) < upper)
            {

                mu.push_back(moments(contours[i], false));
            }

        } // Calculate moments

        vector<Point2f> mc(mu.size());
        vector<float> avg_x(mu.size());
        vector<float> avg_y(mu.size());
        
        for (size_t i = 0; i < mu.size(); i++) // Based on moments calculate centroids
            {
                avg_x[i] = float(mu[i].m10 / mu[i].m00);
                avg_y[i] = float(mu[i].m01 / mu[i].m00);
                mc[i] =
                    Point2f(static_cast<float>(mu[i].m10 / mu[i].m00),
                            static_cast<float>(mu[i].m01 / mu[i].m00)); // for visuals
            }

        // CALCULATING MIDPOINT OF ALL CENTROIDS HERE

        sum_x = accumulate(avg_x.begin(), avg_x.end(), 0); // Sum all centroids x values

        if (avg_x.size() == 0)
        {
            X = -1;
        }
        else
        {
            X = (sum_x / avg_x.size()); // Get average x coordinate.
        }

        sum_y = accumulate(avg_y.begin(), avg_y.end(), 0); // Sum all centroids y values

        if (avg_y.size() == 0)
        {
            Y = -1;
        }
        else
        {
            Y = (sum_y / avg_y.size()); // Get average y coordinate.
        }

        vector<Point2f> XY(1);
        XY[0] =
            Point2f(static_cast<float>(X), static_cast<float>(Y)); // For visuals

        cout << XY[0];
        cout << mc.size();
        // VISUALIZING EVERYTHING HERE

        for (size_t i = 0; i < mc.size(); i++) // Visualize the centroids
        {
            circle(canny_output, mc[i], 4, Scalar(255,5,255), -1, 8, 0);
            circle(canny_output, XY[0], 10, Scalar(255, 5, 5), -1, 8, 0);
        }

        imshow("Contours", canny_output);
        waitKey(1);

        /*  if (waitKey(30) >= 0)
    break; */

        // PUBLISHING STUFF HERE

        if( X >= 0 and Y >= 0) {
            X += X_offset;
            Y = (height - Y) + Y_offset; // (height-Y) makes the Y
            // coordinate start from the bottom.
        }

        msg.header.frame_id = "frames";
        msg.header.stamp = ros::Time::now();

        msg.point.x = X;
        msg.point.y = Y;
        msg.point.z = 0;

        point_pub.publish(msg);
        // Publish point

        ROS_INFO("Msg Sent.");
        ROS_INFO("Subscribers: %d", point_pub.getNumSubscribers());

        // STOP CLOCK HERE, CALCULATE FPS

        duration = (std::clock() - start) / (double)CLOCKS_PER_SEC; // Get FPS here
        cout << "printf: " << 1 / duration << '\n';
    }

    virtual void spin()
    {
        ros::Rate rate(30);
        while (ros::ok())
        {
            if (image_flag)
            {
                process();
                image_flag = false;
            }

            ros::spinOnce();
            rate.sleep();
        }
    }
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualizer");

    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    visuals *node = 0;

    namedWindow("Contours", WINDOW_AUTOSIZE);
    startWindowThread();

    node = new visuals(nh, nhPrivate);

    node->spin();
    return 0;
}

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
#include "geometry_msgs/QuaternionStamped.h"
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
    geometry_msgs::QuaternionStamped msg;
    image_transport::Subscriber sub;
    ros::Publisher point_pub;

  private:

    // MOST OF NEEDED DEFINITIONS HERE

    bool image_flag;

    // IMPORTANT CHENAGEABLE PARAMETERS HERE

    const static bool visualize = true; // True for visuals, false to turn visualizing off

    const static int width = 640;    // Frame width
    const static int height = 480;   // Frame height
    const static int lower  = 1000;  // Lower limit for contour size
    const static int upper = 10000;  // Upper limit for contour size
    const static int X_offset = -(width / 2); // X gets shifted into the middle of the frame .
    const static int Y_offset = 0; // Y shift from the bottom of the frame.
    const static int blur_size = 3; // Blurring size
    const static int lowThreshold = 121; // Canny lower threshold
    const static int ratio = 1.5;  // Canny lower Tresh * ratio = upper thresh
    const static int kernel_size = 3; // Canny kernel size
    const static int mask_w = 150; // Mask cutoff from sides

    Mat frame, img_bw, img_gray, img_final, erod, dil, masked, blurr, canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    float sum_x, sum_y, X, Y, Z, bot_X, bot_Y, plot_x, plot_y;
    int W;

  public:

    visuals(ros::NodeHandle &nh_, ros::NodeHandle &private_) : it_(nh_)
    {
        image_flag = false;
        sub = it_.subscribe("/camera/color/image_raw", 1, &visuals::imgcb, this);
        point_pub = nh_.advertise<geometry_msgs::QuaternionStamped>("/cam_point_vector", 1);
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
        std::clock_t start;
        double duration;
        if (visualize)
            start = std::clock();


        // START CLOCK TO GET FPS



        // IMAGE PROCESSING HERE

        cvtColor(frame, img_gray, CV_BGR2GRAY);              // Turn image to grayscale

        blur(frame, blurr, Size(blur_size, blur_size), Point(-1, -1)); // Blur image

        Canny(blurr, dil, lowThreshold, lowThreshold*ratio, kernel_size ); // Find edges
        dilate(dil, erod, Mat(), Point(-1, -1), 3, 1,1); // Attempt to get rid of noise
        //erode(dil, dil, Mat(), Point(-1, -1), 4, 1, 1);

        Mat mask = cv::Mat::zeros(erod.size(), erod.type()); // all 0
        mask(Rect(mask_w, 0,mask.cols-2*mask_w,mask.rows)) = 1;
        erod.copyTo(canny_output, mask);

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
        float weights;
        weights = 0;
        for (size_t i = 0; i < mu.size(); i++) // Based on moments calculate centroids
            {
                avg_x[i] = float((mu[i].m10 / mu[i].m00))/((width/2)-(mu[i].m10 / mu[i].m00));
                avg_y[i] = float(mu[i].m01 / mu[i].m00);
                weights += 1/((width/2)-(mu[i].m10 / mu[i].m00));

                if (visualize) {
                mc[i] =
                    Point2f(static_cast<float>(mu[i].m10 / mu[i].m00),
                            static_cast<float>(mu[i].m01 / mu[i].m00)); // for visuals
                }

                if (i==0){
                    bot_X = static_cast<float>(mu[i].m10 / mu[i].m00);
                    bot_Y = static_cast<float>(mu[i].m01 / mu[i].m00);
                }
            }

        // CALCULATING MIDPOINT OF ALL CENTROIDS HERE

        sum_x = accumulate(avg_x.begin(), avg_x.end(), 0); // Sum all centroids x values

        if (avg_x.size() == 0)
        {
            X = -1;
            Z = -1;
            W = 0;
            bot_X = -1;
            bot_Y = -1;
            
            if (visualize){
                ROS_INFO("No contours detected.");
            }
           
        }
        else
        {
            X = (sum_x / weights); // Get average x coordinate.
            plot_x = X;
        }

        sum_y = accumulate(avg_y.begin(), avg_y.end(), 0); // Sum all centroids y values

        if (avg_y.size() == 0)
        {
            Y = -1;
        }
        else
        {
            Y = (sum_y / avg_y.size()); // Get average y coordinate.
            plot_y = Y;
        }

        // PUBLISHING STUFF HERE

        if( X >= 0 and Y >= 0) {
            X += X_offset;
            Y = (height - Y) + Y_offset; // (height-Y) makes the Y
            // coordinate start from the bottom.
            bot_X += X_offset;
            bot_Y = (height - bot_Y) + Y_offset;
            Z = atan2(X-bot_X,Y-bot_Y);
            W = 1;
        }

        msg.header.frame_id = "frames";
        msg.header.stamp = ros::Time::now();

        msg.quaternion.x = bot_X;
        msg.quaternion.y = bot_Y;
        msg.quaternion.z = Z;
        msg.quaternion.w = W;

        point_pub.publish(msg); // Publish point
        

        // VISUALIZING EVERYTHING HERE

        Point2f XY;
        XY.x = static_cast<float>(plot_x);
        XY.y = static_cast<float>(plot_y); // For visuals

        if (visualize){

            for (size_t i = 0; i < mc.size(); i++) // Visualize the centroids
            {
                circle(canny_output, mc[i], 4, Scalar(100,100,100), -1, 8, 0);
                circle(canny_output, XY, 10, Scalar(150,150,150), -1, 8, 0);
                circle(canny_output, mc[0], 8, Scalar(127,127,127), -1, 8, 0);
                line(canny_output,mc[0],XY,Scalar(75,75,75),5);
            }

            imshow("Contours", canny_output);
            waitKey(1);

            if (W==1){
                ROS_INFO("Msg Sent.");
                ROS_INFO("Subscribers: %d", point_pub.getNumSubscribers());
            }

            // STOP CLOCK HERE, CALCULATE FPS
            duration = (std::clock() - start) / (double)CLOCKS_PER_SEC; // Get FPS here
            cout << "printf: " << 1 / duration << '\n';
        }
       
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

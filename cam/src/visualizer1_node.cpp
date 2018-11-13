#include "dt_config.hpp"

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


#ifdef DT_BUILD_DEV
  #pragma message "robotex_cam dev build"
#else
  #pragma message "robotex_cam live build"
#endif


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

    //const static bool visualize = false; // True for visuals, false to turn visualizing off

    const static int width = 640;    // Frame width
    const static int height = 480;   // Frame height
    const static int lower  = 1000;  // Lower limit for contour size
    const static int upper = 10000;  // Upper limit for contour size
    const static int X_offset = -(width / 2); // X gets shifted into the middle of the frame .
    const static int Y_offset = -(height / 2); // Y shift from the bottom of the frame.
    const static int blur_size = 3; // Blurring size
    const static int lowThreshold = 121; // Canny lower threshold
    const static int ratio = 1.5;  // Canny lower Tresh * ratio = upper thresh
    const static int kernel_size = 3; // Canny kernel size
    const static int mask_w = 200; // Mask cutoff from sides

    Mat frame;
    Mat img_gray;
    Mat img_bw;
    Mat erod;
    Mat dil;
    Mat masked;
    Mat blurr;
    Mat img_final;
    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    float sum_x, sum_y, X, Y, bot_X, bot_Y, Z;
    int W;

  public:

    visuals(ros::NodeHandle &nh_, ros::NodeHandle &private_) : it_(nh_)
    {
        image_flag = false;
        sub = it_.subscribe("/camera/color/image_raw", 1, &visuals::imgcb, this);
        point_pub = nh_.advertise<geometry_msgs::QuaternionStamped>(DT_CAM_TOPIC, 1);
    }

    #ifdef DT_BUILD_DEV
      ~visuals() { destroyWindow("Contours"); }
    #endif

    void imgcb(const sensor_msgs::ImageConstPtr &msg)
    {
        frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        image_flag = true;
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////

    // MAIN PROCESS STARTS HERE

    void process()
    {
        #ifdef DT_BUILD_DEV
          // Clock for frame rate calculation
          std::clock_t start( std::clock() );
          double duration;
        #endif

        // IMAGE PROCESSING HERE
        cvtColor(frame, img_gray, CV_BGR2GRAY);              // Turn image to grayscale

        blur(frame, blurr, Size(blur_size, blur_size), Point(-1, -1)); // Blur image

        Canny(blurr, dil, lowThreshold, lowThreshold*ratio, kernel_size ); // Find edges
        dilate(dil, erod, Mat(), Point(-1, -1), 4, 1,1); // Attempt to get rid of noise
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

        for (size_t i = 0; i < mu.size(); i++) // Based on moments calculate centroids
            {
                avg_x[i] = float(mu[i].m10 / mu[i].m00);
                avg_y[i] = float(mu[i].m01 / mu[i].m00);

                #ifdef DT_BUILD_DEV
                  mc[i] =
                      Point2f(static_cast<float>(mu[i].m10 / mu[i].m00),
                              static_cast<float>(mu[i].m01 / mu[i].m00)); // for visuals
                #endif

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
            W = 0;
            Z = -1;

            #ifdef DT_BUILD_DEV
              ROS_INFO("No contours detected.");
            #endif
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




        // VISUALIZING EVERYTHING HERE
        #ifdef DT_BUILD_DEV
            canny_output = canny_output * 0.7;

            for (size_t i = 1; i < mc.size(); i++) // Visualize the centroids
            {
                circle(canny_output, mc[i], 4, Scalar(100,100,100), 2, 8, 0);
            }

            if (mc.size() > 0) {
              circle(canny_output, XY[0], 10, Scalar(180,180,180), -1, 8, 0);
              circle(canny_output, mc[0], 8, Scalar(140,140,140), -1, 8, 0);
            }

            imshow("Contours", canny_output);
            waitKey(1);
        #endif



        // PUBLISHING STUFF HERE

        if( X >= 0 and Y >= 0) {
            X += X_offset;
            Y = (height - Y) + Y_offset; // (height-Y) makes the Y
            // coordinate start from the bottom.
            bot_X += X_offset;
            bot_Y = (height - bot_Y) + Y_offset;
            Z = atan2(X-bot_X,Y-bot_Y);
            W = avg_x.size(); //1;
        }

        msg.header.frame_id = "frames";
        msg.header.stamp = ros::Time::now();

        msg.quaternion.x = X / double(width) * 2.0;
        msg.quaternion.y = Y / double(height) * 2.0;
        msg.quaternion.z = Z;
        msg.quaternion.w = W;

        point_pub.publish(msg);
        // Publish point

        /*
        if (visualize){
            if (W==1){
                ROS_INFO("Msg Sent.");
                ROS_INFO("Subscribers: %d", point_pub.getNumSubscribers());
            }

            // STOP CLOCK HERE, CALCULATE FPS
            duration = (std::clock() - start) / (double)CLOCKS_PER_SEC; // Get FPS here
            cout << "printf: " << 1 / duration << '\n';
        }
        */
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

    #ifdef DT_BUILD_DEV
      namedWindow("Contours", WINDOW_AUTOSIZE);
      startWindowThread();
    #endif

    node = new visuals(nh, nhPrivate);

    node->spin();
    return 0;
}
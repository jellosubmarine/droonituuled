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

    Mat frame, img_bw, img_gray, img_final, erod, dil, masked, blurr, canny_output, data_pts;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    vector<Point2d> eigen_vecs;
    float X, Y, Z;
    int W, sz, length, angle;
    double duration;
    Point cntr, P1, P2;
    Point2d XY;
    

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

        // START CLOCK TO GET FPS
        std::clock_t start;
        if (visualize)
            start = std::clock();

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

        // Moments for calculating centroids
        vector<Moments> mu;
        mu.reserve(contours.size());

        for (size_t i = 0; i < contours.size(); i++) // Look through all contours
        {

            if (contourArea(contours[i]) > lower and contourArea(contours[i]) < upper)
                mu.push_back(moments(contours[i], false));

        } // Calculate moments
        vector<Point2f> mc(mu.size());
        cout << mu.size();
        for (size_t i = 0; i < mu.size(); i++) // Based on moments calculate centroids
            {
                mc[i] =
                    Point2f(static_cast<float>(mu[i].m10 / mu[i].m00),
                            static_cast<float>(mu[i].m01 / mu[i].m00));
                }


        //Construct a buffer used by the pca analysis
        sz = static_cast<int>(mc.size());
        data_pts = Mat(sz, 2, CV_64FC1);

        for (int i = 0; i < data_pts.rows; ++i)
        {
            data_pts.at<double>(i, 0) = mc[i].x;
            data_pts.at<double>(i, 1) = mc[i].y;
        }
        if(sz == 0) {
            X = -1;
            Y = -1;
            Z = -1;
            W = 0;
        }

        else
        { 
            //Perform PCA analysis
            PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
            //Store the center of the object

            cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                            static_cast<int>(pca_analysis.mean.at<double>(0, 1)));

            X = cntr.x;
            Y = cntr.y;

            //Store the eigenvectors
            eigen_vecs.reserve(2);
            vector<double> eigen_val(2);
            for (int i = 0; i < 2; ++i)
            {
                eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                        pca_analysis.eigenvectors.at<double>(i, 1));

                eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
            }


        // PUBLISHING STUFF HERE

            X += X_offset;
            Y = (height - Y) + Y_offset; // (height-Y) makes the Y coordinate start from the bottom.
            Z = atan2(eigen_vecs[0].y,eigen_vecs[0].x);
            W = 1;
        }

        msg.header.frame_id = "frames";
        msg.header.stamp = ros::Time::now();

        msg.quaternion.x = X;
        msg.quaternion.y = Y;
        msg.quaternion.z = Z;
        msg.quaternion.w = W;

        point_pub.publish(msg); // Publish point
        

        // VISUALIZING EVERYTHING HERE

        if (visualize){

            angle = Z;
            length = 150;
            P1.x = cntr.x;
            P1.y = cntr.y;
            P2.x =  (int)round(P1.x + length * cos(angle));
            P2.y =  (int)round(P1.y + length * sin(angle));

            for (size_t i = 0; i < data_pts.rows; i++) // Visualize the centroids
            {
                XY.x = data_pts.at<double>(i, 0);
                XY.y = data_pts.at<double>(i, 1);
                circle(canny_output, XY, 4, Scalar(100,100,100), -1, 8, 0);
                circle(canny_output, P1, 10, Scalar(150,150,150), -1, 8, 0);
                line(canny_output,P1,P2,Scalar(75,75,75),5);
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

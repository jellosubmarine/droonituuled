
#include "opencv2/opencv.hpp"
#include <iostream>
using namespace cv;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";
int main(int argc, char* argv[])
{
    VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);
    namedWindow(window_capture_name);
    namedWindow(window_detection_name);
    Mat frame, frame_HSV, frame_threshold;
    while (true) {
        cap >> frame;
        if(frame.empty())
        {
            break;
        }
        // Convert from BGR to HSV colorspace
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        // Detect the object based on HSV Range Values
        inRange(frame_HSV, Scalar(0,0, 0), Scalar(180, 10, 255), frame_threshold);
        // Show the frames
        imshow(window_capture_name, frame);
        imshow(window_detection_name, frame_threshold);
        char key = (char) waitKey(30);
        if (key == 'q' || key == 27)
        {
            break;
        }
    }
    return 0;
}
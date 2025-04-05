#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
 
#include <iostream>
 
using namespace cv;
 
int main(int, char**)
{
    Mat image;
    VideoCapture cap(0);

    if (!cap.isOpened()) {

        std::cout << "cannot open camera" << std::endl;
        return 1;
    }

    cap >> image;
 
    //imshow("Display window", img);
    //int k = waitKey(0); // Wait for a keystroke in the window
 
    //if(k == 's')
    //{
        imwrite("/home/ros/starry_night.png", image);
    //}
 
    return 0;
}
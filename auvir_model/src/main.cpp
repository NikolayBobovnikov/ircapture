#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

//#include "testbed.hpp"

int main()
{
    std::cout << "Hello AuViR!" << std::endl;
    cv::Mat matrix;
    cv::VideoCapture cap;
    cap.open(0);
    if(cap.isOpened())
    {
        cap >> matrix;
    }
    //cv::imshow("image", matrix);
    std::cout << "Image captured" << std::endl;
    bool res = cv::imwrite("testimage.jpg", matrix);
    std::string s = "/srv/ftp/auvir/auvir_model/images/test/";

    if(!res)
    {
        std::cout << "there was an error while saving image"<< std::endl;
    }

    //test_function();


    return -1;
}


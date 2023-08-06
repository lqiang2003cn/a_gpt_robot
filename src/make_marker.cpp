#include <iostream>
#include <aruco/marker.h>
#include <opencv2/opencv.hpp>

int main()
{
    aruco::Marker marker = aruco::Marker(123);
    cv::Mat markerImage;
    marker.draw(markerImage, NULL, 10, true , true);
    cv::imwrite("test4.jpg", markerImage);
    std::cout << "running";
}
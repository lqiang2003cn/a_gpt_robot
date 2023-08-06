#include <iostream>
#include <aruco/marker.h>

int main(int argc, char **argv)
{
    aruco::Marker marker = aruco::Marker(123);
    cv::Mat markerImage;
    marker.draw(markerImage, NULL, 10, true , true);
    std::cout << "running";
}
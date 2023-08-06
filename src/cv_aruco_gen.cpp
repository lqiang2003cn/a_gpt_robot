#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{
    cv::Mat markerImage;
    // Load the predefined dictionary
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    // Generate the marker
    cv::aruco::drawMarker(dictionary, 33, 200, markerImage, 1);
    cv::imwrite("test.jpg", markerImage);
}

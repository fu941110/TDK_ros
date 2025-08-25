#include "camera/cameraBackground.hpp"
#include <iostream>

using namespace cv;
using namespace std;

/*Camera port

    usb-0000:00:14.0-?
    ?: my computer: 1
                    2               3

    /dev/video6 (desk),  /dev/video12(coffee)




*/
Camera::Camera(bool init) 
: Node("cameraBackground")
{
    if (init) {
        // TrackerBar();
        // RCLCPP_INFO(this->get_logger(), "Camera node initialized with trackbars.");
    }
}

void Camera::TrackerBar() {
    namedWindow("tracker");
    resizeWindow("tracker", 300, 300);

    createTrackbar("Hue Min", "tracker", &hue_min_, 179);
    createTrackbar("Hue Max", "tracker", &hue_max_, 179);
    createTrackbar("Sat Min", "tracker", &sat_min_, 255);
    createTrackbar("Sat Max", "tracker", &sat_max_, 255);
    createTrackbar("Val Min", "tracker", &val_min_, 255);
    createTrackbar("Val Max", "tracker", &val_max_, 255);

    updateTrackbarValues();
}     

void Camera::updateTrackbarValues() {
    hue_min_ = getTrackbarPos("Hue Min", "tracker");
    hue_max_ = getTrackbarPos("Hue Max", "tracker");
    sat_min_ = getTrackbarPos("Sat Min", "tracker");
    sat_max_ = getTrackbarPos("Sat Max", "tracker");
    val_min_ = getTrackbarPos("Val Min", "tracker");
    val_max_ = getTrackbarPos("Val Max", "tracker");
}

void Camera::setTrackbarPosition(int hue_min, int hue_max, int sat_min, int sat_max, int val_min, int val_max)
{
    setTrackbarPos("Hue Min", "tracker", hue_min);
    setTrackbarPos("Hue Max", "tracker", hue_max);
    setTrackbarPos("Sat Min", "tracker", sat_min);
    setTrackbarPos("Sat Max", "tracker", sat_max);
    setTrackbarPos("Val Min", "tracker", val_min);
    setTrackbarPos("Val Max", "tracker", val_max);
    updateTrackbarValues();
}

cv::Mat Camera::EdgeDetect(Mat img) {
    Mat gray, edge;
    cvtColor(img, gray, COLOR_BGR2GRAY);
    Canny(gray, edge, 80, 150);
    dilate(edge, edge, getStructuringElement(MORPH_RECT, Size(5, 5)));
    morphologyEx(edge, edge, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(5, 5)));
    erode(edge, edge, getStructuringElement(MORPH_RECT, Size(5, 5)));
    adaptiveThreshold(gray, gray, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 15, 10);
    medianBlur(gray, gray, 5);
    return edge;
}
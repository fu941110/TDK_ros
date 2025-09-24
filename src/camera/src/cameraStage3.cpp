#include <iostream>
#include "camera/cameraBackground.hpp"

using namespace cv;
using namespace std;

class CameraStage3{
public:
    CameraStage3() {
        // setTrackbarPosition(orange_min_hue, orange_max_hue, 
        //                     orange_min_sat, orange_max_sat, 
        //                     orange_min_val, orange_max_val); 
    }


    Mat FindOrange(Mat img) {
        Mat hsv, mask, result;
        cvtColor(img, hsv, COLOR_BGR2HSV);
        inRange(hsv, Scalar(orange_min_hue, orange_min_sat, orange_min_val), 
                Scalar(orange_max_hue, orange_max_sat, orange_max_val),mask);
        // inRange(hsv, Scalar(orange_min_hue, orange_min_sat, orange_min_val), 
        //         Scalar(orange_max_hue, orange_max_sat, orange_max_val), mask);
        bitwise_and(img, img, result, mask);
        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        double maxArea = 0;
        vector<Point> bestContour;
        for (const auto& contour : contours) {
            double area = contourArea(contour);
            if (area > maxArea && area > 1500) { 
                maxArea = area;
                bestContour = contour;
            }
        }
        if (!bestContour.empty()) {
            orange_area = static_cast<int>(maxArea);
            // drawContours(result, vector<vector<Point>>{bestContour}, -1, Scalar(255, 0, 0), 2);

            Moments m = moments(bestContour);
            orange_center_x = static_cast<int>(m.m10 / m.m00) - (img.cols / 2);
            orange_center_y = static_cast<int>(m.m01 / m.m00) - (img.rows / 2);
            Scalar PointColor = (abs(orange_center_x) + abs(orange_center_y) > 50) ? Scalar(0, 0, 255) : Scalar(0, 255, 0); 
            circle(result, Point(orange_center_x + img.cols/2, orange_center_y + img.rows/2), 10, PointColor, -1);
            printf("Orange Center: (%d, %d), Area: %d\n", orange_center_x, orange_center_y, orange_area);
        }

        return result;
    }


private:

    int orange_center_x, orange_center_y, orange_area;

    const int orange_min_hue = 12;
    const int orange_max_hue = 24;
    const int orange_min_sat = 125;
    const int orange_max_sat = 255;
    const int orange_min_val = 150;
    const int orange_max_val = 255;

    // bool orange_type = false;
};


int main(int argc, char **argv) {
    VideoCapture cap(6, CAP_V4L2);
    Mat img, edge, orange; 
    CameraStage3 camera3;

    while (true) {
        bool ret = cap.read(img);
        if (!ret) {
            std::cout << "can't receive frame\n";
            break;
        }
        imshow("image", img); //初始

        orange = camera3.FindOrange(img);
        imshow("Orange Detection", orange); // 橘子檢測結果


        if (waitKey(1) == 27) break;
    }
    return 0;
}

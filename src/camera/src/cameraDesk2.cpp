#include <iostream>
#include "camera/cameraBackground.hpp"

using namespace cv;
using namespace std;

class CameraDesk2 : public Camera {
public:
    CameraDesk2() : Camera() {
        
    }

    Mat GetSquarePoint(Mat img){
        // Mat edge = EdgeDetect(img);
        // vector<vector<Point>> contours;
        // findContours(edge, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

        // double maxColor = 0;
        // vector<Point> bestContour;

        // for (const auto& contour : contours) {
        //     // double area = contourArea(contour);

        //     vector<Point> approx;
        //     approxPolyDP(contour, approx, 0.02 * arcLength(contour, true), true);

        //     if (approx.size() == 4 && isContourConvex(approx)) {
        //         Mat mask = Mat::zeros(img.size(), CV_8UC1);
        //         vector<vector<Point>> singleContour = {approx};
        //         drawContours(mask, singleContour, 0, Scalar(255), FILLED);

        //         // 計算平均顏色
        //         Scalar meanColor = mean(img, mask);
        //         double totalColor = meanColor[0] + meanColor[1] + meanColor[2];
        //         if (totalColor > maxColor) {
        //             maxColor = totalColor;
        //             bestContour = approx;
        //             Moments m = moments(approx);
        //             center_x = static_cast<int>(m.m10 / m.m00);
        //             center_y = static_cast<int>(m.m01 / m.m00);
        //         }
        //     }
        // }

        // Mat output = img.clone();
        // if (!bestContour.empty()) {
        //     polylines(output, bestContour, true, Scalar(0, 255, 0), 3);
        // }

        // return output;

        const int rectWidth = 160;
        const int rectHeight = 160;
        double maxBrightness = -1;
        Rect bestRect;

        for (int y = 0; y <= img.rows - rectHeight; y += 20) { 
            for (int x = 0; x <= img.cols - rectWidth; x += 20) {
                Rect roi(x, y, rectWidth, rectHeight);
                Mat region = img(roi);
                Scalar meanColor = mean(region);
                double brightness = meanColor[0] + meanColor[1] + meanColor[2]; // BGR 總和

                if (brightness > maxBrightness) {
                    maxBrightness = brightness;
                    bestRect = roi;
                    center_x = x + (rectWidth / 2) - (img.cols / 2);
                    center_y = y + (rectHeight / 2) - (img.rows / 2);
                    printf("Center: (%d, %d)\n", center_x, center_y);
                }
            }
        }

        Mat output = img.clone();
        rectangle(output, bestRect, Scalar(0, 255, 0), 2);
        return output;
    }

private:

    int center_x, center_y;
};



int main() {
    VideoCapture cap(0, CAP_V4L2);
    Mat img, edge, findout, end; 
    CameraDesk2 camera2;

    while (true) {
        bool ret = cap.read(img);
        if (!ret) {
            std::cout << "can't receive frame\n";
            break;
        }
        imshow("image", img); //初始

        edge = camera2.EdgeDetect(img);
        dilate(edge, edge, getStructuringElement(MORPH_RECT, Size(5, 5)));
        imshow("Edge Detection", edge);  // 邊緣檢測結果

        findout = camera2.GetSquarePoint(img);
        imshow("Square Detection", findout);  // 四方形檢測結果

        if (waitKey(1) == 27) break;
    }
    return 0;
}

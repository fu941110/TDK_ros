#include <iostream>
#include "camera/cameraBackground.hpp"

using namespace cv;
using namespace std;

class CameraStage2 : public Camera {
public:
    CameraStage2() : Camera() {
        
    }

    Mat layerMask(Mat img){
        Mat edge = EdgeDetect(img);
        vector<vector<Point>> contours;
        findContours(edge, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

        double maxArea = 0;
        vector<Point> bestContour;

        for (const auto& contour : contours) {
            double area = contourArea(contour);
            if (area < 30000) continue;  // 過濾太小的雜訊

            vector<Point> approx;
            approxPolyDP(contour, approx, 0.02 * arcLength(contour, true), true);

            if (approx.size() == 4 && isContourConvex(approx)) {
                if (area > maxArea) {
                    maxArea = area;
                    bestContour = approx;
                    Moments m = moments(approx);
                    center_x = static_cast<int>(m.m10 / m.m00);
                    center_y = static_cast<int>(m.m01 / m.m00);
                }
            }
        }

        Mat output = img.clone();
        if (!bestContour.empty()) {
            polylines(output, bestContour, true, Scalar(0, 255, 0), 3);
        }

        Mat masked(img.size(), img.type(), Scalar(255, 255, 255));  // 預設全黑

        if (!bestContour.empty()) {

                polylines(output, bestContour, true, Scalar(0, 255, 0), 3);

                Mat mask = Mat::zeros(img.size(), CV_8UC1);
                vector<vector<Point>> fillContours = { bestContour };
                fillPoly(mask, fillContours, Scalar(255));

                bitwise_and(img, img, masked, mask);
        }

        return masked;
    }

    Mat findFourSquare(Mat masked)
    {
        Mat gray, edge;
        // cvtColor(masked, gray, COLOR_BGR2GRAY);
        // GaussianBlur(gray, gray, Size(3, 3), 0);
        // Canny(gray, edge, 50, 150);
        edge = EdgeDetect(masked);
        // dilate(edge, edge, getStructuringElement(MORPH_RECT, Size(3, 3)));
        // erode(edge, edge, getStructuringElement(MORPH_RECT, Size(3, 3)));
        // dilate(edge, edge, getStructuringElement(MORPH_RECT, Size(5, 5)));
        imshow("E", edge);

        vector<vector<Point>> contours;
        findContours(edge, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

        Mat output = masked.clone();

        for (const auto& contour : contours) {
            double area = contourArea(contour);
            // printf("Area: %f\n", area);
            if (area < 1500 || area > 80000) continue; 

            vector<Point> approx;
            approxPolyDP(contour, approx, 0.02 * arcLength(contour, true), true);

            if (approx.size() == 4 && isContourConvex(approx)) {
                // 計算中心點
                Moments m = moments(approx);
                int cx = static_cast<int>(m.m10 / m.m00);
                int cy = static_cast<int>(m.m01 / m.m00);

                // 檢查中心點的像素值是否為黑色（近似判斷）
                Vec3b pixel = masked.at<Vec3b>(cy, cx);
                bool isBlack = (pixel[0] < 100 || pixel[1] < 100 || pixel[2] < 100); // BGR 都接近 0

                Scalar color = isBlack ? Scalar(255, 0, 0) : Scalar(0, 0, 255); // 藍色 or 紅色

                if(!isBlack) {
                    // 建立 mask，只選出目前這個 contour 的區域
                    Mat mask = Mat::zeros(masked.size(), CV_8UC1);
                    vector<vector<Point>> singleContour = {approx};
                    drawContours(mask, singleContour, 0, Scalar(255), FILLED);

                    // 計算平均顏色
                    Scalar meanColor = mean(masked, mask);
                    double totalColor = meanColor[0] + meanColor[1] + meanColor[2];

                    // 閾值可依需求調整，這裡設定為 120*3 = 360
                    color = (totalColor < 360) ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
                }
                if(color != Scalar(0, 0, 255)) 
                {
                    int dx = cx - center_x;
                    int dy = cy - center_y;
                    if(dx > 0 && dy < 0) {
                        coffee_number = 1; // 假設咖啡杯在右上
                    } else if(dx < 0 && dy < 0) {
                        coffee_number = 2; // 假設咖啡杯在左上
                    } else if(dx < 0 && dy > 0) {
                        coffee_number = 3; // 假設咖啡杯在左下
                    } else if(dx > 0 && dy > 0) {
                        coffee_number = 4; // 假設咖啡杯在右下
                    }
                    
                    if(color == Scalar(0, 255, 0)) {
                        coffee_type = "White";
                    } else {
                        coffee_type = "Black";
                    }

                    printf("Coffee number: %d, Type: %s\n", coffee_number, coffee_type.c_str());
                }
                polylines(output, approx, true, color, 2);
            }
        }

        return output;
    }

private:

    int center_x, center_y;

    int coffee_number = 0;
    std::string coffee_type = "Unknown";
};



// class CameraStage2 {
// public:
//     CameraStage2() {
//         namedWindow("tracker");
//         resizeWindow("tracker", 300, 300);

//         createTrackbar("Hue Min", "tracker", &hue_min_, 179);
//         createTrackbar("Hue Max", "tracker", &hue_max_, 179);
//         createTrackbar("Sat Min", "tracker", &sat_min_, 255);
//         createTrackbar("Sat Max", "tracker", &sat_max_, 255);
//         createTrackbar("Val Min", "tracker", &val_min_, 255);
//         createTrackbar("Val Max", "tracker", &val_max_, 255);

//         setTrackbarPos("Hue Min", "tracker", 70);
//         setTrackbarPos("Hue Max", "tracker", 179);
//         setTrackbarPos("Sat Min", "tracker", 0);
//         setTrackbarPos("Sat Max", "tracker", 110);
//         setTrackbarPos("Val Min", "tracker", 0);
//         setTrackbarPos("Val Max", "tracker", 125);

//         updateTrackbarValues();
//     }
//     Mat EdgeDetect(Mat img) {
//         Mat gray, edge, processed;
        
//         // 1. 轉灰階
//         cvtColor(img, gray, COLOR_BGR2GRAY);

//         // 2. 模糊降雜訊
//         // GaussianBlur(gray, gray, Size(5, 5), 0);
//         // adaptiveThreshold(gray, gray, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 15, 10);
//         // medianBlur(gray, gray, 5);

//         // 3. 邊緣偵測
//         Canny(gray, edge, 80, 150);
//         // threshold(gray, edge, 20, 255, THRESH_BINARY_INV); 

//         // 4. 膨脹邊緣以加強輪廓
//         dilate(edge, edge, getStructuringElement(MORPH_RECT, Size(5, 5)));
//         morphologyEx(edge, edge, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(5, 5)));
//         erode(edge, edge, getStructuringElement(MORPH_RECT, Size(5, 5)));
//         adaptiveThreshold(gray, gray, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 15, 10);
//         medianBlur(gray, gray, 5);
//         // 5. 關閉操作修補邊緣裂縫
//         // morphologyEx(edge, edge, MORPH_CLOSE, getStructuringElement(MORPH_RECT, Size(5, 5)));

//         return edge;
//     }
//     Mat layerMask(Mat img){
//         Mat edge = EdgeDetect(img);
//         vector<vector<Point>> contours;
//         findContours(edge, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

//         double maxArea = 0;
//         vector<Point> bestContour;

//         for (const auto& contour : contours) {
//             double area = contourArea(contour);
//             if (area < 30000) continue;  // 過濾太小的雜訊

//             vector<Point> approx;
//             approxPolyDP(contour, approx, 0.02 * arcLength(contour, true), true);

//             if (approx.size() == 4 && isContourConvex(approx)) {
//                 if (area > maxArea) {
//                     maxArea = area;
//                     bestContour = approx;
//                     Moments m = moments(approx);
//                     center_x = static_cast<int>(m.m10 / m.m00);
//                     center_y = static_cast<int>(m.m01 / m.m00);
//                 }
//             }
//         }

//         Mat output = img.clone();
//         if (!bestContour.empty()) {
//             polylines(output, bestContour, true, Scalar(0, 255, 0), 3);
//         }

//         Mat masked(img.size(), img.type(), Scalar(255, 255, 255));  // 預設全黑

//         if (!bestContour.empty()) {

//                 polylines(output, bestContour, true, Scalar(0, 255, 0), 3);

//                 Mat mask = Mat::zeros(img.size(), CV_8UC1);
//                 vector<vector<Point>> fillContours = { bestContour };
//                 fillPoly(mask, fillContours, Scalar(255));

//                 bitwise_and(img, img, masked, mask);
//         }

//         return masked;
//     }
    

// private:
//     int hue_min_ = 0, hue_max_ = 179;
//     int sat_min_ = 0, sat_max_ = 255;
//     int val_min_ = 0, val_max_ = 255;
    
//     int coffee_number = 0; // 假設咖啡杯位置
//     int center_x, center_y; // 中心
//     std::string coffee_type = "Unknown"; // 假設咖啡杯類型

//     void updateTrackbarValues() {
//         hue_min_ = getTrackbarPos("Hue Min", "tracker");
//         hue_max_ = getTrackbarPos("Hue Max", "tracker");
//         sat_min_ = getTrackbarPos("Sat Min", "tracker");
//         sat_max_ = getTrackbarPos("Sat Max", "tracker");
//         val_min_ = getTrackbarPos("Val Min", "tracker");
//         val_max_ = getTrackbarPos("Val Max", "tracker");
//     }
// };



int main() {
    VideoCapture cap(0, CAP_V4L2);
    Mat img, edge, masked, end; 
    CameraStage2 camera2;

    while (true) {
        bool ret = cap.read(img);
        if (!ret) {
            std::cout << "can't receive frame\n";
            break;
        }
        imshow("image", img); //初始

        edge = camera2.EdgeDetect(img);
        imshow("Edge Detection", edge);  // 邊緣檢測結果

        masked = camera2.layerMask(img);
        // imshow("Masked Result", masked);　　// 層遮罩結果

        end = camera2.findFourSquare(masked);
        imshow("Four Square Detection", end);  // 四方形檢測結果

        if (waitKey(1) == 27) break;
    }
    return 0;
}

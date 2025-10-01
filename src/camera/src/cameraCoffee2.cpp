#include "camera/cameraBackground.hpp"
#include <iostream>
#include "mainspace/msg/coffee.hpp"
#include "mainspace/msg/command.hpp"
#include <chrono>

using namespace cv;
using namespace std;

class CameraCoffee2 : public Camera {
public:
    CameraCoffee2() : Camera(false)
    {
        combo = 0;
        using namespace std::chrono_literals;

        coffee_pub_ = this->create_publisher<mainspace::msg::Coffee>("/coffee", 10);

        timer_ = this->create_wall_timer(
            300ms, std::bind(&CameraCoffee2::timerCallback, this));

        //test, 刪///////////////////////////////////////////////////////////////////////
        // mainspace::msg::Coffee coffee_msg;
        // coffee_msg.type = "black";
        // coffee_msg.number = 4;
        // for(int i=0; i<10; i++) 
        // {
        //     coffee_pub_->publish(coffee_msg);
        //     rclcpp::sleep_for(30ms);
        // }
    }

    //while loop for camera
    void timerCallback() 
    {
        //find camera
        if (!cap.isOpened()) {
            cap.open(17, CAP_V4L2);
            if(!cap.isOpened()) return;
        }
        bool ret = cap.read(img);
        if (!ret) {
             // std::cout << "can't receive frame\n";
            return;
        }

        // imshow("image", img); //初始

        edge = EdgeDetect(img);
        // imshow("Edge Detection", edge);  // 邊緣檢測結果

        masked = layerMask(img);
        // imshow("Masked Result", masked);　　// 層遮罩結果

        end = findFourSquare(img);
        // imshow("Four Square Detection", end);  // 四方形檢測結果
    }

    // LayerMask for black big rectangle
    Mat layerMask(Mat img)
    {
        Mat edge = EdgeDetect(img);
        vector<vector<Point>> contours;
        findContours(edge, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

        // double maxArea = 0;
        vector<Point> bestContour;

        // for (const auto& contour : contours) {
        //     double area = contourArea(contour);
        //     if (area < 30000) continue;  // 過濾太小的雜訊

        //     vector<Point> approx;
        //     approxPolyDP(contour, approx, 0.02 * arcLength(contour, true), true);

        //     if (approx.size() == 4 && isContourConvex(approx)) {
        //         if (area > maxArea) {
        //             maxArea = area;
        //             bestContour = approx;
        //             Moments m = moments(approx);
        //             center_x = static_cast<int>(m.m10 / m.m00);
        //             center_y = static_cast<int>(m.m01 / m.m00);
        //         }
        //     }
        // }

        center_x = img.cols *0.5;
        center_y = img.rows *0.65;
        bestContour.push_back(Point(img.cols*0.1, img.rows*0.3));                     // 左上
        bestContour.push_back(Point(img.cols*0.9 - 1, img.rows*0.3));      // 右上
        bestContour.push_back(Point(img.cols*0.9 - 1, img.rows - 1)); // 右下
        bestContour.push_back(Point(img.cols*0.1, img.rows - 1));      // 左下

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

        return img;
    }

    //find and distinguish four square inside the rectangle
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
        // imshow("E", edge);

        vector<vector<Point>> contours;
        findContours(edge, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

        Mat output = masked.clone();

        for (const auto& contour : contours) {
            double area = contourArea(contour);
            // printf("Area: %f\n", area);
            if (area < 3000 || area > 80000) continue; 

            vector<Point> approx;
            approxPolyDP(contour, approx, 0.02 * arcLength(contour, true), true);

            if (approx.size() == 4 && isContourConvex(approx)) {
                // 計算中心點
                Moments m = moments(approx);
                int cx = static_cast<int>(m.m10 / m.m00);
                int cy = static_cast<int>(m.m01 / m.m00);

                // 檢查中心點的像素值是否為黑色（近似判斷）
                Vec3b pixel = masked.at<Vec3b>(cy, cx);
                bool isBlack = (pixel[0] < 70 || pixel[1] < 70 || pixel[2] < 70); // BGR 都接近 0

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
                    color = (totalColor < 300) ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
                }
                if(color != Scalar(0, 0, 255)) 
                {
                    int dx = cx - center_x;
                    int dy = cy - center_y;
                    if(dx >= 0 && dy <= 0) {
                        coffee_number = 3; // 假設咖啡杯在右上
                    } else if(dx <= 0 && dy <= 0) {
                        coffee_number = 2; // 假設咖啡杯在左上
                    } else if(dx <= 0 && dy >= 0) {
                        coffee_number = 0; // 假設咖啡杯在左下
                    } else if(dx >= 0 && dy >= 0) {
                        coffee_number = 1; // 假設咖啡杯在右下
                    }
                    
                    if(color == Scalar(0, 255, 0)) {
                        coffee_type = 0;
                    } else {
                        coffee_type = 1;
                    }
                    if(last_coffee_number == coffee_number && last_coffee_type == coffee_type) combo++;
                    else combo = 0;

                    if(combo > 5)
                    {
                        mainspace::msg::Coffee coffee_msg;
                        coffee_msg.type = coffee_type;
                        coffee_msg.number = coffee_number;
                        coffee_pub_->publish(coffee_msg);
                    }
                    
                    last_coffee_number = coffee_number;
                    last_coffee_type = coffee_type;

                    // printf("Coffee number: %d, Type: %d\n", coffee_number, coffee_type);
                }
                polylines(output, approx, true, color, 2);
            }
        }

        return output;
    }

private:

    //layerMask canter point
    int center_x, center_y;

    // coffee info
    int coffee_number = 0;
    int coffee_type = 0;
    int last_coffee_number = 0;
    int last_coffee_type = 0;
    int combo = 0;

    rclcpp::Publisher<mainspace::msg::Coffee>::SharedPtr coffee_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    VideoCapture cap;
    Mat img, edge, masked, end; 
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto camera2 = std::make_shared<CameraCoffee2>();
    rclcpp::spin(camera2);
    rclcpp::shutdown();
    // CameraCoffee2 camera2;
    // Mat img, edge, masked, end; 
    // VideoCapture cap(6, CAP_V4L2);
    // while (true) {
    //     bool ret = cap.read(img);
    //     if (!ret) {
    //          // std::cout << "can't receive frame\n";
    //         break;
    //     }
    //     imshow("image", img); //初始

    //     edge = camera2.EdgeDetect(img);
    //     // imshow("Edge Detection", edge);  // 邊緣檢測結果

    //     masked = camera2.layerMask(img);
    //     // imshow("Masked Result", masked);　　// 層遮罩結果

    //     end = camera2.findFourSquare(masked);
    //     imshow("Four Square Detection", end);  // 四方形檢測結果

    //     waitKey(1);
    //     if (waitKey(1) == 27) break; // 按下 ESC
    // }


    return 0;
}

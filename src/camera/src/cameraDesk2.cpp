#include "camera/cameraBackground.hpp"
#include <iostream>
#include "mainspace/msg/desk.hpp"

using namespace cv;
using namespace std;

class CameraDesk2 : public Camera {
public:
    CameraDesk2() : Camera(false) 
    {
        using namespace std::chrono_literals;

        desk_pub_ = this->create_publisher<mainspace::msg::Desk>("/desk", 10);

        timer_ = this->create_wall_timer(
            500ms, std::bind(&CameraDesk2::timerCallback, this));

        // mainspace::msg::Desk desk_msg;
        // desk_msg.x = 0; // 假設桌子位置
        // desk_msg.y = -100; // 假設桌子位置
        // desk_pub_->publish(desk_msg);

    }
    void timerCallback() 
    {
        //find camera
        if (!cap.isOpened()) {
            cap.open(4, CAP_V4L2);
            if(!cap.isOpened()) return;
        }
        bool ret = cap.read(img);
        if (!ret) {
            // std::cout << "can't receive frame\n";
            return;
        }
        // imshow("image", img); //初始

        // desk = camera2.findDesk(img);
        // imshow("Desk Detection", desk);  // 桌面檢測結果

        edge = EdgeDetect(img);
        // imshow("Edge Detection", edge);  // 邊緣檢測結果

        findout = GetSquarePoint(edge);
        // imshow("Square Detection", findout);  // 四方形檢測結果
    }
    Mat GetSquarePoint(Mat img)
    {
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
                }
            }
        }
        printf("Center: (%d, %d)\n", center_x, center_y);
        
        //send desk position
        mainspace::msg::Desk desk_msg;
        desk_msg.x = center_x;
        desk_msg.y = center_y;
        desk_pub_->publish(desk_msg);

        // Mat output = img.clone();
        // rectangle(output, bestRect, Scalar(255, 0, 0), 2);
        return img;
    }

        //有一大段空閒時間再嘗試透視變換///////////////////////////////////////////////////////
    // Mat findDesk(Mat img) 
    // {
    //     Mat hsv, mask, result;
    //     cvtColor(img, hsv, COLOR_BGR2HSV);
    //     inRange(hsv, Scalar(hue_min_, sat_min_, val_min_), 
    //             Scalar(hue_max_, sat_max_, val_max_),mask);
    //     // inRange(hsv, Scalar(orange_min_hue, orange_min_sat, orange_min_val), 
    //     //         Scalar(orange_max_hue, orange_max_sat, orange_max_val), mask);
    //     bitwise_and(img, img, result, mask);

    //     vector<vector<Point>> contours;
    //     findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    //     double maxArea = 0;
    //     vector<Point> bestContour;
    //     for (const auto& contour : contours) {
    //         double area = contourArea(contour);
    //         if (area > maxArea && area > 1500) { 
    //             maxArea = area;
    //             bestContour = contour;
    //         }
    //     }

    //     Mat masked(img.size(), img.type(), Scalar(255, 255, 255));
    //     if (!bestContour.empty()) {
    //         drawContours(result, vector<vector<Point>>{bestContour}, -1, Scalar(255, 0, 0), 2);
    //         Mat mask = Mat::zeros(img.size(), CV_8UC1);
    //         vector<vector<Point>> fillContours = { bestContour };
    //         fillPoly(mask, fillContours, Scalar(255));
    //         bitwise_and(result, result, masked, mask);
    //         // imshow("Desk Masked", masked); // 顯示桌面遮罩結果

    //         // Mat warped = warpDesk(masked, bestContour);
    //         // return warped; // 回傳俯視圖
    //     }

    //     return masked;
    // } 
    // Mat warpDesk(Mat img, vector<Point> contour)
    // {
    //     // 擬合四邊形
    //     vector<Point> approx;
    //     approxPolyDP(contour, approx, 0.02 * arcLength(contour, true), true);

    //     if (approx.size() != 4) {
    //         return img.clone(); // 找不到四角就回原圖
    //     }

    //     // 將 Point 轉為 Point2f
    //     vector<Point2f> srcPoints;
    //     for (auto &p : approx) srcPoints.push_back(Point2f(p.x, p.y));

    //     // 排序四個角（左上、右上、右下、左下）
    //     sort(srcPoints.begin(), srcPoints.end(), [](Point2f a, Point2f b) {
    //         return a.y < b.y || (a.y == b.y && a.x < b.x);
    //     });
    //     Point2f tl = srcPoints[0].x < srcPoints[1].x ? srcPoints[0] : srcPoints[1];
    //     Point2f tr = srcPoints[0].x > srcPoints[1].x ? srcPoints[0] : srcPoints[1];
    //     Point2f bl = srcPoints[2].x < srcPoints[3].x ? srcPoints[2] : srcPoints[3];
    //     Point2f br = srcPoints[2].x > srcPoints[3].x ? srcPoints[2] : srcPoints[3];

    //     // 設定輸出大小（這裡假設轉成 800x600）
    //     float width = 800, height = 600;
    //     vector<Point2f> dstPoints = {
    //         Point2f(0, 0),
    //         Point2f(width - 1, 0),
    //         Point2f(width - 1, height - 1),
    //         Point2f(0, height - 1)
    //     };

    //     // 計算透視變換矩陣
    //     Mat matrix = getPerspectiveTransform(vector<Point2f>{tl, tr, br, bl}, dstPoints);

    //     // 透視變換
    //     Mat warped;
    //     warpPerspective(img, warped, matrix, Size(width, height));

    //     return warped;
    // }

private:

    int center_x, center_y;

    rclcpp::Publisher<mainspace::msg::Desk>::SharedPtr desk_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    VideoCapture cap;
    Mat img, edge, findout; 
};



int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraDesk2>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

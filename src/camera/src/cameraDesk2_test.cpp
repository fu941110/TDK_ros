#include "camera/cameraBackground.hpp"
#include <iostream>

using namespace cv;
using namespace std;

class CameraDesk2{
public:
    CameraDesk2()
    {
        using namespace std::chrono_literals;

        // desk_pub_ = this->create_publisher<mainspace::msg::Desk>("/desk", 10);

        // timer_ = this->create_wall_timer(
        //     300ms, std::bind(&CameraDesk2::timerCallback, this));

        // mainspace::msg::Desk desk_msg;
        // desk_msg.x = 0; // 假設桌子位置
        // desk_msg.y = -100; // 假設桌子位置
        // desk_pub_->publish(desk_msg);

    }
    // void timerCallback() 
    // {
    //     //find camera
    //     if (!cap.isOpened()) {
    //         cap.open(6, CAP_V4L2);
    //         if(!cap.isOpened()) return;
    //     }
    //     bool ret = cap.read(img);
    //     if (!ret) {
    //         // std::cout << "can't receive frame\n";
    //         return;
    //     }
    //     // imshow("image", img); //初始

    //     // desk = camera2.findDesk(img);
    //     // imshow("Desk Detection", desk);  // 桌面檢測結果

    //     edge = EdgeDetect(img);
    //     // imshow("Edge Detection", edge);  // 邊緣檢測結果

    //     findout = GetSquarePoint(edge);
    //     // imshow("Square Detection", findout);  // 四方形檢測結果
    // }
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
        // mainspace::msg::Desk desk_msg;
        // desk_msg.x = center_x;
        // desk_msg.y = center_y;
        // desk_pub_->publish(desk_msg);

        // Mat output = img.clone();
        // rectangle(output, bestRect, Scalar(255, 0, 0), 2);
        return img;
    }
    Mat EdgeDetect(Mat img) {
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

    Mat GetSquarePoint_Fast(Mat img)
    {
        const int rectWidth = 200;
        const int rectHeight = 200;

        // 建立積分影像 (int type，避免 overflow)
        Mat integralImg;
        integral(img, integralImg, CV_64F);  

        double maxBrightness = -1;
        Rect bestRect;
        int center_x = 0, center_y = 0;

        // 用積分影像快速計算每個 ROI 的總亮度
        for (int y = 0; y <= img.rows - rectHeight; y += 20) {
            for (int x = 0; x <= img.cols - rectWidth; x += 20) {
                int x2 = x + rectWidth;
                int y2 = y + rectHeight;

                // 使用積分影像計算區塊亮度
                double sum = integralImg.at<double>(y2, x2)
                        - integralImg.at<double>(y, x2)
                        - integralImg.at<double>(y2, x)
                        + integralImg.at<double>(y, x);

                if (sum > maxBrightness) {
                    maxBrightness = sum;
                    bestRect = Rect(x, y, rectWidth, rectHeight);
                    center_x = x + rectWidth / 2 - (img.cols / 2);
                    center_y = y + rectHeight / 2 - (img.rows / 2);
                }
            }
        }

        printf("Center: (%d, %d)\n", center_x, center_y);

        // 發布桌子位置
        // mainspace::msg::Desk desk_msg;
        // desk_msg.x = center_x;
        // desk_msg.y = center_y;
        // desk_pub_->publish(desk_msg);

        // 畫出偵測到的 ROI
        Mat output = img.clone();
        rectangle(output, bestRect, Scalar(0, 255, 0), 2);
        return output;
    }


private:

    int center_x, center_y;

    // rclcpp::Publisher<mainspace::msg::Desk>::SharedPtr desk_pub_;
    // rclcpp::TimerBase::SharedPtr timer_;

    // VideoCapture cap;
    // Mat img, edge, findout; 
};



int main(int argc, char **argv) 
{
    VideoCapture cap(6, CAP_V4L2);
    Mat img, edge, findout; 
    CameraDesk2 camera2;

    while (true) {
        bool ret = cap.read(img);
        if (!ret) {
            // std::cout << "can't receive frame\n";
        }
        imshow("image", img); //初始

        // desk = camera2.findDesk(img);
        // imshow("Desk Detection", desk);  // 桌面檢測結果

        edge = camera2.EdgeDetect(img);
        imshow("Edge Detection", edge);  // 邊緣檢測結果

        findout = camera2.GetSquarePoint_Fast(edge);
        imshow("Square Detection", findout);  // 四方形檢測結果

        if (waitKey(1) == 27) break;
    }
    return 0;
}

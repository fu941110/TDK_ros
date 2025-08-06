#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

void trackBar();
Mat HSV(Mat img);
Mat Mask(Mat img_hsv);
Mat Mix(Mat img, Mat mask);
Mat contours(Mat img, Mat mask);

int main(){

    VideoCapture cap(0);
    Mat img;
    trackBar();
    while(true){
        bool ret = cap.read(img);
        if(!ret){  // 確認有影像傳輸
            std::cout<<"can't receive frame\n";
            break;
        }
        imshow("image", img);
        Mat img_hsv = HSV(img);
        // imshow("img_hsv", img_hsv);
        Mat mask = Mask(img_hsv);
        // imshow("mask", mask);
        Mat mix_img = Mat::zeros(img.size(), CV_8UC3);
        mix_img = Mix(img, mask);
        // imshow("result", mix_img);
        Mat end_img = contours(mix_img, mask);
        imshow("end", end_img);

        if(waitKey(1) == 27) break;
    }
    return 0;
}

void trackBar()
{
    namedWindow("tracker");
    resizeWindow("tracker", 300, 300);

    createTrackbar("Hue Min", "tracker", 0, 179);
    createTrackbar("Hue Max", "tracker", 0, 179);
    createTrackbar("Sat Min", "tracker", 0, 255);
    createTrackbar("Sat Max", "tracker", 0, 255);
    createTrackbar("Val Min", "tracker", 0, 255);
    createTrackbar("Val Max", "tracker", 0, 255);

    setTrackbarPos("Hue Min", "tracker", 0);
    setTrackbarPos("Hue Max", "tracker", 179);
    setTrackbarPos("Sat Min", "tracker", 0);
    setTrackbarPos("Sat Max", "tracker", 255);
    setTrackbarPos("Val Min", "tracker", 0);
    setTrackbarPos("Val Max", "tracker", 255);
}

Mat HSV(Mat img)
{
    Mat img_hsv;
    cvtColor(img, img_hsv, COLOR_BGR2HSV);
    return img_hsv;
}

Mat Mask(Mat img)
{
    int hue_m = getTrackbarPos("Hue Min", "tracker");
    int hue_M = getTrackbarPos("Hue Max", "tracker");
    int sat_m = getTrackbarPos("Sat Min", "tracker");
    int sat_M = getTrackbarPos("Sat Max", "tracker");
    int val_m = getTrackbarPos("Val Min", "tracker");
    int val_M = getTrackbarPos("Val Max", "tracker");
    
    // Scalar 實例 lower, upper
    Scalar lower(hue_m, sat_m, val_m); 
    Scalar upper(hue_M, sat_M, val_M);
    // 將 img_hsv 過濾至 mask 
    Mat mask; 
    inRange(img, lower, upper, mask);
    // 形態學處理去除雜訊
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(mask, mask, kernel);
    dilate(mask, mask, kernel);

    // 可選：模糊處理
    GaussianBlur(mask, mask, Size(3, 3), 0);
    return mask;
}

Mat Mix(Mat img, Mat mask)
{
    Mat result = Mat::zeros(img.size(), CV_8UC3);
    result.setTo(Scalar(255,255,255)); // 全白
    result.setTo(Scalar(0,0,0), mask); // mask區設黑
    return result;
}

Mat contours(Mat img, Mat mask)  //img 作為背景 , mask 分析
{
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Mat contourOutput = img.clone();
    Rect MaxboundingBox;
    double maxarea = 0;

    for (size_t i = 0; i < contours.size(); i++) 
    {
        Rect boundingBox = boundingRect(contours[i]);
        double area = boundingBox.area();
        if (area > maxarea) {
            maxarea = area;
            MaxboundingBox = boundingBox;
        }
    }
    // 畫出矩形框
    rectangle(contourOutput, MaxboundingBox, Scalar(0, 0, 255), 3);
    
    // 顯示中心座標
    Point rectcenter(MaxboundingBox.x + MaxboundingBox.width / 2, MaxboundingBox.y + MaxboundingBox.height / 2);
    Point imgcenter(img.cols / 2, img.rows / 2);

    circle(contourOutput, rectcenter, 5, Scalar(255, 0, 255), -1);  
    circle(contourOutput, imgcenter, 5, Scalar(255, 0, 255), -1);  
    line(contourOutput, imgcenter, rectcenter, Scalar(155, 0, 255), 2);

    Point delta = Point(rectcenter.x - imgcenter.x, rectcenter.y - imgcenter.y);
    Point textpoint = Point(10 + (rectcenter.x + imgcenter.x)/2, (rectcenter.y + imgcenter.y)/2);
        
    // 顯示座標文字
    std::string text = "(" + std::to_string(delta.x) + "," + std::to_string(delta.y) + ")";
    putText(contourOutput, text, textpoint, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 55, 255), 1);

    return contourOutput;
}
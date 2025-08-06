#pragma once

#include <opencv2/opencv.hpp>
#include <string>

class Camera {
public:
    Camera();
    virtual ~Camera() = default;
    cv::Mat EdgeDetect(cv::Mat img);
    void setTrackbarPosition(int hue_min, int hue_max, int sat_min, int sat_max, int val_min, int val_max);
    int hue_min_ = 0, hue_max_ = 179;
    int sat_min_ = 0, sat_max_ = 255;
    int val_min_ = 0, val_max_ = 255;
    
private:
    void TrackerBar();
    void updateTrackbarValues();
};
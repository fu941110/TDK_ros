#pragma once

#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>

class Camera : public rclcpp::Node
{
public:
    Camera(bool init = true);
    virtual ~Camera() = default;
    cv::Mat EdgeDetect(cv::Mat img);
    void setTrackbarPosition(int hue_min, int hue_max, int sat_min, int sat_max, int val_min, int val_max);
    int hue_min_ = 0, hue_max_ = 179;
    int sat_min_ = 0, sat_max_ = 255;
    int val_min_ = 0, val_max_ = 255;
    
private:
    void TrackerBar();
    void updateTrackbarValues();

protected:
/*Camera port

    usb-0000:00:14.0-?
    ?: my computer: 1
                    2               3

    /dev/video6 (desk),  /dev/video12(coffee)

    usb-0000:01:00.0-?
    ?: pi:   3       1
             4       2

    /dev/video4 (desk),  /dev/video17 (coffee)        
*/

    int coffee_port_ = 17;
    int desk_port_ = 4;
};


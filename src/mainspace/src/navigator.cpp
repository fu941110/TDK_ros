#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "mainspace/msg/position.hpp"
#include "mainspace/msg/to_stm_speed.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>

using namespace std::chrono_literals;

const double AngleRange = 0.02;
const double DistanceRange = 0.02;

struct WayPoint {
  double x;
  double y;
  double theta;
  int mode;  //"straight", "rotate", "arc", "shift", "mix"
  double radius; // for arc
};

class NavigatorNode : public rclcpp::Node
{
public:
  NavigatorNode()
  : Node("navigator_node")  
  {
    using std::placeholders::_1;
    using std::placeholders::_2;

    position_sub_ = this->create_subscription<mainspace::msg::Position>(
      "/stm_position", 10, std::bind(&NavigatorNode::positionCallback, this, _1));

    speed_pub_ = this->create_publisher<mainspace::msg::ToStmSpeed>(
      "/motor_speed", 10);

    //存座標點
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    //讀取csv路徑
    std::string csv_path = ament_index_cpp::get_package_share_directory("mainspace") + "/MainPath_csv/Stage2.csv";
    loadWayPoints(csv_path);

    control_timer_ = this->create_wall_timer(
      100ms, std::bind(&NavigatorNode::controlLoop, this));
  }
  //讀csv檔
  void loadWayPoints(const std::string &filename)
  {
    std::ifstream file(filename);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open CSV file: %s", filename.c_str());
      return;
    }

    std::string line;
    bool first_line = true;

    while (std::getline(file, line)) {
      if (line.empty()) continue;

      std::stringstream ss(line);
      std::string sx, sy, stheta, smode, sradius;

      if (!std::getline(ss, sx, ',')) continue;
      if (!std::getline(ss, sy, ',')) continue;
      if (!std::getline(ss, stheta, ',')) continue;
      if (!std::getline(ss, smode, ',')) continue;
      if (!std::getline(ss, sradius, ',')) continue;


      if (first_line) {
        first_line = false;
        if (!isdigit(sx[0])) continue; // 跳過表頭
      }

      WayPoint wp;
      wp.x = std::stod(sx);
      wp.y = std::stod(sy);
      wp.theta = std::stod(stheta);
      wp.mode = std::stoi(smode);
      wp.radius = std::stod(sradius);
      waypoints_.push_back(wp);
    }
  }

private:
  // 接收位置資訊
  void positionCallback(const mainspace::msg::Position::SharedPtr msg)
  {
    last_position_ = *msg;
    has_position_ = true;
  }

  // 計算速度百分比
  double SpeedPercent(double distance, double k, double max_distance, double min_speed = 0.1, double max_speed = 1.0)
  {
    double percent = std::log1p(k * std::abs(distance)) / std::log1p(k * max_distance);
    percent = std::clamp(percent, 0.0, 1.0);
    return (distance < 0 ? -1 : 1) * (min_speed + (max_speed - min_speed) * percent); 
  }
  // 檢查是否到達路徑點
  // 如果到達，則更新當前索引並發布停止命令
  bool ArriveAtWaypoint()
  {
    mainspace::msg::ToStmSpeed stop_cmd;
    stop_cmd.vx = 0.0;
    stop_cmd.vy = 0.0;
    stop_cmd.w  = 0.0;
    speed_pub_->publish(stop_cmd);
    RCLCPP_INFO(this->get_logger(), "Arrived at waypoint (%f, %f, %f)", 
                waypoints_[current_index_].x, 
                waypoints_[current_index_].y, 
                waypoints_[current_index_].theta);
    if (current_index_+1 >= waypoints_.size()) {
      return true; // 所有路徑點已到達
    }
    else current_index_++; 
    return false; // 還有路徑點未到達
  }

  //主迴圈/////////////////////////////////////////////////////////////////
  void controlLoop()
  {
    double vx = 0.0;
    double vy = 0.0;
    double w  = 0.0;

    if (!has_position_ && waypoints_.empty()) 
    {
      // 如果沒有接收到位置資訊或沒有路徑點，則不執行控制，速度調整成零
      mainspace::msg::ToStmSpeed stop_cmd;
      stop_cmd.vx = 0.0;
      stop_cmd.vy = 0.0;
      stop_cmd.w  = 0.0;
      speed_pub_->publish(stop_cmd);
      RCLCPP_WARN(this->get_logger(), "No position data or waypoints available.");
      return;
    }

    // [1] 計算導航行為：從目前位置導航至 target
    const auto &target = waypoints_[current_index_];
    double dx = target.x - last_position_.x;
    double dy = target.y - last_position_.y;
    double distance = std::hypot(dx, dy);
    double angle_error = target.theta - last_position_.theta;
    angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error)); // Normalize

    double d_forward = std::cos(last_position_.theta) * dx + std::sin(last_position_.theta) * dy;
    double d_shift = -std::sin(last_position_.theta) * dx + std::cos(last_position_.theta) * dy;
    vx = SpeedPercent(d_forward, 1, 0.4);
    vy = SpeedPercent(d_shift, 1, 0.4);
    w = SpeedPercent(angle_error, 3, 1.0, 0.05, 1.0);

    //分不同mode進行
    switch (target.mode) {
      case 1: 
        // 直線行駛
        vy = 0.0;
        // 檢查是否到達目標點
        if (distance < DistanceRange && std::abs(angle_error) < AngleRange) {
          ArriveAtWaypoint();
          return;
        }
        break;

      case 2:
        // 旋轉至目標角度
        vx = 0.0;
        vy = 0.0;
        if (std::abs(angle_error) < AngleRange) {
          ArriveAtWaypoint();
          return;
        }
        break;

      case 3:
        // 偏移行駛
        vx = 0.0;
        if (distance < DistanceRange && std::abs(angle_error) < AngleRange) {
          ArriveAtWaypoint();
          return;
        }
        break;

      case 4:
        // 弧形行駛
        if (target.radius == 0.0) return;
        vx = std::abs(target.radius) * std::abs(w);
        vy = 0.0;

        if (std::abs(angle_error) < AngleRange) {
          ArriveAtWaypoint();
          return;
        }
        break;

      case 5:
        // 混合模式
        if (distance < DistanceRange && std::abs(angle_error) < AngleRange) {
          ArriveAtWaypoint();
          return;
        }

        break;
    }

    vx = std::clamp(vx, -1.0, 1.0);  // 單位: m/s
    vy = std::clamp(vy, -1.0, 1.0);
    w = std::clamp(w, -1.0, 1.0);

    // [4] 發送至 STM32
    mainspace::msg::ToStmSpeed cmd;
    cmd.vx = vx;
    cmd.vy = vy;
    cmd.w  = w;
    speed_pub_->publish(cmd);

    /////////////////////模擬用
    // 模擬位置更新（每 100ms 依據速度移動）
    last_position_.x += (cos(last_position_.theta)*vx - sin(last_position_.theta)*vy) * 0.1; 
    last_position_.y += (sin(last_position_.theta)*vx + cos(last_position_.theta)*vy) * 0.1;
    last_position_.theta += w * 0.1;
    // 正規化角度
    last_position_.theta = std::atan2(std::sin(last_position_.theta), std::cos(last_position_.theta));
    RCLCPP_INFO(this->get_logger(), "Current position: (%f, %f, %f)", 
                last_position_.x, last_position_.y, last_position_.theta);
                
    RvizShow();
  }

  //發布 RViz 顯示資訊
  void RvizShow()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = last_position_.x;
    transform.transform.translation.y = last_position_.y;
    transform.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, last_position_.theta);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transform);

    // [6] 發布 Odometry 資訊
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = last_position_.x;
    odom.pose.pose.position.y = last_position_.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = 0.0;  // 可選填真實速度
    odom.twist.twist.angular.z = 0.0;

    odom_pub_->publish(odom);
  }

protected:
  // ROS interface
  rclcpp::Subscription<mainspace::msg::Position>::SharedPtr position_sub_;
  rclcpp::Publisher<mainspace::msg::ToStmSpeed>::SharedPtr speed_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  //rviz
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 狀態
  mainspace::msg::Position last_position_;
  bool has_position_ = true;
  // 目標位置
  std::vector<WayPoint> waypoints_;
  size_t current_index_ = 0;

  now_stage = 1; // 當前階段
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
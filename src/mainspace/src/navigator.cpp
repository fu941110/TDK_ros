#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "mainspace/msg/position.hpp"
#include "mainspace/msg/encoder_speed.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class NavigatorNode : public rclcpp::Node
{
public:
  NavigatorNode()
  : Node("navigator_node")
  {
    using std::placeholders::_1;

    position_sub_ = this->create_subscription<mainspace::msg::Position>(
      "/stm_position", 10, std::bind(&NavigatorNode::positionCallback, this, _1));

    encoder_pub_ = this->create_publisher<mainspace::msg::EncoderSpeed>(
      "/motor_speed", 10);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 設定定時控制迴圈（例如 50Hz）
    control_timer_ = this->create_wall_timer(
      100ms, std::bind(&NavigatorNode::controlLoop, this));
  }

private:
  void positionCallback(const mainspace::msg::Position::SharedPtr msg)
  {
    current_x_ = msg->x;
    current_y_ = msg->y;
    current_theta_ = msg->theta;
    has_position_ = true;
  }

  void controlLoop()
  {
    if (!has_position_) return;

    // [1] 計算導航行為：從目前位置導航至 target
    double dx = target_x_ - current_x_;
    double dy = target_y_ - current_y_;
    double distance = std::hypot(dx, dy);
    // double target_theta = std::atan2(dy, dx);
    double angle_error = target_theta_ - current_theta_;

    double vx = std::cos(current_theta_) * dx + std::sin(current_theta_) * dy;
    double vy = -std::sin(current_theta_) * dx + std::cos(current_theta_) * dy;

    angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error)); // Normalize

    // [2] 設定前進與旋轉速度
    vx *= 25;   
    vy *= 25;   
    double w = 0.04 * angle_error;

    vx = std::clamp(vx, -50.0, 50.0);  // 單位: m/s
    vy = std::clamp(vy, -50.0, 50.0);
    w = std::clamp(w, -1.0, 1.0);

    // [3] 將速度轉為輪子速度（Mecanum）
    double L = 0.5, W = 0.5, r = 0.12;  // 根據實際情況設定
    double fl = (vx - vy - w * (L + W)) / r;
    double fr = (vx + vy + w * (L + W)) / r;
    double rl = (vx + vy - w * (L + W)) / r;
    double rr = (vx - vy + w * (L + W)) / r;

    // [4] 發送至 STM32
    mainspace::msg::EncoderSpeed cmd;
    cmd.fl = static_cast<int32_t>(fl);
    cmd.fr = static_cast<int32_t>(fr);
    cmd.rl = static_cast<int32_t>(rl);
    cmd.rr = static_cast<int32_t>(rr);
    encoder_pub_->publish(cmd);

    RvizShow();
  }

  void RvizShow()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = current_x_;
    transform.transform.translation.y = current_y_;
    transform.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, current_theta_);
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
    odom.pose.pose.position.x = current_x_;
    odom.pose.pose.position.y = current_y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = 0.0;  // 可選填真實速度
    odom.twist.twist.angular.z = 0.0;

    odom_pub_->publish(odom);
  }

  // ROS interface
  rclcpp::Subscription<mainspace::msg::Position>::SharedPtr position_sub_;
  rclcpp::Publisher<mainspace::msg::EncoderSpeed>::SharedPtr encoder_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  //rviz
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 狀態
  double current_x_ = 0.0, current_y_ = 0.0, current_theta_ = 0.0;
  double target_x_ = 1.0, target_y_ = 1.0, target_theta_ = M_PI/2;  // 可後續透過 service 或 param 設定
  bool has_position_ = false;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
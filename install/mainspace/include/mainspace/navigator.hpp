#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "mainspace/msg/position.hpp"
#include "mainspace/msg/to_stm_speed.hpp"
#include "mainspace/msg/csv_file.hpp"
#include "mainspace/msg/pause.hpp"
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
  int mode;
  double radius;
};

class NavigatorNode : public rclcpp::Node
{
public:
  NavigatorNode(bool init = true);
protected:
  void pauseCallback(const mainspace::msg::Pause::SharedPtr msg);
  void loadWayPoints(const std::string &filename);
  void positionCallback(const mainspace::msg::Position::SharedPtr msg);
  double SpeedPercent(double distance, double k, double max_distance, double min_speed, double max_speed);
  bool ArriveAtWaypoint();
  void controlLoop();
  void RvizShow();
  void CsvFileReceive(const mainspace::msg::CsvFile::SharedPtr msg);

  rclcpp::Subscription<mainspace::msg::Position>::SharedPtr position_sub_;
  rclcpp::Publisher<mainspace::msg::ToStmSpeed>::SharedPtr speed_pub_;
  rclcpp::Publisher<mainspace::msg::Position>::SharedPtr last_waypoint_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Subscription<mainspace::msg::Pause>::SharedPtr pause_sub_;
  rclcpp::Subscription<mainspace::msg::CsvFile>::SharedPtr csvfile_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  mainspace::msg::Position last_position_;
  std::vector<WayPoint> waypoints_;
  WayPoint last_waypoint_;
  bool is_paused_ = false;
  std::string csv_path;
};
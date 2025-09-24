#include "mainspace/navigator.hpp"

NavigatorNode::NavigatorNode(bool init)
: Node("navigator_node")
{
  if(init) {
    using std::placeholders::_1;
    using std::placeholders::_2;

    //msg for stm <-> ros2
    position_sub_ = this->create_subscription<mainspace::msg::Position>(
      "/stm_position", 10, std::bind(&NavigatorNode::positionCallback, this, _1));
    speed_pub_ = this->create_publisher<mainspace::msg::ToStmSpeed>(
      "/motor_speed", 10);
    
    //memorize last waypoint, StageManager use this to determine stage change
    last_waypoint_pub_ = this->create_publisher<mainspace::msg::Position>(
      "/last_waypoint", 10);      
    
    //to get csv file from each stage node
    csvfile_sub_ = this->create_subscription<mainspace::msg::CsvFile>(
      "/csv_file", 10, std::bind(&NavigatorNode::CsvFileReceive, this, _1));

    //pause v = 0
    pause_sub_ = this->create_subscription<mainspace::msg::Pause>(
     "/pause", 10, std::bind(&NavigatorNode::pauseCallback, this, _1));

    //publish odometry and tf
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    //mainLoop timer
    control_timer_ = this->create_wall_timer(
      250ms, std::bind(&NavigatorNode::controlLoop, this));

    //test///////////////////////////////////////////////////////////////
    csv_path = ament_index_cpp::get_package_share_directory("mainspace") + "/MainPath_csv/Stage2.csv";
    loadWayPoints(csv_path); 
  }
}

void NavigatorNode::CsvFileReceive(const mainspace::msg::CsvFile::SharedPtr msg)
{
  if(msg->file == csv_path) return;
  csv_path = msg->file;
  std::string csv = ament_index_cpp::get_package_share_directory("mainspace") + csv_path;
  loadWayPoints(csv);
  // RCLCPP_INFO(this->get_logger(), "CSV file loaded: %s", csv_path.c_str());
}

void NavigatorNode::loadWayPoints(const std::string &filename)
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
        if (!isdigit(sx[0])) continue;
      }

      WayPoint wp;
      wp.x = std::stod(sx);
      wp.y = std::stod(sy);
      wp.theta = std::stod(stheta) * M_PI / 180.0; // Convert degrees to radians
      wp.mode = std::stoi(smode);
      wp.radius = std::stod(sradius);
      waypoints_.push_back(wp);
    }
}

//receive position from STM, update last_position_
//also add a waypoint by setting theta to 9999
void NavigatorNode::positionCallback(const mainspace::msg::Position::SharedPtr msg)
{
  if(msg->theta > 9000)
  {
    mainspace::msg::ToStmSpeed temp_vel;
    temp_vel.vx = int(msg->x);
    temp_vel.vy = int(msg->y);
    temp_vel.w = 0;
    speed_pub_->publish(temp_vel);
    return;
  }
  else if(!waypoints_.empty() && msg->theta > 1000) 
  {
    return;
  }
  else if(msg->theta > 1000) 
  {
    WayPoint wp;
    wp.x = msg->x + last_position_.x;
    wp.y = msg->y + last_position_.y;
    wp.theta = last_position_.theta;
    wp.mode = 1;
    wp.radius = 0.0;
    waypoints_.push_back(wp);
    return;
  }
  last_position_ = *msg;
}

//pause callback, set speed to 0
void NavigatorNode::pauseCallback(const mainspace::msg::Pause::SharedPtr msg)
{
    is_paused_ = msg->pause;
}

//to calculate speed percent based on distance (當前距離目標多遠，　減速強度，　開始減速距離，　最小速度，　最大速度)
double NavigatorNode::SpeedPercent(double distance, double k, double max_distance, double min_speed, double max_speed)
{
    double percent = std::log1p(k * std::abs(distance)) / std::log1p(k * max_distance);
    percent = std::clamp(percent, 0.0, 1.0);
    return (distance < 0 ? -8 : 8) * (min_speed + (max_speed - min_speed) * percent); 
}

bool NavigatorNode::ArriveAtWaypoint()
{
    if (waypoints_.empty()) {
      return true;
    }
    mainspace::msg::ToStmSpeed stop_cmd;
    stop_cmd.vx = 0.0;
    stop_cmd.vy = 0.0;
    stop_cmd.w  = 0.0;
    speed_pub_->publish(stop_cmd);
    RCLCPP_INFO(this->get_logger(), "Arrived at waypoint (%f, %f, %f)", 
                waypoints_.front().x, 
                waypoints_.front().y, 
                waypoints_.front().theta);
    last_waypoint_ = waypoints_.front();
    waypoints_.erase(waypoints_.begin());
    RCLCPP_INFO(this->get_logger(), "last: %f, %f", last_waypoint_.x, last_waypoint_.y);

    mainspace::msg::Position last_pos;
    last_pos.x = last_waypoint_.x;
    last_pos.y = last_waypoint_.y;
    last_pos.theta = waypoints_.empty();
    last_waypoint_pub_->publish(last_pos);
    return false;
}

void NavigatorNode::controlLoop()
{
    double vx = 0.0;
    double vy = 0.0;
    double w  = 0.0;

    if (waypoints_.empty() || is_paused_) 
    {
      if(waypoints_.empty() && is_paused_)
      {
        mainspace::msg::ToStmSpeed stop_cmd;
        stop_cmd.vx = 0.0;
        stop_cmd.vy = 0.0;
        stop_cmd.w  = 0.0;
        speed_pub_->publish(stop_cmd);
      }
      // RCLCPP_WARN(this->get_logger(), "No position data or waypoints available.");
      return;
    }

    const auto &target = waypoints_[0];
    double dx = target.x - last_position_.x;
    double dy = target.y - last_position_.y;
    double distance = std::hypot(dx, dy);
    double angle_error = target.theta - last_position_.theta;
    angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));

    double d_forward = std::cos(last_position_.theta) * dx + std::sin(last_position_.theta) * dy;
    double d_shift = -std::sin(last_position_.theta) * dx + std::cos(last_position_.theta) * dy;
    vx = SpeedPercent(d_forward, 1, 0.4, 0.07, 1.0);
    vy = SpeedPercent(d_shift, 1, 0.4, 0.07, 1.0);
    w = SpeedPercent(angle_error, 3, 1.0, 0.05, 1.0);

    switch (target.mode) {
      case 1:  //normal
        if (distance < DistanceRange && std::abs(angle_error) < AngleRange) {
          ArriveAtWaypoint();
          return;
        }
        break;
      case 2:  //arc
        if (target.radius == 0.0) return;
        vx = std::abs(target.radius) * std::abs(w);
        vy = 0.0;
        if (std::abs(angle_error) < AngleRange) {
          ArriveAtWaypoint();
          return;
        }
        break;
      case 3:  //shift arc
        if (target.radius == 0.0) return;
        vx = 0.0;
        vy = std::abs(target.radius) * std::abs(w);
        if (std::abs(angle_error) < AngleRange) {
          ArriveAtWaypoint();
          return;
        }
        break;
      // case 4:
      //   if (target.radius == 0.0) return;
      //   vx = std::abs(target.radius) * std::abs(w);
      //   vy = 0.0;
      //   if (std::abs(angle_error) < AngleRange) {
      //     ArriveAtWaypoint();
      //     return;
      //   }
      //   break;
      // case 5:
      //   if (distance < DistanceRange && std::abs(angle_error) < AngleRange) {
      //     ArriveAtWaypoint();
      //     return;
      //   }
      //   break;
    }

    vx = std::clamp(vx, -8.0, 8.0);
    vy = std::clamp(vy, -8.0, 8.0);
    w = std::clamp(w, -8.0, 8.0);
    
    mainspace::msg::ToStmSpeed cmd;
    cmd.vx = int(vx + (vx > 0 ? 1 : -1));
    cmd.vy = int(vy + (vy > 0 ? 1 : -1));
    cmd.w  = int(w + (w > 0 ? 1 : -1));
    speed_pub_->publish(cmd);

    // //test/////////////////////////////////////////////////////////////////////////////
    // last_position_.x += (cos(last_position_.theta)*vx - sin(last_position_.theta)*vy) * 0.1; 
    // last_position_.y += (sin(last_position_.theta)*vx + cos(last_position_.theta)*vy) * 0.1;
    // last_position_.theta += w * 0.1;
    // last_position_.theta = std::atan2(std::sin(last_position_.theta), std::cos(last_position_.theta));
    // // RCLCPP_INFO(this->get_logger(), "Current position: (%f, %f, %f)", 
    // //             last_position_.x, last_position_.y, last_position_.theta);
                
    // RvizShow();
}

void NavigatorNode::RvizShow()
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
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.angular.z = 0.0;

    odom_pub_->publish(odom);
}

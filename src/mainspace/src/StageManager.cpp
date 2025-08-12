#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "mainspace/msg/coffee.hpp"
#include "mainspace/src/navigator.cpp"
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;


class StageManager : public NavigatorNode
{
public:
  NavigatorNode()
  : Node("StageManager")  
  {
    control_timer_manager_ = this->create_wall_timer(
      100ms, std::bind(&StageManager::ManagerLoop, this));

    Stage2_ = this->create_publisher<std_msgs::msg::Bool>("/Stage2_start", 10);
    cameraStage2_ = this->create_publisher<std_msgs::msg::Bool>("/cameraStage2_start", 10);
    cameraDesk2_ = this->create_publisher<std_msgs::msg::Bool>("/cameraDesk2_start", 10);
    Stage3_ = this->create_publisher<std_msgs::msg::Bool>("/Stage3_start", 10);
    Stage4_ = this->create_publisher<std_msgs::msg::Bool>("/Stage4_start", 10);
  }

private:
  void ManagerLoop()
  {
    switch (now_stage) {
      case 1:
        if(waypoints_[current_index_].x == 1.0 && waypoints_[current_index_].y == -1.0)
        {
            now_stage = 2; // 更新階段
        }
        break;
      case 2:
        if (current_index_ >= waypoints_.size()) {
            if(waypoints_[current_index_].x == 1.0 && waypoints_[current_index_].y == -1.0)
            {
                Stage2_->publish(std_msgs::msg::Bool{true});
                cameraStage2_->publish(std_msgs::msg::Bool{true});
            }
            else if(waypoints_[current_index_].x == 1.0 && waypoints_[current_index_].y == 2.0)
            {
                now_stage = 3; // 更新階段
                Stage2_->publish(std_msgs::msg::Bool{false});
            }
            else
            {
                Stage2_->publish(std_msgs::msg::Bool{true});
                cameraDesk2_->publish(std_msgs::msg::Bool{true});
            }
        }
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown stage: %d", now_stage);
    }
  }

protected:
    rclcpp::TimerBase::SharedPtr control_timer_manager_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Stage2_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cameraStage2_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cameraDesk2_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Stage3_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Stage4_;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StageManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
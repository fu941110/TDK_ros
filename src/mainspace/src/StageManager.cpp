#include "mainspace/navigator.hpp"
#include "mainspace/msg/coffee.hpp"
#include "mainspace/msg/desk.hpp"
#include "mainspace/msg/command.hpp"
#include "std_msgs/msg/bool.hpp"

//be used to change stage
struct CheckPoint {
  double x;
  double y;
};

//by class NavigatorNode
class StageManager : public NavigatorNode
{
public:
  StageManager()
  : NavigatorNode(false)
  {
    using std::placeholders::_1;

    //get position from stm, to detetmine reset stage
    position_sub_ = this->create_subscription<mainspace::msg::Position>(
      "/stm_position", 10, std::bind(&StageManager::StageReset, this, _1));

    //get last_waypoint by navigator
    last_waypoint_sub_ = this->create_subscription<mainspace::msg::Position>(
      "/last_waypoint", 10, std::bind(&StageManager::last_waypoint_callback, this, _1));

    //mainLoop
    control_timer_manager_ = this->create_wall_timer(
      100ms, std::bind(&StageManager::ManagerLoop, this));

    command_pub_ = this->create_publisher<mainspace::msg::Command>("/commandToSTM", 10);  

    //send Bool to control_node, then decide which node will be open or closed
    Stage2_ = this->create_publisher<std_msgs::msg::Bool>("/Stage2_start", 10);
    cameraCoffee2_ = this->create_publisher<std_msgs::msg::Bool>("/cameraCoffee2_start", 10);
    cameraDesk2_ = this->create_publisher<std_msgs::msg::Bool>("/cameraDesk2_start", 10);
    Stage3_ = this->create_publisher<std_msgs::msg::Bool>("/Stage3_start", 10);
    Stage4_ = this->create_publisher<std_msgs::msg::Bool>("/Stage4_start", 10);
  }

private:
  void last_waypoint_callback(const mainspace::msg::Position::SharedPtr msg)
  {
    //here, we only need x, y
    auto message = *msg;
    last_position_.x = message.x;
    last_position_.y = message.y;
  }
  bool isClose(CheckPoint a, CheckPoint b, double delta = 0.01)
  {
    if(abs(a.x-b.x) < delta && abs(a.y-b.y) < delta)
    {
      return true;
    }
    return false;
  }
  void StageReset(const mainspace::msg::Position::SharedPtr msg)
  {
    auto message = *msg;
    CheckPoint now_position = {message.x, message.y};
    if(isClose(now_position, Stage2_reset) && now_stage != 2) 
    {
      Stage3_->publish(std_msgs::msg::Bool().set__data(false));
      Stage4_->publish(std_msgs::msg::Bool().set__data(false));
      RCLCPP_WARN(this->get_logger(), "Stage 2 reset");
      now_stage = 2; 
    }
    else if(isClose(now_position, Stage3_reset) && now_stage != 3) 
    {
      Stage2_->publish(std_msgs::msg::Bool().set__data(false));
      Stage4_->publish(std_msgs::msg::Bool().set__data(false));
      RCLCPP_WARN(this->get_logger(), "Stage 3 reset");
      now_stage = 3; 
    }
    else if(isClose(now_position, Stage4_reset) && now_stage != 4) 
    {
      Stage2_->publish(std_msgs::msg::Bool().set__data(false));
      Stage3_->publish(std_msgs::msg::Bool().set__data(false));
      RCLCPP_WARN(this->get_logger(), "Stage 4 reset");
      now_stage = 4; 
    }
  }
  void ManagerLoop()
  {
    switch (now_stage) {
      case 1:
        cameraCoffee2_->publish(std_msgs::msg::Bool().set__data(true));
        //end Stage1, start Stage2
        if(isClose(last_position_, Stage1_end)) 
        {
            now_stage = 2; 
        }
        break;
      case 2:
        Stage2_->publish(std_msgs::msg::Bool().set__data(true));

        //open front camera
        if(isClose(last_position_, Stage2_reset) && first_time_coffee) 
        {
            first_time_coffee = false;
            cameraCoffee2_->publish(std_msgs::msg::Bool().set__data(true));
        }
        else if(isClose(last_position_, Stage2_coffee_2) && first_time_coffee_2) 
        {
            first_time_coffee_2 = false;
            command_pub_->publish(mainspace::msg::Command().set__info("S2_1"));
        }
        //open down camera
        else if((isClose(last_position_, Stage2_desk)) && first_time_desk)   
        {
            first_time_desk = false;
            cameraDesk2_->publish(std_msgs::msg::Bool().set__data(true));
        }
        //end Stage2, start Stage3
        else if(isClose(last_position_, Stage2_end))  
        {
            now_stage = 3;
            Stage2_->publish(std_msgs::msg::Bool().set__data(false));
        }
        break;
      case 3:
        Stage3_->publish(std_msgs::msg::Bool().set__data(true));
        //end Stage3, start Stage4
        if(isClose(last_position_, Stage3_end))
        {
            now_stage = 4;
            Stage3_->publish(std_msgs::msg::Bool().set__data(false));
        }
        break;
      case 4:
        Stage4_->publish(std_msgs::msg::Bool().set__data(true));
        //end Stage4, stop all
        if(isClose(last_position_, Stage4_end))
        {
            Stage4_->publish(std_msgs::msg::Bool().set__data(false));
            now_stage = 5;
        }
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown stage: %d", now_stage);
    }
  }

protected:
    rclcpp::TimerBase::SharedPtr control_timer_manager_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Stage2_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cameraCoffee2_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cameraDesk2_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Stage3_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Stage4_;

    rclcpp::Subscription<mainspace::msg::Position>::SharedPtr last_waypoint_sub_;

    rclcpp::Publisher<mainspace::msg::Command>::SharedPtr command_pub_;

    CheckPoint last_position_ = {0.0, 0.0};
    //set checkpoint for each
    CheckPoint Stage1_end = {6.71,1.28}; 
    CheckPoint Stage2_reset = {6.71, 0.45};

    // CheckPoint Stage2_coffee = {7.03, 0.45};
    CheckPoint Stage2_coffee_2 = {7.03, 0.6};
    CheckPoint Stage2_desk = {6.46, 2.09};
    CheckPoint Stage2_end = {6.71,3.95};
    CheckPoint Stage3_reset = {6.71, 3.95};

    CheckPoint Stage3_end = {3.9, 6.2};
    CheckPoint Stage4_reset = {3.9, 6.2};

    CheckPoint Stage4_end = {0.5, 5.91};

    int now_stage = 1;
    //run only once
    bool first_time_coffee = true; 
    bool first_time_coffee_2 = true;
    bool first_time_desk = true; 
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StageManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

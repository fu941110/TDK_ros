#include "mainspace/navigator.hpp"
#include "mainspace/msg/coffee.hpp"
#include "mainspace/msg/desk.hpp"
#include "std_msgs/msg/bool.hpp"
#include "mainspace/msg/command.hpp"
#include <chrono>

using std::placeholders::_1;

class Stage2 : public NavigatorNode
{
public:
  Stage2()
  : NavigatorNode(false)  
  {

    //get coffee and desk info by cameraCoffee2 and cameraDesk2
    coffee_sub_ = this->create_subscription<mainspace::msg::Coffee>(
      "/coffee", 10, std::bind(&Stage2::coffeeRead, this, _1));
    desk_sub_ = this->create_subscription<mainspace::msg::Desk>(
      "/desk", 10, std::bind(&Stage2::deskRead, this, _1));

    //for desk delta x and y
    position_pub_ = this->create_publisher<mainspace::msg::Position>("/stm_position", 10);
    
    //by coffee and desk info, throw csv file to navigator
    csvfile_pub_ = this->create_publisher<mainspace::msg::CsvFile>("/csv_file", 10);

    //publish Bool to control_node, then decide which node will be open or closed
    cameraCoffee2_ = this->create_publisher<std_msgs::msg::Bool>("/cameraCoffee2_start", 10);
    cameraDesk2_ = this->create_publisher<std_msgs::msg::Bool>("/cameraDesk2_start", 10);

    //when pause = true -> v = 0
    pause_pub_ = this->create_publisher<mainspace::msg::Pause>("/pause", 10);

    //command STM -> ROS
    command_sub_ = this->create_subscription<mainspace::msg::Command>(
      "/commandToROS", 10, std::bind(&Stage2::CommandCallBack, this, _1));

    //command ROS -> STM
    //test  改爲“/commandToSTM“
    command_pub_ = this->create_publisher<mainspace::msg::Command>("/commandToSTM", 10);

    //test, 刪///////////////////////////////////////////////////////////////
    // mainspace::msg::CsvFile csv_path;
    // // csv_path.file = "/MainPath_csv/Stage2.csv";
    // csv_path.file = "/Stage2Path_csv/coffee1.csv";
    // csvfile_pub_->publish(csv_path);
  }

private:

  void CommandCallBack(const mainspace::msg::Command::SharedPtr msg)
  {
    //receive takecoffee OK, push new route to navigator and run
    if(msg->info == "Stage2_takecoffee_OK")
    {
      SendRoute();
      pause_pub_->publish(mainspace::msg::Pause().set__pause(false));
    }
    if(msg->info == "Stage2_putdowncoffee_OK")
    {
      pause_pub_->publish(mainspace::msg::Pause().set__pause(false));
    }
  }
  void SendRoute()
  {
    mainspace::msg::CsvFile csv_path;
    if(number == 1)
    {
      csv_path.file = "/Stage2Path_csv/coffee1.csv";
    }
    else if(number == 2)
    {
      csv_path.file = "/Stage2Path_csv/coffee2.csv";
    }
    else if(number == 3)
    {
      csv_path.file = "/Stage2Path_csv/coffee3.csv";
    }  
    else if(number == 4)
    {
      csv_path.file = "/Stage2Path_csv/coffee4.csv";
    }

    csvfile_pub_->publish(csv_path);
  }

  void coffeeRead(const mainspace::msg::Coffee::SharedPtr msg)
  {
    type = msg->type;
    number = msg->number;

    mainspace::msg::Position position_msg;
    if(position_msg.y != 0.0)  //only once
    {
      return;
    }
    else if(type == "black")
    {
      position_msg.x = 0.0;
      position_msg.y = -0.1;
      position_msg.theta = 9999;
    } 
    else if(type == "white")
    {
      position_msg.x = 0.0;
      position_msg.y = 0.1;
      position_msg.theta = 9999;
    }
    else
    {
      return;
    }
    position_pub_->publish(position_msg);
    
    //to STM, take coffee
    command_pub_->publish(mainspace::msg::Command().set__info("Stage2_takecoffee"));

    //test, 寫在STM，/////////////////////////////////////////////////////////////////
    // command_pub_->publish(mainspace::msg::Command().set__info("Stage2_takecoffee_OK"));

    //destory cameraCoffee2 node
    cameraCoffee2_->publish(std_msgs::msg::Bool().set__data(false));
  }

  bool RangeCheck(double x, double k)
  {
    if(x < k+21 && x > k-21) 
    {
      return true; // 在範圍內
    }
    return false; // 不在範圍內
  }

  void deskRead(const mainspace::msg::Desk::SharedPtr msg)
  {
    double x = msg->x;
    double y = msg->y;

    //can put down coffee
    if(RangeCheck(x, 160) && RangeCheck(y,  0)) 
    {
      //stop robot
      pause_pub_->publish(mainspace::msg::Pause().set__pause(true));

      mainspace::msg::CsvFile csv_path;
      if(number == 1)
      {
        csv_path.file = "/Stage2Path_csv/desk1.csv";
        csvfile_pub_->publish(csv_path);
      }
      else if(number == 2)
      {
        csv_path.file = "/Stage2Path_csv/desk2.csv";
        csvfile_pub_->publish(csv_path);
      }
      else if(number == 3)
      {
        csv_path.file = "/Stage2Path_csv/desk3.csv";
        csvfile_pub_->publish(csv_path);
      }
      else if(number == 4)
      {
        csv_path.file = "/Stage2Path_csv/desk4.csv";
        csvfile_pub_->publish(csv_path);
      }

      //to STM, put down coffee
      command_pub_->publish(mainspace::msg::Command().set__info("Stage2_putdowncoffee"));

      //test, 寫在STM，/////////////////////////////////////////////////////////////////
      // command_pub_->publish(mainspace::msg::Command().set__info("Stage2_putdowncoffee_OK"));

      //destory cameraDesk2 node
      cameraDesk2_->publish(std_msgs::msg::Bool().set__data(false));
    } 
    //adjust position
    else 
    {
      double px = 0.0005;
      double py = 0.0005;

      double dx = -y*py;
      double dy = -(x-160)*px;

      if(number == 2 || number == 4)
      {
        dx = -dx;
        dy = -dy;
      }

      mainspace::msg::Position position_msg;
      position_msg.x = dx;
      position_msg.y = dy;
      position_msg.theta = 9999;
      // position_pub_->publish(position_msg);
    }
  }

protected:
    rclcpp::Subscription<mainspace::msg::Coffee>::SharedPtr coffee_sub_;
    rclcpp::Subscription<mainspace::msg::Desk>::SharedPtr desk_sub_;
    rclcpp::Subscription<mainspace::msg::Command>::SharedPtr command_sub_;
    rclcpp::Publisher<mainspace::msg::Command>::SharedPtr command_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cameraCoffee2_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cameraDesk2_;
    rclcpp::Publisher<mainspace::msg::Position>::SharedPtr position_pub_;
    rclcpp::Publisher<mainspace::msg::CsvFile>::SharedPtr csvfile_pub_;
    rclcpp::Publisher<mainspace::msg::Pause>::SharedPtr pause_pub_;

    //coffee and desk info
    std::string type;
    int number = 1;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Stage2>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
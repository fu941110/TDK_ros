#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "mainspace/msg/coffee.hpp"
#include "mainspace/src/desk.cpp"
#include "mainspace/src/navigator.cpp"
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

struct WayPoint {
  double x;
  double y;
  double theta;
  int mode;  //"straight", "rotate", "arc", "shift", "mix"
  double radius; // for arc
};

class Stage2 : public NavigatorNode
{
public:
  NavigatorNode()
  : Node("Stage2")  
  {
    using std::placeholders::_1;

    coffee_sub_ = this->create_subscription<mainspace::msg::Coffee>(
      "/coffee", 10, std::bind(&Stage2::coffeeRead, this, _1));

    desk_sub_ = this->create_subscription<mainspace::msg::Desk>(
      "/desk", 10, std::bind(&Stage2::deskRead, this, _1));

    cameraStage2_ = this->create_publisher<std_msgs::msg::Bool>("/cameraStage2_start", 10);
    cameraDesk2_ = this->create_publisher<std_msgs::msg::Bool>("/cameraDesk2_start", 10);
  }

private:
  void coffeeRead(const mainspace::msg::Position::SharedPtr msg)
  {
    std::string type = msg->type;
    int number = msg->number;
    if(type == "black")
    {

    } 
    else if(type == "white")
    {

    }

    if(number == 1)
    {
        std::string csv_path = ament_index_cpp::get_package_share_directory("mainspace") + "/Stage2Path_csv/coffee1.csv";
        loadWayPoints(csv_path);
    }
    else if(number == 2)
    {
        std::string csv_path = ament_index_cpp::get_package_share_directory("mainspace") + "/Stage2Path_csv/coffee2.csv";
        loadWayPoints(csv_path);
    }
    else if(number == 3)
    {
        std::string csv_path = ament_index_cpp::get_package_share_directory("mainspace") + "/Stage2Path_csv/coffee3.csv";
        loadWayPoints(csv_path);
    }  
    else if(number == 4)
    {
        std::string csv_path = ament_index_cpp::get_package_share_directory("mainspace") + "/Stage2Path_csv/coffee4.csv";
        loadWayPoints(csv_path);
    }
    cameraStage2_->publish(std_msgs::msg::Bool{false});
  }

  void deskRead(const mainspace::msg::Desk::SharedPtr msg)
  {
    double x = msg->x;
    double y = msg->y;

    if(x < 21 && x > -21 && y < -79 && y > -121) {
        //發送給STM
        cameraDesk2_->publish(std_msgs::msg::Bool{false});
    } else {
        //發送給STM

    }
  }

protected:
    rclcpp::Subscription<mainspace::msg::Coffee>::SharedPtr coffee_sub_;
    rclcpp::Subscription<mainspace::msg::Desk>::SharedPtr desk_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cameraStage2_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cameraDesk2_;
}
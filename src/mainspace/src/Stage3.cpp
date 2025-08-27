#include "mainspace/navigator.hpp"
#include "std_msgs/msg/bool.hpp"
#include <chrono>

class Stage3 : public NavigatorNode
{
public:
  Stage3()
  : NavigatorNode(false)  
  {
    using std::placeholders::_1;
    using namespace std::chrono_literals;
    
    //by coffee and desk info, throw csv file to navigator
    csvfile_pub_ = this->create_publisher<mainspace::msg::CsvFile>("/csv_file", 10);


    //test///////////////////////////////////////////////////////////////
    mainspace::msg::CsvFile csv_path;
    csv_path.file = "/MainPath_csv/Stage3.csv";
      
    for(int i=0; i<10; i++) 
    {
      rclcpp::sleep_for(30ms);
      csvfile_pub_->publish(csv_path);
    }
  }

private:
  

protected:
    rclcpp::Publisher<mainspace::msg::CsvFile>::SharedPtr csvfile_pub_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Stage3>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
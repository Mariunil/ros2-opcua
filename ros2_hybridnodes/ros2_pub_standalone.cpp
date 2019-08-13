#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"


using namespace std::chrono_literals;


int main(int argc, char* argv[]){


  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_publisher");
  auto publisher = node->create_publisher<std_msgs::msg::Float64>("topic", 10);
  std_msgs::msg::Float64 message;
  auto publish_count = 0;
  rclcpp::WallRate loop_rate(500ms);

  while (rclcpp::ok()) {
    // fake sensor data
    double randDec1 = (rand() % 19 + (-9))/10.0;
    double randDec2 = (rand() % 19 + (-9))/100.0;
    double randDec3 = (rand() % 19 + (-9))/1000.0;
    message.data = 25+randDec1+randDec2+randDec3;

    RCLCPP_INFO(node->get_logger(), "[ROS2] Publishing: '%f'", message.data);
    publisher->publish(message);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}



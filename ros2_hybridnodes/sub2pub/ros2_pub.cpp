#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"


using namespace std::chrono_literals;


int main(int argc, char* argv[]){


  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_publisher");
  auto publisher = node->create_publisher<std_msgs::msg::Int32>("topic", 10);
  std_msgs::msg::Int32 message;
  auto publish_count = 0;
  rclcpp::WallRate loop_rate(500ms);

  while (rclcpp::ok()) {
    message.data = publish_count++;
    RCLCPP_INFO(node->get_logger(), "Publishing: '%i'", message.data);
    publisher->publish(message);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}




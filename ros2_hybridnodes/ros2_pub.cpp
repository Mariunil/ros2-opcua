#include "ros2_pub.h"

using namespace std::chrono_literals;


/* This style is chosen for our hybrid-nodes due to their simplicity. Object-oriented
   structuring with a node class is preferable */


void* ros2PublisherRun(void* ptr){

  int argc{1};
  std::string ve = "ros2_pub";
  const char* v  = ve.c_str();

  rclcpp::init(argc, &v);
  auto node = rclcpp::Node::make_shared("minimal_publisher");
  auto publisher = node->create_publisher<std_msgs::msg::Float64>("topic", 10);
  std_msgs::msg::Float64 message;
  auto publish_count = 0;
  rclcpp::WallRate loop_rate(500ms);

  while (rclcpp::ok()) {
    message.data = globalVal;
    RCLCPP_INFO(node->get_logger(), "[ROS2] Publishing: '%f', recieved from OPC UA Subscriber", 
                message.data);
    publisher->publish(message);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }


  rclcpp::shutdown();
  return 0;
}

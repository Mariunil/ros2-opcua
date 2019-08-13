#include <pthread.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

rclcpp::Node::SharedPtr g_node = nullptr;

void topic_callback(const std_msgs::msg::Float64::SharedPtr msg){

    RCLCPP_INFO(g_node->get_logger(), "I heard: '%f'", msg->data);
}


int main(int argc, char* argv[]){

    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("minimal_subscriber");
    auto subscription =
        g_node->create_subscription<std_msgs::msg::Float64>("topic", 10, topic_callback);
    rclcpp::spin(g_node);
    rclcpp::shutdown();

    subscription = nullptr;
    g_node = nullptr;

}


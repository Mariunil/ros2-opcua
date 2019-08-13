#include "ros2_sub.h"



rclcpp::Node::SharedPtr g_node = nullptr;

void topic_callback(const std_msgs::msg::Float64::SharedPtr msg){

    RCLCPP_INFO(g_node->get_logger(), "I heard: '%f'", msg->data);
    globalVal = msg->data;
}


void* ros2SubscriberRun(void* ptr){

    int argc{1};
    std::string ve = "ros2_sub";
    const char* v  = ve.c_str();
    rclcpp::init(argc, &v );
    g_node = rclcpp::Node::make_shared("minimal_subscriber");
    auto subscription =
        g_node->create_subscription<std_msgs::msg::Float64>("topic", 10, topic_callback);
    rclcpp::spin(g_node);
    rclcpp::shutdown();

    subscription = nullptr;
    g_node = nullptr;

}


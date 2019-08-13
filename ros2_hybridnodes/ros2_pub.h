#ifndef ROS2_PUB_H
#define ROS2_PUB_H

#include "opcuaSub_ros2Pub_node.h"

#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"


void* ros2PublisherRun(void* ptr);



#endif //header-guard
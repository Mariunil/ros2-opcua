#ifndef ROS2_SUB_H
#define ROS2_SUB_H


#include "ros2Sub_opcuaPub_node.h"

#include <pthread.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"



void* ros2SubscriberRun(void* ptr);


#endif // header-guard
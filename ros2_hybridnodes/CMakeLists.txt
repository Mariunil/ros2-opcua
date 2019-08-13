cmake_minimum_required(VERSION 3.10) # setting this is required

project(hyb) # this sets the project name

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#ROS2 sub and OPC UA Pub hybrid
add_executable(rsub_uapub
               ros2Sub_opcuaPub_node.cpp opcua_pub.c ros2_sub.cpp open62541.c )
ament_target_dependencies(rsub_uapub rclcpp std_msgs)

#OPC UA sub and ROS2 pub hybrid
add_executable(uasub_rpub
	       opcuaSub_ros2Pub_node.cpp opcua_sub.c ros2_pub.cpp open62541.c )
ament_target_dependencies(uasub_rpub rclcpp std_msgs)

#standard ROS2 publisher
add_executable(pub ros2_pub_standalone.cpp)
ament_target_dependencies(pub rclcpp std_msgs)

#standard ROS2 subscriber
add_executable(sub ros2_sub_standalone.cpp)
ament_target_dependencies(sub rclcpp std_msgs)


install(TARGETS
        sub
	pub
	rsub_uapub
	uasub_rpub
  	DESTINATION lib/${PROJECT_NAME}
)

ament_package()

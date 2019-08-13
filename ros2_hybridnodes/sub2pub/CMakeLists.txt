cmake_minimum_required(VERSION 3.10) # setting this is required

project(sub2pub) # this sets the project name

#set(CMAKE_C_COMPILER "gcc")
#set(CMAKE_C_FLAGS "-std=c99") 

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(hyb_sub2pub ros2Sub_opcuaPub_node.cpp opcua_pub.c ros2_sub.cpp open62541.c)
ament_target_dependencies(hyb_sub2pub rclcpp std_msgs)

add_executable(pub ros2_pub.cpp)
ament_target_dependencies(pub rclcpp std_msgs)

install(TARGETS
        hyb_sub2pub
	pub
  	DESTINATION lib/${PROJECT_NAME}
)

ament_package()

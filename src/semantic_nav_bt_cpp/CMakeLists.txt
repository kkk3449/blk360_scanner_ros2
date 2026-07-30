cmake_minimum_required(VERSION 3.8)
project(semantic_nav_bt_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(semantic_nav_msgs REQUIRED)

add_executable(mission_bt_cpp src/mission_bt_cpp.cpp)
ament_target_dependencies(mission_bt_cpp
  ament_index_cpp rclcpp rclcpp_action behaviortree_cpp nav2_msgs
  geometry_msgs sensor_msgs std_msgs semantic_nav_msgs)

install(TARGETS mission_bt_cpp DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY trees DESTINATION share/${PROJECT_NAME})

ament_package()

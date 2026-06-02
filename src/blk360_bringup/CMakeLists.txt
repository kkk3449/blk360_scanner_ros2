cmake_minimum_required(VERSION 3.8)
project(blk360_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz maps worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

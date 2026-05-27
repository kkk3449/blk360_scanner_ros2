cmake_minimum_required(VERSION 3.12)
project(blk360_scanner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Prebuilt, vendored BLK360 client library (x86-64 Linux only).
add_library(BLK360 SHARED IMPORTED)
set_target_properties(BLK360 PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/third_party/lib/libBLK360.so"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/third_party/include")

add_executable(scan_node src/scan_node.cpp)
target_include_directories(scan_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/include)
target_link_libraries(scan_node BLK360)
ament_target_dependencies(scan_node rclcpp std_msgs)

# Keep the vendored .so next to the installed executable and resolve it via $ORIGIN,
# so the package runs after a plain `git clone` + `colcon build` without touching paths.
set_target_properties(scan_node PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/lib"
  INSTALL_RPATH "$ORIGIN")

install(TARGETS scan_node DESTINATION lib/${PROJECT_NAME})
install(FILES third_party/lib/libBLK360.so DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()

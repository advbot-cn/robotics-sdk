cmake_minimum_required(VERSION 3.8)
project(adv_gmsl_camera_baseline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node rclcpp sensor_msgs)

add_executable(benchmark_subscriber src/benchmark_subscriber.cpp)
ament_target_dependencies(benchmark_subscriber rclcpp sensor_msgs)

install(TARGETS
  camera_node
  benchmark_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

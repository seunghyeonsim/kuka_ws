cmake_minimum_required(VERSION 3.8)
project(kuka_control_box)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# include 디렉토리를 포함
include_directories(include)

# Robotics 라이브러리 추가
add_library(Robotics src/Robotics.cpp)
ament_target_dependencies(Robotics rclcpp Eigen3)

add_executable(trajectory_publisher src/trajectory_publisher.cpp)
ament_target_dependencies(trajectory_publisher rclcpp trajectory_msgs)

add_executable(kuka_ik_node src/kuka_ik_node.cpp)
target_link_libraries(kuka_ik_node Robotics)
ament_target_dependencies(kuka_ik_node rclcpp Eigen3)

install(TARGETS
  trajectory_publisher
  kuka_ik_node
  Robotics
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

ament_package()

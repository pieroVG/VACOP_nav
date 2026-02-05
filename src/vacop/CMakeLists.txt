cmake_minimum_required(VERSION 3.5)

project(vacop)

find_package(ament_cmake REQUIRED)

find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
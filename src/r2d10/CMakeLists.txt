cmake_minimum_required(VERSION 3.8)
project(r2d10)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros_gz_sim REQUIRED)


install(
  DIRECTORY 
  meshes
  urdf
  rviz
  worlds
  launch 
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/odom_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

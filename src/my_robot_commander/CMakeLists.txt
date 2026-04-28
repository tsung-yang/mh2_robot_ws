cmake_minimum_required(VERSION 3.8)
project(my_robot_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED) 
find_package(std_srvs REQUIRED)

add_executable(test_moveit src/test_moveit.cpp)
add_executable(arm_commander src/commander_ros_node.cpp)
ament_target_dependencies(test_moveit rclcpp moveit_ros_planning_interface)
ament_target_dependencies(arm_commander rclcpp moveit_ros_planning_interface moveit_core std_srvs)

install(
  TARGETS test_moveit arm_commander
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

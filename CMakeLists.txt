cmake_minimum_required(VERSION 3.8)
project(liquid_pickup)

set(CMAKE_BUILD_TYPE Debug)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
# find_package(gazebo_ros REQUIRED)
# find_package(OGRE REQUIRED)
# find_package(moveit_task_constructor_core REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

include_directories(include config) 

set(BT_SOURCES
  src/manipulator.cpp
  src/manipulator_behaviors.cpp
  src/gripper_behavior.cpp
  src/robot.cpp
  src/helper.cpp
  src/time_logger.cpp
  src/navigation_behaviors.cpp
  src/sensors_deploy_behaviors.cpp
  # src/move_it_task_constructor_behaviors.cpp
)

add_executable(liquid_pickup_node src/bt_node.cpp ${BT_SOURCES})
ament_target_dependencies(liquid_pickup_node rclcpp behaviortree_cpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs yaml-cpp moveit_ros_planning_interface rclcpp_action rclcpp_components moveit_msgs control_msgs moveit_visual_tools nav2_msgs gazebo_msgs tf2_eigen std_msgs nav2_costmap_2d) # gazebo_ros OGRE moveit_task_constructor_core)

target_link_libraries(liquid_pickup_node yaml-cpp)

install(TARGETS
    liquid_pickup_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch 
  config
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

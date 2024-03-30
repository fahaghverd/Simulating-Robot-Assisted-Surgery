cmake_minimum_required(VERSION 3.0.2)
project(moveit_playground)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  interactive_markers
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  rviz_visual_tools
  moveit_visual_tools
  geometric_shapes
  tf2_ros
  tf2_eigen
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem date_time thread)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS}
)

add_executable(wam_sim_node src/wam_sim_node.cpp)
target_link_libraries(wam_sim_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


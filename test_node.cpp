/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();
 
  ros::Publisher pub = node_handle.advertise<moveit_msgs::RobotTrajectory>("trjbuffer", 1); //trajectiry published here

  static const std::string PLANNING_GROUP = "wam_arm";

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("wam/base_link");
  visual_tools.deleteAllMarkers();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;

  visual_tools.trigger();

  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the simulation");

 
 // Planning to a Pose goal

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 0.024396;
  target_pose1.orientation.x = 0.018702;
  target_pose1.orientation.y = 0.99866;
  target_pose1.orientation.z = 0.041691;
  /*target_pose1.orientation.w = 0.21232;
  target_pose1.orientation.x = 0.026261;
  target_pose1.orientation.y = 0.97613;
  target_pose1.orientation.z = 0.037377;*/
  target_pose1.position.x = 0.72887076;
  target_pose1.position.y = -0.02036049;
  target_pose1.position.z = 0.4;
  move_group_interface.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group_interface
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to publish the trajectory for Ammat");

  pub.publish(my_plan.trajectory_);
  // j["list"] = my_plan.trajectory_;


  ros::shutdown();
  return 0;
}
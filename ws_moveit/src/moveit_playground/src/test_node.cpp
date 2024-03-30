#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

const double TAU = 2 * M_PI;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wam_sim_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher pub = nh.advertise<moveit_msgs::RobotTrajectory>("trjbuffer", 1);

    static const std::string PLANNING_GROUP = "wam_arm";

    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("wam/base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    ROS_INFO("Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    move_group_interface.setEndEffectorLink("wam/palm_yaw_joint");
    ROS_INFO("End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

    ROS_INFO("Available Planning Groups:");
    std::copy(move_group_interface.getJointModelGroupNames().begin(),
              move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    // Visualize the setup
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "Needle Insertion Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // Planning for Plan 1
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 0.024396;
    target_pose1.orientation.x = 0.018702;
    target_pose1.orientation.y = 0.99866;
    target_pose1.orientation.z = 0.041691;
    target_pose1.position.x = 0.72887076;
    target_pose1.position.y = -0.02036049;
    target_pose1.position.z = 0.4;
    move_group_interface.setPoseTarget(target_pose1);

    //move_group_interface.setPlannerId("RRT");
    move_group_interface.setPlanningTime(30.0); // Set 10 seconds as the planning time


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Planning for Plan 2 - Set up with a different planner or parameters if needed
    move_group_interface.setPlannerId("RRTstar");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
    success = (move_group_interface.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 2 (pose goal) %s", success ? "" : "FAILED");

    // Visualization of both Plan 1 and Plan 2
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Needle Insertion Plans", rvt::WHITE, rvt::XLARGE);

    // Plan 1
    visual_tools.publishAxisLabeled(target_pose1, "Plan 1");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group, rvt::GREEN);

    // Plan 2
    visual_tools.publishAxisLabeled(target_pose1, "Plan 2", rvt::RED); // Adjust label position if needed
    visual_tools.publishTrajectoryLine(my_plan2.trajectory_, joint_model_group, rvt::BLUE);

    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to publish the trajectories.");

    // Publish the trajectories
    pub.publish(my_plan.trajectory_);
    pub.publish(my_plan2.trajectory_);

    ros::shutdown();
    return 0;
}

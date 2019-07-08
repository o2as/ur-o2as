#include "ros/ros.h"
#include "o2as_helper_functions.h"
#include <tf/transform_listener.h>    // Includes the TF conversions
#include <moveit/move_group_interface/move_group_interface.h>

// This example moves two robots to different points in the scene.

int main (int argc, char **argv) {
  
  // Initialize ROS
  ros::init(argc, argv, "CommandRobotMoveit");
  ros::NodeHandle nh("~");
  
  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  tf::TransformListener tflistener;
  
  std::string movegroup_name, ee_link_a, ee_link_b;
  
  // Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
  nh.param<std::string>("move_group", movegroup_name, "front_bots");
  nh.param<std::string>("ee_link1", ee_link_a, "b_bot_robotiq_85_tip_link");
  nh.param<std::string>("ee_link2", ee_link_b, "a_bot_robotiq_85_tip_link");
  
  // Dynamic parameter to choose the rate at which this node should run
  double ros_rate;
  nh.param("ros_rate", ros_rate, 0.2); // 0.2 Hz = 5 seconds
  ros::Rate* loop_rate_ = new ros::Rate(ros_rate);
    
  int bin = 1;
  std::string bin_header1, bin_header2;

  ros::Duration(1.0).sleep(); // 1 second
  
  // Create MoveGroup
  moveit::planning_interface::MoveGroupInterface group(movegroup_name);
  moveit::planning_interface::MoveGroupInterface::Plan myplan;
      
  // Configure planner 
  group.setPlanningTime(0.5);
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setEndEffectorLink(ee_link_a);
  moveit::planning_interface::MoveItErrorCode success_plan = moveit_msgs::MoveItErrorCodes::FAILURE, 
  motion_done = moveit_msgs::MoveItErrorCodes::FAILURE;

  ros::Duration(2.0).sleep(); // 2 seconds
  while (ros::ok()) {
    if (true) {
      
      if (bin == 1) {bin_header1 = "/set2_bin2_1"; bin_header2 = "/set2_bin2_2"; }
      if (bin == 2) {bin_header1 = "/set2_bin2_2"; bin_header2 = "/set2_bin2_3"; }

      geometry_msgs::PoseStamped ps_b, ps_a;
      ps_a.pose = makePose(0, 0, .02);
      // The lines below rotate the pose so the robot looks downwards.
      rotatePoseByRPY(0.0, 0.0, M_PI/2.0, ps_a.pose);
      rotatePoseByRPY(0.0, M_PI/2.0, 0.0, ps_a.pose);
      ps_b.pose = ps_a.pose;
      
      ps_a.header.frame_id = bin_header2;
      ps_b.header.frame_id = bin_header1;
      
      // ROS_INFO_STREAM("Moving to pose: " << ps.pose.position.x << ", " << ps.pose.position.y << ", " << ps.pose.position.z
      //                 << "; " << ps.pose.orientation.x << ", " << ps.pose.orientation.y << ", " << ps.pose.orientation.z << ", " << ps.pose.orientation.w);

      group.setStartStateToCurrentState();
      group.setPoseTarget(ps_a, ee_link_a);
      group.setPoseTarget(ps_b, ee_link_b);
      success_plan = group.plan(myplan);
      if (success_plan == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        motion_done = group.execute(myplan);
      }
      if (motion_done) {
        loop_rate_->sleep(); // Sleep for some millisecond. The while loop will run every 10 seconds in this example.
      }
      bin++;
      if (bin > 3)
      {
        bin = 1;
      }
    }
  }  
}; 

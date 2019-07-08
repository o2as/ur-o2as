#include "ros/ros.h"
#include "o2as_helper_functions.h"
#include <tf/transform_listener.h>    // Includes the TF conversions
#include <moveit/move_group_interface/move_group_interface.h>

// This example moves one of three robots to a point in the scene in turn.

int main (int argc, char **argv) {
  
  // Initialize ROS
  ros::init(argc, argv, "CommandRobotMoveit");
  ros::NodeHandle nh("~");
  
  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  tf::TransformListener tflistener;
  
  std::string movegroup_name, ee_link_a, ee_link_b, ee_link_c;
  
  // Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
  nh.param<std::string>("move_group", movegroup_name, "all_bots");
  nh.param<std::string>("ee_link1", ee_link_a, "a_bot_robotiq_85_tip_link");
  nh.param<std::string>("ee_link2", ee_link_b, "b_bot_robotiq_85_tip_link");
  nh.param<std::string>("ee_link3", ee_link_c, "c_bot_robotiq_85_tip_link");
  // Dynamic parameter to choose the rate at which this node should run
  double ros_rate;
  nh.param("ros_rate", ros_rate, 0.2); // 0.2 Hz = 5 seconds
  ros::Rate* loop_rate_ = new ros::Rate(ros_rate);
    
  int counter = 1;
  std::string bin_header;

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

  // Send the robots to the home position, if they are not there yet.
  group.setStartStateToCurrentState();
  group.setNamedTarget("home_all");
  success_plan = group.plan(myplan);
  if (success_plan == moveit_msgs::MoveItErrorCodes::SUCCESS) {
    motion_done = group.execute(myplan);
  }
  else 
  {
    ROS_WARN("Something went wrong moving the robots to home position.");
  }

  ros::Duration(2.0).sleep(); // 2 seconds

  geometry_msgs::PoseStamped home1, home2,home3;
  home1.pose.orientation.w = 1.0;
  home2.pose.orientation.w = 1.0;
  home3.pose.orientation.w = 1.0;
  home1.header.frame_id="a_bot_robotiq_85_tip_link";
  home2.header.frame_id="b_bot_robotiq_85_tip_link";
  home3.header.frame_id="c_bot_robotiq_85_tip_link";
  home1 = transform_pose_now(home1, "/world", tflistener);
  home2 = transform_pose_now(home2, "/world", tflistener);
  home3 = transform_pose_now(home3, "/world", tflistener);
  
  while (ros::ok()) {
    if (true) {
      bin_header = "/set3_tray_2"; 

      geometry_msgs::PoseStamped ps;
      ps.pose = makePose(0, 0, .03);
      // The lines below rotate the pose so the robot looks downwards.
      rotatePoseByRPY(0.0, 0.0, M_PI/2.0, ps.pose);
      rotatePoseByRPY(0.0, M_PI/2.0, 0.0, ps.pose);

      ps.header.frame_id = bin_header;

      // ROS_INFO_STREAM("Moving to pose: " << ps.pose.position.x << ", " << ps.pose.position.y << ", " << ps.pose.position.z
      //                 << "; " << ps.pose.orientation.x << ", " << ps.pose.orientation.y << ", " << ps.pose.orientation.z << ", " << ps.pose.orientation.w);

      group.setStartStateToCurrentState();
      if(counter==1){group.setPoseTarget(ps,    ee_link_a);  group.setPoseTarget(home2, ee_link_b);  group.setPoseTarget(home3, ee_link_c);}
      if(counter==2){group.setPoseTarget(home1, ee_link_a);  group.setPoseTarget(ps,    ee_link_b);  group.setPoseTarget(home3, ee_link_c);}
      if(counter==3){group.setPoseTarget(home1, ee_link_a);  group.setPoseTarget(home2, ee_link_b);  group.setPoseTarget(ps,    ee_link_c);}
      
      success_plan = group.plan(myplan);
      if (success_plan == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        motion_done = group.execute(myplan);
      }
      if (motion_done) {
        loop_rate_->sleep(); // Sleep for some milliseconds. The while loop will run every 5 seconds in this example.
      }
      counter++;
      if (counter > 3)
      {
        counter = 1;
      }
    }
  }  
}; 

#!/usr/bin/env python

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from moveit_commander.conversions import pose_to_list
import sys
import rospy

def main():
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('2_bot_kitting_moveit',
                    anonymous=True)
  robot = moveit_commander.RobotCommander()
  group_name = rospy.get_param("move_group_name", "a_bot")
  rospy.loginfo(group_name)
  ee_link1 = rospy.get_param("ee_link", "a_bot_robotiq_85_tip_link")
  ee_link2 = rospy.get_param("ee_link", "b_bot_robotiq_85_tip_link")
  rospy.loginfo(ee_link1)
  rospy.loginfo(ee_link2)
  group = moveit_commander.MoveGroupCommander(group_name)
  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
  planning_frame = group.get_planning_frame()
  print "============ Reference frame: %s" % planning_frame
  eef_link = group.get_end_effector_link()
  print "============ End effector: %s" % eef_link
  group_names = robot.get_group_names()
  print "============ Robot Groups:", robot.get_group_names()
  print "============ Printing robot state"
  print robot.get_current_state()
  print ""

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    exit(1)
  except KeyboardInterrupt:
    exit(0)

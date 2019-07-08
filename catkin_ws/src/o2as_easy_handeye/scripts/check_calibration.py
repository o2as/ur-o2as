#!/usr/bin/env python

import sys
import os
import copy
import rospy

import moveit_msgs.msg
import geometry_msgs.msg
import tf_conversions

from o2as_aruco_ros.msg import Corners
from std_srvs.srv import Trigger

from o2as_routines.base import O2ASBaseRoutines
from math import radians, degrees

######################################################################
#  class VisitRoutines                                               #
######################################################################
class VisitRoutines(O2ASBaseRoutines):
  """Wrapper of MoveGroupCommander specific for this script"""
  def __init__(self, camera_name, robot_name):
    super(VisitRoutines, self).__init__()
    
    cs = "/{}/".format(camera_name)
    self.start_acquisition = rospy.ServiceProxy(cs + "start_acquisition",
                                                Trigger)
    self.stop_acquisition  = rospy.ServiceProxy(cs + "stop_acquisition",
                                                Trigger)

    ## Initialize `moveit_commander`
    self.group_name = robot_name
    group = self.groups[self.group_name]

    # Set `_ee_link` as end effector wrt `_base_link` of the robot
    group.set_pose_reference_frame("workspace_center")
    #group.set_end_effector_link(robot_name + "_gripper_tip_link")
    group.set_end_effector_link(robot_name + "_dual_suction_gripper_pad_link")

    # Trajectory publisher
    display_trajectory_publisher = rospy.Publisher(
      '/move_group/display_planned_path',
      moveit_msgs.msg.DisplayTrajectory,
      queue_size=20
    )

    # Logging
    print("============ Reference frame: %s" % group.get_planning_frame())
    print("============ End effector: %s"    % group.get_end_effector_link())

    
  def move(self, speed):
    self.start_acquisition()
    position = rospy.wait_for_message("/aruco_tracker/position",
                                      geometry_msgs.msg.Vector3Stamped, 10)
    self.stop_acquisition()

    print("move to {}".format(position.vector))
    group = self.groups[self.group_name]
    poseStamped = geometry_msgs.msg.PoseStamped()
    poseStamped.header.frame_id = group.get_pose_reference_frame()
    poseStamped.pose.position.x = position.vector.x
    poseStamped.pose.position.y = position.vector.y
    poseStamped.pose.position.z = position.vector.z + 0.05 # - 0.0285
    poseStamped.pose.orientation \
      = geometry_msgs.msg.Quaternion(
        *tf_conversions.transformations.quaternion_from_euler(
          radians(180), radians(90), radians(90)))
    [all_close, move_success] \
        = self.go_to_pose_goal(self.group_name, poseStamped, speed,
                               end_effector_link=group.get_end_effector_link(),
                               move_lin=False)
    rospy.sleep(1)
    poseStamped.pose.position.z = position.vector.z # -0.0285
    [all_close, move_success] \
        = self.go_to_pose_goal(self.group_name, poseStamped, speed,
                               end_effector_link=group.get_end_effector_link(),
                               move_lin=False)
        

  def go_home(self):
    self.go_to_named_pose("home", self.group_name)


  def run(self, speed):
    self.stop_acquisition()
    self.go_home()

    while True:
      try:
        key = raw_input(">> ")
        if key == 'q':
          break
        self.move(speed)
      except Exception as ex:
        print ex.message
      except rospy.ROSInterruptException:
        return
      except KeyboardInterrupt:
        return

    self.go_home()

    
######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
  camera_name = sys.argv[1]
  robot_name  = sys.argv[2]
    
  assert(camera_name in {"a_phoxi_m_camera", "a_bot_camera"})
  assert(robot_name  in {"a_bot", "b_bot", "c_bot"})

  routines = VisitRoutines(camera_name, robot_name)
  speed = 0.05
  routines.run(speed)


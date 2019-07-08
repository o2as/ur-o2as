#!/usr/bin/env python

import sys
import copy
import rospy
import geometry_msgs.msg
import tf_conversions
import tf
from math import pi, radians, degrees

from o2as_msgs.srv import *
import actionlib
import o2as_msgs.msg

from o2as_routines.base import O2ASBaseRoutines

poses = [
      # [-0.35, 0.487, 0.3, radians(-110), radians(  0), radians(90)],
    [ 0.0, 0.5, 0.2, radians(0), radians(90),  radians(0)],
    [-0.35, 0.487, 0.3, radians(-110), radians(  0), radians(90)],
    [-0.35, 0.487, 0.3, radians(-110), radians(-30), radians(90)],
    [-0.35, 0.487, 0.3, radians(-140), radians(-30), radians(90)],
    [-0.35, 0.487, 0.3, radians(-140), radians(  0), radians(90)],
    [-0.35, 0.487, 0.3, radians(- 80), radians(  0), radians(90)],
    [-0.35, 0.487, 0.3, radians(- 80), radians(-30), radians(90)],
#    [-0.3, 0.6, 0.5, radians(-110), radians(30),  radians(0)]
]

if __name__ == '__main__':

    robot_name   = "b_bot"
    baseRoutines = O2ASBaseRoutines()
    baseRoutines.go_to_named_pose("home", robot_name)

    baseRoutines.groups[robot_name].set_end_effector_link(robot_name + '_ee_link')
    # position 1
    for i, pose in enumerate(poses):
        print("Motion [{}/{}]: move to {}".format(i+1, len(poses), pose))

        poseStamped = geometry_msgs.msg.PoseStamped()
        poseStamped.header.frame_id  = robot_name + "_base_link"
        poseStamped.pose.position.x  = pose[0]
        poseStamped.pose.position.y  = pose[1]
        poseStamped.pose.position.z  = pose[2]
        poseStamped.pose.orientation = \
           geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pose[3], pose[4], pose[5]))

        baseRoutines.go_to_pose_goal(robot_name, poseStamped,
                                     move_lin=True, speed=1)

        rospy.sleep(3)

    baseRoutines.go_to_named_pose("home", robot_name)

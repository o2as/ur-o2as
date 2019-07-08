#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def get_rotation (pose):
    orientation_q = pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    return euler_from_quaternion (orientation_list)

def get_quaternion (roll, pitch, yaw):
  quaternion = quaternion_from_euler(roll, pitch, yaw)
  pose = Pose()
  pose.orientation.x = quaternion[0]
  pose.orientation.y = quaternion[1]
  pose.orientation.z = quaternion[2]
  pose.orientation.w = quaternion[3]
  return pose

rospy.init_node('my_quaternion_to_euler')

pose = get_quaternion(0,0,np.deg2rad(30))
(roll, pitch, yaw) = get_rotation(pose)
print ("{},{},{}".format(np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)))

angle = euler_from_quaternion ([0.27,0.65,-0.26,0.65])
print (np.rad2deg(angle))

r = rospy.Rate(1)
while not rospy.is_shutdown():
    r.sleep()

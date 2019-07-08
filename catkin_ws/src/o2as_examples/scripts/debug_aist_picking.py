#!/usr/bin/env python

import sys
import copy
import rospy
import geometry_msgs.msg
import tf_conversions
import tf
from math import pi
import numpy as np
from collections import namedtuple

from o2as_msgs.srv import *
import actionlib
import o2as_msgs.msg
from o2as_usb_relay.srv import *
from o2as_graspability_estimation.srv import *

from o2as_routines.base import O2ASBaseRoutines

LOG_LEVEL = log_level = rospy.DEBUG
# LOG_LEVEL = log_level = rospy.INFO

class DebugAistRealRobotClass(O2ASBaseRoutines):
    """
    This contains the routine used to run the kitting task. See base.py for shared convenience functions.
    """
    def __init__(self):

        super(DebugAistRealRobotClass, self).__init__()
        
        # params
        self.speed_fast = 0.05
        self.speed_slow = 0.05
        self.bin_list = rospy.get_param("bin_list")
        self.tray_partition_list = rospy.get_param("tray_partition_list")
        # self.gripper_id = rospy.get_param('gripper_id')
        # self.tray_id = rospy.get_param('tray_id')
        
        # services
        # self._suction = rospy.ServiceProxy("o2as_usb_relay/set_power", SetPower)
        # self._search_grasp = rospy.ServiceProxy("search_grasp", SearchGrasp)

        # self.set_up_item_parameters()
        # rospy.sleep(.5)

        self.tf_listener = tf.TransformListener()

    def set_up_item_parameters(self):
        self.item_names = []
        self.downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
        # 

    def move_to_point(self):
        try:
            self.go_to_named_pose("home", "b_bot", self.speed_fast)

            for i in range(10):
                goal_pose = geometry_msgs.msg.PoseStamped()
                goal_pose.header.frame_id = "world"
                goal_pose.pose.position.z = 0.86
                goal_pose.pose.orientation = copy.deepcopy(self.downward_orientation)
                self.move_lin("b_bot", goal_pose, self.speed_slow, "b_bot_dual_suction_gripper_pad_link")
                raw_input()

        except rospy.ROSInterruptException:
            return

    def go_to_each_bin(self):

        try:
            self.go_to_named_pose("home", "b_bot", self.speed_fast)

            for i in range(10):
                goal_pose = geometry_msgs.msg.PoseStamped()
                goal_pose.header.frame_id = self.bin_list[i]["bin_name"]
                goal_pose.pose.position.z = 0.15
                goal_pose.pose.orientation = copy.deepcopy(self.downward_orientation)
                self.move_lin("b_bot", goal_pose, self.speed_slow, "b_bot_dual_suction_gripper_pad_link")
                raw_input()

        except rospy.ROSInterruptException:
            return

    def go_to_each_tray(self):
        try:

            self.go_to_named_pose("home", "b_bot", self.speed_fast)

            for i in range(10):
                if self.tray_partition_list[i] == "NONE":
                    continue
                goal_pose = geometry_msgs.msg.PoseStamped()
                goal_pose.header.frame_id = self.tray_partition_list[i]
                goal_pose.pose.position.z = 0.05
                goal_pose.pose.orientation = copy.deepcopy(self.downward_orientation)
                self.move_lin("b_bot", goal_pose, self.speed_slow, "b_bot_dual_suction_gripper_pad_link")
                raw_input()

        except rospy.ROSInterruptException:
            return

    def check_tf_transformPoint(self):
        rospy.logdebug("check_tf_transformPoint")
        pose_origin = geometry_msgs.msg.PointStamped()
        pose_origin.header.frame_id = "world"
        pose_origin.point = geometry_msgs.msg.Point(0, 0, 0.81)
        rospy.logdebug("origin point in %s: (x, y, z) = %f %f %f ", 
            pose_origin.header.frame_id, 
            pose_origin.point.x, 
            pose_origin.point.y, 
            pose_origin.point.z)

        pose_origin_to_workspace_center = tf_listener.transformPoint("workspace_center", pose_origin)
        rospy.logdebug("origin point in %s: (x, y, z) = %f %f %f ", 
            pose_origin_to_workspace_center.header.frame_id, 
            pose_origin_to_workspace_center.point.x, 
            pose_origin_to_workspace_center.point.y, 
            pose_origin_to_workspace_center.point.z)

    def check_tf_transformPose(self):
        rospy.logdebug("check_tf_transformPose")
        pose_origin = geometry_msgs.msg.PoseStamped()
        pose_origin.header.frame_id = "a_phoxi_m_sensor"
        pose_origin.pose.position.x = 0
        pose_origin.pose.position.y = 0
        pose_origin.pose.position.z = 0
        pose_origin.pose.orientation = copy.deepcopy(self.downward_orientation)
        rospy.logdebug("origin point in %s: (x, y, z) = %f %f %f ", pose_origin.header.frame_id, 
                                                                    pose_origin.pose.position.x, 
                                                                    pose_origin.pose.position.y, 
                                                                    pose_origin.pose.position.z)
        
        pose_origin_to_workspace_center = self.tf_listener.transformPose("/workspace_center", pose_origin)
        rospy.logdebug("origin point in %s: (x, y, z) = %f %f %f ", pose_origin_to_workspace_center.header.frame_id, 
                                                                    pose_origin_to_workspace_center.pose.position.x, 
                                                                    pose_origin_to_workspace_center.pose.position.y, 
                                                                    pose_origin_to_workspace_center.pose.position.z)

        # tf_listener2 = tf.TransformListener()
        pose_workspace_center = self.tf_listener.transformPose("/workspace_center", pose_origin_to_workspace_center)
        rospy.logdebug("origin point in %s: (x, y, z) = %f %f %f ", pose_workspace_center.header.frame_id, 
                                                                    pose_workspace_center.pose.position.x, 
                                                                    pose_workspace_center.pose.position.y, 
                                                                    pose_origin_to_workspace_center.pose.position.z)

    def check_geometry_msgs_msg_point_stamped(self):
        # geometry_msgs.msg.PointStamped() doesn't allow object initialization with parameters.
        point_stamped_without_params = geometry_msgs.msg.PointStamped()
        point_stamped_without_params.header.frame_id = "world"
        rospy.logdebug("Initialize without paramters: (%f, %f, %f) in %s.", 
            point_stamped_without_params.point.x,
            point_stamped_without_params.point.y,
            point_stamped_without_params.point.z,
            point_stamped_without_params.header.frame_id)

        point_stamped_with_params = geometry_msgs.msg.PointStamped(0, 0, 0.81)
        point_stamped_with_params.header.frame_id = "world"
        rospy.logdebug("Initialize with paramters: (%f, %f, %f) in %s.", 
            point_stamped_with_params.point.x,
            point_stamped_with_params.point.y,
            point_stamped_with_params.point.z,
            point_stamped_with_params.header.frame_id)

    def check_geometry_msgs_msg_point(self):
        # geometry_msgs.msg.Point() doesn't allow object initialization with parameters?
        point_without_params = geometry_msgs.msg.Point()
        rospy.logdebug("Initialize without paramters: (%f, %f, %f).", 
            point_without_params.x,
            point_without_params.y,
            point_without_params.z)

        point_with_params = geometry_msgs.msg.Point(0, 0, 0.81)
        rospy.logdebug("Initialize with paramters: (%f, %f, %f).", 
            point_with_params.x,
            point_with_params.y,
            point_with_params.z)

    def calc_transmat_phoxi(self):
        """
            We can get extrinsic paramters related to phoxi_m_sensor to world.
            But if you want to set origin of phoxi in URDF, you should calculate the pose of phoxi_m_camera.
        """

        T_sensor = np.array([
            [-0.034784670752, 0.873298269909, -0.485942546454, 0.594404677631629],
            [0.979858695089, -0.065869488303, -0.188515644360, 0.128504467275411],
            [-0.196639172950, -0.482712484078, -0.853417654714, 1.427249342095321],
            [0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000]
        ])

        try:
            (trans,rot) = self.tf_listener.lookupTransform('/a_phoxi_m_sensor', '/a_phoxi_m_camera', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        
        # print(trans)
        # print(rot)

        T_camera_to_sensor = tf.transformations.quaternion_matrix(rot)
        T_camera_to_sensor[0,3] = trans[0]
        T_camera_to_sensor[1,3] = trans[1]
        T_camera_to_sensor[2,3] = trans[2]

        # rpy = tf.transformations.euler_from_matrix(T_camera_to_sensor)
        # xyz = [T_camera_to_sensor[0,3], T_camera_to_sensor[1,3], T_camera_to_sensor[2,3]]
        # print(rpy)
        # print(xyz)

        pose_phoxi = np.dot(T_camera_to_sensor, T_sensor)

        print(T_w)
        rpy = tf.transformations.euler_from_matrix(T_w)
        xyz = [T_w[0,3], T_w[1,3], T_w[2,3]]
        print(rpy)
        print(xyz)

    def prompt_ee_point(self, robot_name, ee_link_name="", frame_id = "world"):
        robot_pose = self.groups[robot_name].get_current_pose(robot_name+"_"+ee_link_name)
        robot_pose_converted = self.tf_listener.transformPose(frame_id, robot_pose)
        rpy = tf.transformations.euler_from_quaternion([robot_pose.pose.orientation.x, 
                                                        robot_pose.pose.orientation.y, 
                                                        robot_pose.pose.orientation.z, 
                                                        robot_pose.pose.orientation.w])

        rospy.logdebug("\nrobot ee position in %s: (x, y, z) = (%s, %s, %s)", 
            robot_pose.header.frame_id, 
            robot_pose.pose.position.x, 
            robot_pose.pose.position.y, 
            robot_pose.pose.position.z)
        rospy.logdebug("\nrobot ee rotation in %s: (rx, ry, rz) = (%s, %s, %s)", 
            robot_pose.header.frame_id, 
            rpy[0], rpy[1], rpy[2])

        rospy.logdebug("\nrobot ee position in %s: (x, y, z) = (%s, %s, %s)", 
            robot_pose_converted.header.frame_id, 
            robot_pose_converted.pose.position.x, 
            robot_pose_converted.pose.position.y, 
            robot_pose_converted.pose.position.z)
        rospy.logdebug("\nrobot ee rotation in %s: (rx, ry, rz) = (%s, %s, %s)", 
            robot_pose_converted.header.frame_id, 
            rpy[0], rpy[1], rpy[2])

if __name__ == '__main__':

  try:
    debug = DebugAistRealRobotClass()
    debug.set_up_item_parameters()
    # debug.go_to_each_bin()
    # debug.go_to_each_tray()
    # debug.check_tf_transformPoint()
    # debug.check_tf_transformPose()
    # debug.check_geometry_msgs_msg_point()
    # debug.calc_transmat_phoxi()
    # debug.prompt_ee_point("b_bot", "dual_suction_gripper_pad_link", "b_bot_base_link")
    debug.move_to_point()

    print "============ Done!"
  except rospy.ROSInterruptException:
    exit(0)

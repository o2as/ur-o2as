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
from o2as_msgs.msg import *
# from o2as_usb_relay.srv import *
from graspability_estimation.srv import *
# from o2as_graspability_estimation.srv import *
from std_msgs.msg import Bool

from o2as_routines.base import O2ASBaseRoutines

import rospkg
rp = rospkg.RosPack()
import csv
import os
import random


import math
from geometry_msgs.msg import Polygon, Point32
from PIL import Image as ImagePIL
from PIL import ImageDraw as ImageDrawPIL

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

LOG_LEVEL = log_level = rospy.DEBUG
# LOG_LEVEL = log_level = rospy.INFO

CroppedArea = namedtuple("CroppedArea", ["min_row", "max_row", "min_col", "max_col"])

class kitting_order_entry():
  """
  Object that tracks if its order was fulfilled, and the number of attempts spent on it.
  """
  def __init__(self, part_id, set_number, number_in_set, bin_name, target_frame, ee_to_use, item_name, dropoff_height, bin_is_inclined):
    self.part_id = part_id   # The part id
    self.set_number = set_number
    self.number_in_set = number_in_set
    self.bin_name = bin_name
    self.target_frame = target_frame
    self.ee_to_use = ee_to_use
    self.item_name = item_name
    self.dropoff_height = dropoff_height
    self.bin_is_inclined = bin_is_inclined

    self.attempts = 0
    self.fulfilled = False
    self.in_feeder = False

def clamp(n, minn, maxn):
  """Constrain a number n to the interval [minn, maxn]"""
  return min(max(n, minn), maxn)

class KittingClass(O2ASBaseRoutines):
  """
  This contains the routine used to run the kitting task. See base.py for shared convenience functions.
  """
  def __init__(self):
    super(KittingClass, self).__init__()
    #subscribe to gripper position
    self._inner_gripper_pos_sub = rospy.Subscriber("o2as_precision_gripper/inner_gripper_motor_pos", std_msgs.msg.Int32, self.update_motorPosition)
    self._motorPos = -1

    # services
    self._feeder_srv = rospy.ServiceProxy("o2as_usb_relay/set_power", o2as_msgs.srv.SetPower)
    self._suction = actionlib.SimpleActionClient('o2as_fastening_tools/suction_control', SuctionControlAction)
    self._suctioned = False
    self._suction_state = rospy.Subscriber("suction_tool/screw_suctioned", Bool, self._suction_state_callback)
    # self._suction.wait_for_server()
    self._search_grasp = rospy.ServiceProxy("search_grasp", SearchGrasp)
    self.search_grasp_client = actionlib.SimpleActionClient("search_grasp_phoxi", o2as_msgs.msg.SearchGraspPhoxiAction)
    # action
    self.blob_detection_client = actionlib.SimpleActionClient('blob_detection_action', o2as_msgs.msg.blobDetectionAction)

    # Image Subscriber for monitoring    
    self.img_blob_topic = "o2as_blob_detection/img_w_blob"    
    #self.img_blob_sub =  rospy.Subscriber(self.img_blob_topic, Image, self.image_blob_callback)

    # Image Publisher for monitoring    
    self.bridge = CvBridge()

    self.img_inner_pick_topic = "o2as_monitor/vision_res" 
    self.img_inner_pick_pub = rospy.Publisher(self.img_inner_pick_topic, Image, latch=True, queue_size=10)
    self.current_img_w_goal = Image()
    self.current_img_w_blob = Image()

    #for gazebo
    #    self.cameraMatK = np.array([[554.3827128226441, 0.0, 320.5],
    #                           [0.0, 554.3827128226441, 240.5],
    #                           [0.0, 0.0, 1.0]])

    #for ID Realsense on robot ID61*41   width 640 height 360
    self.cameraMatK = np.array([[461.605774, 0.0, 318.471497],
                           [0.0, 461.605804, 180.336258],
                           [0.0, 0.0, 1.0]])
    #For phoxi in omron
    self.cameraPhoxiMatK = np.array([[2226.52477235, 0.0, 978.075053258],
                           [0.0, 2226.52477235, 775.748572147],
                           [0.0, 0.0, 1.0]])

    self.initial_setup()
    rospy.sleep(.5)
    # Initialize debug monitor
    self.start_task_timer()
    self.log_to_debug_monitor(text="Init", category="task")
    self.log_to_debug_monitor(text="Init", category="subtask")
    self.log_to_debug_monitor(text="Init", category="operation")
    rospy.loginfo("Kitting class started up!")

  def update_motorPosition(self, msg):
    self._motorPos = msg.data

  def initial_setup(self):
    ### First, set up internal parameters
    ### Then, read order file and create the list to iterate through

    # Used for validating grasp poses. This measures only the inside borders of the bin.
    self.bin_1_width = .11
    self.bin_1_length = .11
    self.bin_2_width = .16
    self.bin_2_length = .09
    self.bin_3_width = .20
    self.bin_3_length = .16
    
    self.initial_phoxi_image_recorded = False
    self.max_candidates_from_phoxi = 3

    # Used to prepare the suction place poses (because the UR linear driver goes through a singularity sometimes)
    self.joints_above_set_1_tray_2 = [0.6957670450210571, -1.5090416113482874, 1.9396471977233887, -0.4243395964251917, 0.7138931751251221, -3.1503987948047083]
    self.joints_above_set_2_tray_2 = [1.195457935333252, -1.6689851919757288, 2.0958428382873535, -0.4254062811480921, 1.2131953239440918, -3.146900002156393]
    self.joints_above_set_3_tray_2 = [0.550778865814209, -1.4439471403705042, 1.867344856262207, -0.42732316652406865, -1.0016759077655237, -3.134343926106588]
    self.joints_above_set_1_tray_1 = [1.333075761795044, -1.8985555807696741, 2.2680954933166504, -0.3765481154071253, 2.572127342224121, -3.1496201197253626]

    self.grasp_strategy = {
        "part_4" : "suction", 
        "part_5" : "suction", 
        "part_6" : "robotiq_gripper", 
        "part_7" : "suction",
        "part_8" : "suction", 
        "part_9" : "precision_gripper_from_inside", 
        "part_10": "precision_gripper_from_inside", 
        "part_11": "suction",  # Can be one of the grippers
        "part_12": "suction",  # Can be precision_gripper_from_outside in inclined bin
        "part_13": "suction",  # Should be precision_gripper_from_inside
        "part_14": "precision_gripper_from_outside", 
        "part_15": "precision_gripper_from_inside", 
        "part_16": "precision_gripper_from_inside", 
        "part_17": "precision_gripper_from_outside", 
        "part_18": "precision_gripper_from_outside"}
      
    # How high the end effector should hover over the tray when delivering the item
    self.dropoff_heights = {
        "part_4" : 0.03, 
        "part_5" : 0.02, 
        "part_6" : 0.01, 
        "part_7" : 0.04,
        "part_8" : 0.01, 
        "part_9" : 0.005, 
        "part_10": 0.005, 
        "part_11": 0.01,
        "part_12": 0.01,
        "part_13": 0.01,
        "part_14": 0.005, 
        "part_15": 0.005, 
        "part_16": 0.005, 
        "part_17": 0.005, 
        "part_18": 0.005}

    # NEEDS MANUAL CALIBRATION
    self.precision_gripper_pick_heights = {
        "part_9" : 0.00, 
        "part_10": 0.00, 
        "part_14": 0.00, 
        "part_15": 0.00, 
        "part_16": 0.00,
        "part_17": 0.00, 
        "part_18": 0.00}
    
    # How high the end effector should insert the bin when pick up the item.
    self.insert_offsets = {
        "part_4" : 0.000, 
        "part_5" : 0.000, 
        "part_6" : 0.002, 
        "part_7" : 0.000,
        "part_8" : 0.000, 
        "part_9" : 0.003, 
        "part_10": 0.002, 
        "part_11": 0.000,
        "part_12": 0.000,
        "part_13": 0.000,
        "part_14": 0.000, 
        "part_15": 0.002, 
        "part_16": 0.0005, 
        "part_17": 0.0035, 
        "part_18": 0.0025
    }

    self.grasp_candidates = {
        4 : {
          "pick_was_successful": False,
          "vision_was_attempted": False,
          "positions": []
        },
        5 : {
          "pick_was_successful": False,
          "vision_was_attempted": False,
          "positions": []
        }, 
        6 : {
          "pick_was_successful": False,
          "vision_was_attempted": False,
          "positions": []
        }, 
        7 : {
          "pick_was_successful": False,
          "vision_was_attempted": False,
          "positions": []
        },
        8 : {
          "pick_was_successful": False,
          "vision_was_attempted": False,
          "positions": []
        }, 
        9 : {
          "pick_was_successful": False,
          "vision_was_attempted": False,
          "positions": []
        }, 
        10: {
          "pick_was_successful": False,
          "vision_was_attempted": False,
          "positions": []
        }, 
        11: {
          "pick_was_successful": False,
          "vision_was_attempted": False,
          "positions": []
        },
        12: {
          "pick_was_successful": False,
          "vision_was_attempted": False,
          "positions": []
        },
        13: {
          "pick_was_successful": False,
          "vision_was_attempted": False,
          "positions": []
        },
        14: {
          "pick_was_successful": False,
          "vision_was_attempted": False,
          "positions": []
        }, 
        15: {
          "pick_was_successful": False,
          "vision_was_attempted": False,
          "positions": []
        }, 
        16: {
          "pick_was_successful": False,
          "vision_was_attempted": False,
          "positions": []
        }, 
        17: {
          "pick_was_successful": False,
          "vision_was_attempted": False,
          "positions": []
        }, 
        18: {
          "pick_was_successful": False,
          "vision_was_attempted": False,
          "positions": []
        }
    }

    # self.part_bin_list = {
    #   "part_4" : "bin2_1",
    #   "part_5" : "bin2_1",
    #   "part_6" : "bin3_1",
    #   "part_8" : "bin2_2",
    #   "part_9" : "bin1_1",
    #   "part_10" : "bin1_1",
    #   "part_11" : "bin2_3",
    #   "part_12" : "bin1_2",
    #   "part_13" : "bin2_4",
    #   "part_14" : "bin1_1",
    #   "part_15" : "bin1_2",
    #   "part_16" : "bin1_3",
    #   "part_17" : "bin1_4",
    #   "part_18" : "bin1_5"}
    self.part_bin_list = rospy.get_param("part_bin_list")

    self.part_position_in_tray = {
      "part_4" : "tray_1_partition_4",
      "part_5" : "tray_2_partition_6",
      "part_6" : "tray_1_partition_3",
      "part_7" : "tray_1_partition_2",
      "part_8" : "tray_2_partition_1",
      "part_9" : "tray_2_partition_4",
      "part_10": "tray_2_partition_7",
      "part_11": "tray_1_partition_1",
      "part_12": "tray_2_partition_3",
      "part_13": "tray_1_partition_5",
      "part_14": "tray_2_partition_2",
      "part_15": "tray_2_partition_5",
      "part_16": "tray_2_partition_8",
      "part_17": "tray_2_this_is_a_screw_so_should_be_ignored",
      "part_18": "tray_2_this_is_a_screw_so_should_be_ignored" }

    # This can be copied from kitting_part_bin_list_auto.yaml
    # Possible values: "left", "right"
    # "left" means the right side of the bin is higher than the left. It leans to the left.
    self.bin_for_this_part_is_inclined = {
      "part_4" : False,  # motor
      "part_5" : False,  # small pulley
      "part_6" : False,  # belt
      "part_7" : False,  # bearing
      "part_8" : False,  # shaft
      "part_9" : False,  # end cap
      "part_10" : False, # output spacer
      "part_11" : False, # output pulley
      "part_12" : False, # idler spacer
      "part_13" : False, # idler pulley/bearing
      "part_14" : False, # idler pin
      "part_15" : False, # m6 nut
      "part_16" : False, # m6 washer
      "part_17" : False, # m4 screw
      "part_18" : False} # m3 screw

    # The "from_behind" orientation is slightly turned to avoid the robot locking up
    self.suction_orientation_from_behind = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi*10/180))
    self.suction_orientation_left_bins = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/4))
    self.suction_orientation_right_bins = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/4))
    self.suction_orientation_from_45 = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/4))
    self.suction_orientation_from_side = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))
    self.downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    self.downward_orientation_2 = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))

    ### Create the item list
    self.order_list_raw, self.ordered_items = self.read_order_file()
    rospy.loginfo("Received order list:")
    rospy.loginfo(self.order_list_raw)

    ### Sort by grasp strategy
    self.screw_items = []
    self.screws = dict()
    self.screws["m3"] = []
    self.screws["m4"] = []
    self.suction_items = []
    self.robotiq_gripper_items = []
    self.precision_gripper_items = []
    for order_item in self.ordered_items:
      if order_item.part_id in [17,18]:
        rospy.loginfo("Appended item nr." + str(order_item.number_in_set) + " from set " + str(order_item.set_number) + " (part ID:" + str(order_item.part_id) + ") to list of screw items")
        self.screw_items.append(order_item)
        if order_item.part_id == 17:
          self.screws["m4"].append(order_item)
        elif order_item.part_id == 18:
          self.screws["m3"].append(order_item)
      elif order_item.ee_to_use == "suction":
        rospy.loginfo("Appended item nr." + str(order_item.number_in_set) + " from set " + str(order_item.set_number) + " (part ID:" + str(order_item.part_id) + ") to list of suction items")
        self.suction_items.append(order_item)
      elif order_item.ee_to_use == "robotiq_gripper":
        rospy.loginfo("Appended item nr." + str(order_item.number_in_set) + " from set " + str(order_item.set_number) + " (part ID:" + str(order_item.part_id) + ") to list of robotiq_gripper items")
        self.robotiq_gripper_items.append(order_item)
      elif "precision_gripper" in order_item.ee_to_use:
        rospy.loginfo("Appended item nr." + str(order_item.number_in_set) + " from set " + str(order_item.set_number) + " (part ID:" + str(order_item.part_id) + ") to list of precision gripper items")
        self.precision_gripper_items.append(order_item)
    

  def read_order_file(self):
    """Read in the order file, return kitting_list and order_entry_list.
       kitting_list is a list of lists with only the part IDs that are ordered. 
       order_entry_list is a list of kitting_entry_item objects."""

    order_entry_list = []
    kitting_list = []
    kitting_list.append([])
    kitting_list.append([])
    kitting_list.append([])
    
    with open(os.path.join(rp.get_path("o2as_scene_description"), "config", "kitting_order_file.csv"), 'r') as f:
      reader = csv.reader(f)
      header = next(reader)
      # [0, 1, 2, 3, 4] = ["Set", "No.", "ID", "Name", "Note"]
      for data in reader:
        kitting_list[int(data[0])-1].append(int(data[2]))
        order_entry_list.append(kitting_order_entry(part_id=int(data[2]), set_number=int(data[0]), 
                              number_in_set=int(data[1]),
                              bin_name=self.part_bin_list["part_" + data[2]],
                              target_frame="set_" + data[0] + "_" + self.part_position_in_tray["part_" + data[2]], 
                              ee_to_use=self.grasp_strategy["part_" + data[2]],
                              item_name=data[4],
                              dropoff_height=self.dropoff_heights["part_" + data[2]],
                              bin_is_inclined=self.bin_for_this_part_is_inclined["part_" + data[2]]))
    return kitting_list, order_entry_list

    

  ################ ----- Routines  
  ################ 
  ################

  def pick(self, robot_name, object_pose, grasp_height, speed_fast, speed_slow, gripper_command, approach_height = 0.05, special_pick = False):
    # If the pick uses suction, pass it to the local function. Otherwise to the parent class.
    if gripper_command == "suction":
      object_pose.pose.position.z += approach_height
      self.move_lin(robot_name, object_pose, speed_fast, end_effector_link="b_bot_suction_tool_tip_link")
      object_pose.pose.position.z -= approach_height
      picked = self.pick_using_dual_suction_gripper(robot_name, object_pose, speed_slow, end_effector_link="b_bot_suction_tool_tip_link")
      object_pose.pose.position.z += approach_height
      self.move_lin(robot_name, object_pose, speed_slow, end_effector_link="b_bot_suction_tool_tip_link")
      object_pose.pose.position.z -= approach_height
      if self._suctioned:
        object_pose.pose.position.z += approach_height + .05
        self.move_lin(robot_name, object_pose, speed_slow, end_effector_link="b_bot_suction_tool_tip_link")
        object_pose.pose.position.z -= approach_height + .05
      return self._suctioned
    else:
      return super(KittingClass, self).pick(robot_name, object_pose, grasp_height, speed_fast, speed_slow, gripper_command, approach_height, special_pick)

  def place(self, robot_name, object_pose, grasp_height, speed_fast, speed_slow, gripper_command, approach_height = 0.05, lift_up_after_place = True):
    # If the place uses suction, pass it to the local function. Otherwise to the parent class.
    if gripper_command == "suction":
      above_place_pose = copy.deepcopy(object_pose)
      place_pose = copy.deepcopy(object_pose)
      above_place_pose.pose.position.z += approach_height + grasp_height
      place_pose.pose.position.z += grasp_height + .03
      
      # rospy.loginfo("Going above tray.")
      # self.move_lin(robot_name, above_place_pose, speed_slow, end_effector_link="b_bot_suction_tool_tip_link")
      rospy.loginfo("Placing in tray.")
      self.move_lin(robot_name, place_pose, speed_slow, end_effector_link="b_bot_suction_tool_tip_link")
      self.suck(turn_suction_on=False, eject=True)
      rospy.sleep(2.0)
      self.suck(turn_suction_on=False, eject=False)
      self.move_lin(robot_name, above_place_pose, speed_fast, end_effector_link="b_bot_suction_tool_tip_link")
      return True
    else:
      return super(KittingClass, self).place(robot_name, object_pose, grasp_height, speed_fast, speed_slow, gripper_command, approach_height, lift_up_after_place)
    
  def set_feeder_power(self, turn_on=True):
    if not self.use_real_robot:
      return True
    req = o2as_msgs.srv.SetPowerRequest()
    req.port = 2
    req.on = turn_on
    res = self._feeder_srv.call(req)
    return res.success

  def _suction_state_callback(self, msg):
    self._suctioned = msg.data

  def suck(self, turn_suction_on=False, eject=False):
    # Judge success or fail using pressure status.
    if not self.use_real_robot:
      return True
    
    goal = SuctionControlGoal()
    goal.fastening_tool_name = "suction_tool"
    goal.turn_suction_on = turn_suction_on
    goal.eject_screw = False
    self._suction.send_goal(goal)
    self._suction.wait_for_result(rospy.Duration(2.0))
    if not turn_suction_on:
      goal.eject_screw = True
      self._suction.send_goal(goal)
      self._suction.wait_for_result(rospy.Duration(2.0))
      goal.eject_screw = False
      self._suction.send_goal(goal)
      self._suction.wait_for_result(rospy.Duration(2.0))
    return self._suction.get_result()

  def pick_using_dual_suction_gripper(self, group_name, pose_goal_stamped, speed, end_effector_link="b_bot_suction_tool_tip_link"):
    rospy.loginfo("Try picking up by suction.")
    res = self.suck(True)
    if not res:
      return False
    rospy.loginfo("Pushing into the bin.")
    if self.use_real_robot:
      self.do_linear_push("b_bot", force=5.0, wait=True, direction="Y+", max_approach_distance=.092, forward_speed=.04)
      self.confirm_to_proceed("Went to the target. Press enter")
      # The max_approach_distance parameter affects how far the robot goes into the bin
      res = True
    else:
      res = self.move_lin(group_name, pose_goal_stamped, speed, end_effector_link=end_effector_link)
    return res

  def naive_pick(self, group_name, bin_id, speed_fast = 1.0, speed_slow = 1.0, approach_height = 0.05, bin_eff_height = 0.07, bin_eff_xoff = 0, bin_eff_yoff = 0, bin_eff_deg_angle = 0,end_effector_link = ""):
    #Place gripper above bin
    goal_pose_above = geometry_msgs.msg.PoseStamped()
    goal_pose_above.header.frame_id = bin_id
    goal_pose_above.pose.position.x = bin_eff_xoff
    goal_pose_above.pose.position.y = bin_eff_yoff
    goal_pose_above.pose.position.z = bin_eff_height
    goal_pose_above.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2 , 0))
    res = self.move_lin(group_name, goal_pose_above, speed=speed_fast, end_effector_link=end_effector_link)
    if not res:
      rospy.loginfo("Couldn't go to the target.")
      return False

    #Open the gripper 
    self.send_gripper_command(gripper= "precision_gripper_inner", command="close")
    self.precision_gripper_inner_open()
    rospy.sleep(1)

    #Descend
    rospy.loginfo("Descend on target.")
    goal_pose_descend = copy.deepcopy(goal_pose_above)
    goal_pose_descend.pose.position.z = goal_pose_descend.pose.position.z - approach_height
    res  = self.move_lin(group_name, goal_pose_descend, speed=speed_slow, end_effector_link=end_effector_link)
    if not res:
      rospy.loginfo("Couldn't go to above the target bin.")
      return False

    #Close the gripper 
    self.send_gripper_command(gripper= "precision_gripper_inner", command="open")
    self.precision_gripper_inner_close()
    rospy.sleep(1)

    #Ascend
    rospy.loginfo("Ascend with target.")
    goal_pose_ascend = copy.deepcopy(goal_pose_descend)
    goal_pose_ascend.pose.position.z = goal_pose_ascend.pose.position.z + approach_height
    res  = self.move_lin(group_name, goal_pose_ascend, speed=speed_slow, end_effector_link=end_effector_link)
    if not res:
      rospy.loginfo("Couldn't go to above the target bin.")
      return False

  def pick_screw_with_precision_gripper(self, bin_id, screw_size, speed_fast = 1.0, speed_slow = .1, approach_height = 0.05, bin_eff_height = 0.07, bin_eff_deg_angle = 0,end_effector_link = ""):
    success_pick = False
    #Close the gripper and save the motor position
    self.send_gripper_command(gripper= "precision_gripper_inner", command="open")
    rospy.sleep(0.1)
    close_pos = rospy.get_param("/precision_gripper_server/inner_close_motor_position", 2222)

    posx = 0
    posy = 0
    posz = 0.08

    while (not success_pick):
      pick_pose = geometry_msgs.msg.PoseStamped()
      pick_pose.header.frame_id = bin_id
      pick_pose.pose.orientation = self.downward_orientation
      gripper_command = "inner_gripper_from_outside"
      self.pick("a_bot", pick_pose, 0.0, speed_fast = 1.0, speed_slow = 0.15, 
                        gripper_command=gripper_command, approach_height = 0.05)

      #Check posture of the screw
      rospy.loginfo("Begin check motion")
      over_bin_3_joint_pose = [0.5639523868982476, -1.2023834734104668, 2.084380077110544, -4.115980903386012, -1.350262946004677, 1.5910085738144437]
      self.move_joints("a_bot", over_bin_3_joint_pose, speed=.5, force_ur_script=self.use_real_robot)

      over_bin_3_inclined_joint_pose = [0.5646427393623301, -1.202333924982511, 2.040723585395901, -4.197616886811121, -1.3550737620026068, 1.6180429123653095]
      self.move_joints("a_bot", over_bin_3_inclined_joint_pose, speed=.5, force_ur_script=self.use_real_robot)
      
      #Open just a bit the gripper
      if(screw_size == 4):
        open_range = 80
      elif(screw_size == 3):
        open_range = 60
      else:
        rospy.logerror("Screw size is wrong")
        open_range = 0
      rospy.loginfo("begin inner open slightly")
      self.precision_gripper_inner_open_slightly(open_range)
      rospy.loginfo("ending inner open slightly")
      rospy.sleep(1.0)
      #Close the gripper fully
      self.send_gripper_command(gripper= "precision_gripper_inner", command="close")
      rospy.sleep(1.0)
      #Check the motor position 
      success_pick = True
      if(abs(self._motorPos - close_pos) > 25 ):
        rospy.loginfo("Screw was picked successfully. self._motorPos = " + str(self._motorPos))
        success_pick = True
      else:
        rospy.loginfo("Did not pick screw")
        return False
      if not self.use_real_robot:
        rospy.loginfo("Assume success for the simulation")
        success_pick = True
    
    above_handover_joint_pose = [1.3983707427978516, -1.343994442616598, 2.048433780670166, -4.056087795888082, -1.9765618483172815, 1.4695862531661987]
    self.move_joints("a_bot", above_handover_joint_pose, speed=.3, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("screw_handover", "a_bot", force_ur_script=self.use_real_robot)
    self.precision_gripper_inner_open_slightly(open_range)
    return True

  def mask_bin(self, group_name, bin_id):
    mask_margin = 0.0
    point_top1 = geometry_msgs.msg.PointStamped()
    point_top1.header.frame_id = str(bin_id)+"_bottom_front_right_corner"
    point_top1.point = geometry_msgs.msg.Point(-mask_margin, -mask_margin, 0.0)

    point_top2 = geometry_msgs.msg.PointStamped()
    point_top2.header.frame_id = str(bin_id)+"_bottom_back_right_corner"
    point_top2.point = geometry_msgs.msg.Point(mask_margin, -mask_margin, 0.0)

    point_top3 = geometry_msgs.msg.PointStamped()
    point_top3.header.frame_id = str(bin_id)+"_bottom_back_left_corner"
    point_top3.point = geometry_msgs.msg.Point(mask_margin, mask_margin, 0.0)

    point_top4 = geometry_msgs.msg.PointStamped()
    point_top4.header.frame_id = str(bin_id)+"_bottom_front_left_corner"
    point_top4.point = geometry_msgs.msg.Point(-mask_margin, mask_margin, 0.0)

    #TODO change the fisheye from to the depth frame in casse of offset. but fisheye should e ok since the two images are aligned (depth and rgb) after the real sense node
    point_top1_cam = self.listener.transformPoint("a_phoxi_m_sensor", point_top1).point
    point_top2_cam = self.listener.transformPoint("a_phoxi_m_sensor", point_top2).point
    point_top3_cam = self.listener.transformPoint("a_phoxi_m_sensor", point_top3).point
    point_top4_cam = self.listener.transformPoint("a_phoxi_m_sensor", point_top4).point


    #print("point_top1_cam")
    #print(point_top1_cam)
    #print("point_top2_cam")
    #print(point_top2_cam)
    #print("point_top3_cam")
    #print(point_top3_cam)
    #print("point_top4_cam")
    #print(point_top4_cam)

    #point_test_center_cam = geometry_msgs.msg.Point(0, 0, 0.4)
    #print(point_test_center_cam)

    #TODO project the 4 points in the depth plane (which projection matrix to use from the camera)
    #projection matrix
    #554.3827128226441, 0.0, 320.5, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 1.0    
    #
    #P: [554.3827128226441, 0.0, 320.5, -38.80678989758509, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    #TODO take the parameters from the /camera_info topic instead



    point_top1_cam_np = np.array([point_top1_cam.x, point_top1_cam.y, point_top1_cam.z])   
    point_top2_cam_np = np.array([point_top2_cam.x, point_top2_cam.y, point_top2_cam.z])   
    point_top3_cam_np = np.array([point_top3_cam.x, point_top3_cam.y, point_top3_cam.z])   
    point_top4_cam_np = np.array([point_top4_cam.x, point_top4_cam.y, point_top4_cam.z])   
    #used to test the projection
    #point_test_center_cam_np = np.array([point_test_center_cam.x, point_test_center_cam.y, point_test_center_cam.z])   
    
    #need the camera parameter of the phoxy
      
    point_top1_img_np = self.cameraPhoxiMatK.dot(point_top1_cam_np)
    point_top2_img_np = self.cameraPhoxiMatK.dot(point_top2_cam_np)
    point_top3_img_np = self.cameraPhoxiMatK.dot(point_top3_cam_np)
    point_top4_img_np = self.cameraPhoxiMatK.dot(point_top4_cam_np)
    #point_test_center_img_np = self.cameraMatK.dot(point_test_center_cam_np) 

    #print(point_top1_img_np)
    #print(point_top2_img_np)
    #print(point_top3_img_np)
    #print(point_top4_img_np)
    #print(point_test_center_img_np)

    in_polygon = Polygon()
    in_polygon.points = [Point32(point_top1_img_np[0]/point_top1_img_np[2],point_top1_img_np[1]/point_top1_img_np[2],0),
                    Point32(point_top2_img_np[0]/point_top2_img_np[2],point_top2_img_np[1]/point_top2_img_np[2],0),
                    Point32(point_top3_img_np[0]/point_top3_img_np[2],point_top3_img_np[1]/point_top3_img_np[2],0),
                    Point32(point_top4_img_np[0]/point_top4_img_np[2],point_top4_img_np[1]/point_top4_img_np[2],0)]

    polygon = [ (in_polygon.points[0].x,in_polygon.points[0].y),
                    (in_polygon.points[1].x,in_polygon.points[1].y),
                    (in_polygon.points[2].x,in_polygon.points[2].y),
                    (in_polygon.points[3].x,in_polygon.points[3].y)]


    mask_img = ImagePIL.new('L', (2064,1544), 0)
    ImageDrawPIL.Draw(mask_img).polygon(polygon, outline = 1, fill = 255)
    mask_image_np = np.array(mask_img)
    namefile = "mask_phoxi_"+str(bin_id)+".png"
    mask_img.save(namefile,'PNG')


  def view_bin(self, group_name, bin_id, part_id, speed_fast = 1.0, speed_slow = 1.0, bin_eff_height = 0.2, bin_eff_xoff = 0, bin_eff_deg_angle = 20,end_effector_link = ""):
    # TODO: adjust the x,z and end effector orientatio for optimal view of the bin to use with the  \search_grasp service
    goal_pose = geometry_msgs.msg.PoseStamped()
    goal_pose.header.frame_id = bin_id
    goal_pose.pose.position.x = bin_eff_xoff - .1
    goal_pose.pose.position.y = -.05
    goal_pose.pose.position.z = bin_eff_height - .1

    goal_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
    res = self.move_lin(group_name, goal_pose, speed_slow, end_effector_link="")
    if not res:
      rospy.loginfo("Couldn't go to the target.")
    #TODO Problem with gazebo controller while controlling the robot with movelin
      return False 
 
    #TODO Ensure that the the motion is finished before generating the mask.

    #TODO not sure if the sleep is necessary is move_lin wait for the motion to be finished?
    rospy.sleep(2)

    mask_margin = 0.0
    point_top1 = geometry_msgs.msg.PointStamped()
    point_top1.header.frame_id = str(bin_id)+"_bottom_front_right_corner"
    point_top1.point = geometry_msgs.msg.Point(-mask_margin, -mask_margin, 0.0)

    point_top2 = geometry_msgs.msg.PointStamped()
    point_top2.header.frame_id = str(bin_id)+"_bottom_back_right_corner"
    point_top2.point = geometry_msgs.msg.Point(mask_margin, -mask_margin, 0.0)

    point_top3 = geometry_msgs.msg.PointStamped()
    point_top3.header.frame_id = str(bin_id)+"_bottom_back_left_corner"
    point_top3.point = geometry_msgs.msg.Point(mask_margin, mask_margin, 0.0)

    point_top4 = geometry_msgs.msg.PointStamped()
    point_top4.header.frame_id = str(bin_id)+"_bottom_front_left_corner"
    point_top4.point = geometry_msgs.msg.Point(-mask_margin, mask_margin, 0.0)

    #TODO change the fisheye from to the depth frame in casse of offset. but fisheye should e ok since the two images are aligned (depth and rgb) after the real sense node
    point_top1_cam = self.listener.transformPoint("a_bot_camera_calibrated_frame_link", point_top1).point
    point_top2_cam = self.listener.transformPoint("a_bot_camera_calibrated_frame_link", point_top2).point
    point_top3_cam = self.listener.transformPoint("a_bot_camera_calibrated_frame_link", point_top3).point
    point_top4_cam = self.listener.transformPoint("a_bot_camera_calibrated_frame_link", point_top4).point


    #point_test_center_cam = geometry_msgs.msg.Point(0, 0, 0.4)
    #print(point_test_center_cam)

    #TODO project the 4 points in the depth plane (which projection matrix to use from the camera)
    #projection matrix
    #554.3827128226441, 0.0, 320.5, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 1.0    
    #
    #P: [554.3827128226441, 0.0, 320.5, -38.80678989758509, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    #TODO take the parameters from the /camera_info topic instead


    point_top1_cam_np = np.array([point_top1_cam.x, point_top1_cam.y, point_top1_cam.z])   
    point_top2_cam_np = np.array([point_top2_cam.x, point_top2_cam.y, point_top2_cam.z])   
    point_top3_cam_np = np.array([point_top3_cam.x, point_top3_cam.y, point_top3_cam.z])   
    point_top4_cam_np = np.array([point_top4_cam.x, point_top4_cam.y, point_top4_cam.z])   
    #used to test the projection
    #point_test_center_cam_np = np.array([point_test_center_cam.x, point_test_center_cam.y, point_test_center_cam.z])   
      
    point_top1_img_np = self.cameraMatK.dot(point_top1_cam_np)
    point_top2_img_np = self.cameraMatK.dot(point_top2_cam_np)
    point_top3_img_np = self.cameraMatK.dot(point_top3_cam_np)
    point_top4_img_np = self.cameraMatK.dot(point_top4_cam_np)
    #point_test_center_img_np = self.cameraMatK.dot(point_test_center_cam_np) 


    mask_polygon = Polygon()
    mask_polygon.points = [Point32(point_top1_img_np[0]/point_top1_img_np[2],point_top1_img_np[1]/point_top1_img_np[2],0),
                    Point32(point_top2_img_np[0]/point_top2_img_np[2],point_top2_img_np[1]/point_top2_img_np[2],0),
                    Point32(point_top3_img_np[0]/point_top3_img_np[2],point_top3_img_np[1]/point_top3_img_np[2],0),
                    Point32(point_top4_img_np[0]/point_top4_img_np[2],point_top4_img_np[1]/point_top4_img_np[2],0)]

    goal = o2as_msgs.msg.blobDetectionGoal()
    goal.maskCorner = mask_polygon
    goal.param_part_id = "part_" + str(part_id)

    self.blob_detection_client.send_goal(goal)
    self.blob_detection_client.wait_for_result()
    result = self.blob_detection_client.get_result()
    #rospy.loginfo(result)

    #TODO select which poses to choose in the array
    if result:
      poseArrayRes = geometry_msgs.msg.PoseArray()

      #TODO Sort pose in the midle of the bin
      #min x min y in the bin frame

      if(result.success): 
        poseArrayRes = result.posesDetected 
        distanceToBinCenter = []
        for i in range(len(poseArrayRes.poses)): 
          pointCam = geometry_msgs.msg.PointStamped()
          #simulation only
          poseArrayRes.header.frame_id = "a_bot_camera_calibrated_frame_link"
          pointCam.header = poseArrayRes.header
          pointCam.point = poseArrayRes.poses[i].position
          pointBin = self.listener.transformPoint(bin_id, pointCam).point
          distanceToBinCenter.append(math.sqrt(pointBin.x*pointBin.x + pointBin.y*pointBin.y))
        minPoseIndex = np.argmin(distanceToBinCenter)
         
        rospy.loginfo("point closest to the bin center in the xy plane")
        rospy.loginfo(poseArrayRes.poses[minPoseIndex])

        #Transform point from camera reference to bin reference
        pointPartCam = geometry_msgs.msg.PointStamped()
        pointPartCam.header = poseArrayRes.header
        pointPartCam.point = poseArrayRes.poses[minPoseIndex].position
        pointPartBin = self.listener.transformPoint(bin_id, pointPartCam)

        rospy.loginfo("Pose in bin")
        rospy.loginfo(pointPartBin)

        #Transform point from camera reference to bin reference
        goal_part = geometry_msgs.msg.PoseStamped()
        goal_part.header.frame_id = pointPartBin.header.frame_id
        goal_part.pose.position.x = pointPartBin.point.x
        goal_part.pose.position.y = pointPartBin.point.y
        goal_part.pose.position.z = pointPartBin.point.z
        goal_part.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2 , 0))

        self.current_img_w_blob = copy.deepcopy(result.blobImage)
        self.draw_point3D_after_view_bin(pointPartCam)

        return goal_part     

    else:
        rospy.loginfo("no pose detected")
    #TODO if nothing is detected move the camera a bit to try to detect somethin
        return False      

  def draw_point3D_after_view_bin(self, pointPartCam):
        ### Project the goal on the image plane using the camera matrix like for the mask
        #Transform Point in camera reference
        point3D_to_draw = geometry_msgs.msg.PointStamped()
        point3D_to_draw  = self.listener.transformPoint("a_bot_camera_calibrated_frame_link", pointPartCam)
        #Copy the results

        # Project the point to the camera plane
        pointPart_cam_np = np.array([point3D_to_draw.point.x, point3D_to_draw.point.y, point3D_to_draw.point.z])
        pointPart_img_np = self.cameraMatK.dot(pointPart_cam_np)
        #Transfom into pixel coordinate
        pointPart_pix= pointCam = geometry_msgs.msg.Point(pointPart_img_np[0]/pointPart_img_np[2],pointPart_img_np[1]/pointPart_img_np[2],0)
        img_blob_cv = self.bridge.imgmsg_to_cv2(self.current_img_w_blob , desired_encoding="passthrough")
        #img_blob_cv_np = np.asarray(img_blob_cv)[:, :, ::-1]
        #Draw the circle in orange for the propose pose
        cv2.circle(img_blob_cv,(int(pointPart_pix.x),int(pointPart_pix.y)), 3, (0,255,0), 4) 

        self.current_img_w_goal = self.bridge.cv2_to_imgmsg(img_blob_cv, "rgb8")

        #publish the results
        try:
          print("publish")
          self.img_inner_pick_pub.publish(self.current_img_w_goal)
        except CvBridgeError as e:
          print(e)
          
  def place_screw_in_tray(self, screw_size, set_number, hole_number):
    """
    Places a screw in a tray. A screw needs to be carried by the screw tool!
    """
    # Use this command to equip the screw tool: do_change_tool_action(self, "c_bot", equip=True, screw_size = 4)
    
    if not screw_size==3 and not screw_size==4:
      rospy.logerr("Screw size needs to be 3 or 4!")
      return False
    
    # Turn to the right to face the feeders
    self.go_to_named_pose("feeder_pick_ready", "c_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
    
    pose_tray = geometry_msgs.msg.PoseStamped()
    pose_tray.header.frame_id = "set_" + str(set_number) + "_tray_2_screw_m" + str(screw_size) + "_" + str(hole_number)
    if set_number == 1:
      pose_tray.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/2, 0, 0))
      pose_tray.pose.position.x = -.01
    elif set_number == 2:
      pose_tray.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/4, 0, 0))
      pose_tray.pose.position.x = -.005
    elif set_number == 3:
      pose_tray.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
      pose_tray.pose.position.x = -.005
    if screw_size == 3:
      pose_tray.pose.position.x += .005

    success = self.do_place_action("c_bot", pose_tray, tool_name = "screw_tool", screw_size=screw_size)
    if not success:
      return False
    
    self.go_to_named_pose("feeder_pick_ready", "c_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
    return True
    

  ################ ----- Demos  
  ################ 
  ################ 

  def pick_screw_from_bin_and_put_into_feeder(self, item, max_attempts = 5):
    robot_name = "a_bot"
    end_effector_link = "a_bot_precision_gripper_tip_link"
    if item.part_id == 17:
      screw_size = 4
    elif item.part_id == 18:
      screw_size = 3

    attempts = 0
    while attempts < max_attempts:
      attempts += 1
      item.attempts += 1
      rospy.loginfo("Attempting to pick screw. Item nr." + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + "). Attempt nr. " + str(item.attempts))
      
      # Attempt to pick the item
      pick_pose = self.get_random_pose_in_bin(item)
      if not item.bin_is_inclined:
        pick_pose.pose.orientation = self.downward_orientation_2
        if item.part_id == 18: #M3
          pick_pose.pose.position.z = .005   # MAGIC NUMBER! Depends on the amount of screws in the bin!
        if item.part_id == 17: #M4
          pick_pose.pose.position.z = .00   # MAGIC NUMBER! Depends on the amount of screws in the bin!
      gripper_command = "inner_gripper_from_outside"

      self.pick(robot_name, pick_pose, 0.0, speed_fast = 1.0, speed_slow = 0.3, 
                        gripper_command=gripper_command, approach_height = 0.04)

      screw_picked = self.check_pick(item.part_id)
      if not screw_picked and self.use_real_robot:
        rospy.logerr("Failed an attempt to pick item nr." + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + "). Reattempting. Current attempts: " + str(attempts))
        continue
      
      self.put_screw_in_feeder(screw_size)
      
      item.in_feeder = True
      rospy.loginfo("Delivered screw m" + str(screw_size) + " to feeder (item nr." + str(item.number_in_set) + " from set " + str(item.set_number))
      return True
    
    return False
    
  def get_random_pose_in_bin(self, item):
    pick_pose = geometry_msgs.msg.PoseStamped()
    pick_pose.header.frame_id = item.bin_name

    if "bin1" in item.bin_name:
      bin_length = self.bin_1_width
      bin_width = self.bin_1_length
    elif "bin2" in item.bin_name:
      bin_length = self.bin_2_width
      bin_width = self.bin_2_length
    elif "bin3" in item.bin_name:
      bin_length = self.bin_3_width
      bin_width = self.bin_3_length

    pick_pose.pose.position.x += -bin_length/2 + random.random()*bin_length
    pick_pose.pose.orientation = self.downward_orientation_2
    if item.bin_is_inclined:
      rospy.loginfo("Applying position for inclined bin")
      if "bin1" in item.bin_name:
        if item.bin_is_inclined == "left":
          pick_pose.pose.position.y = -.035
        elif item.bin_is_inclined == "right":
          pick_pose.pose.position.y = .035
      else: # Does not work because the width values are not strictly true (but safe for the regular random pick)
        pick_pose.pose.position.y = bin_width/2-.01  
      pick_pose.pose.position.z = .01
      if "precision_gripper" in item.ee_to_use:
        pick_pose.pose.position.z = self.precision_gripper_pick_heights["part_"+str(item.part_id)]
      if item.part_id in [9, 16, 17,18]:
        pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/2))
      if item.part_id in [16, 17,18]:
        pick_pose.pose.position.z = 0.0
      if item.part_id in [9]:
        pick_pose.pose.position.y += .005*random.random()
        pick_pose.pose.position.z = 0.005
      rospy.loginfo(pick_pose.pose)
    else:
      pick_pose.pose.position.y += -bin_width/2 + random.random()*bin_width
    return pick_pose

  def get_grasp_candidates_from_phoxi(self, item, take_new_image=False):
    """This requests a list of grasp positions and takes a new image with the phoxi if take_new_image is True."""
    goal = o2as_msgs.msg.SearchGraspPhoxiGoal()
    goal.part_id = item.part_id
    goal.bin_name = item.bin_name
    goal.gripper_type = item.ee_to_use
    goal.update_image = take_new_image
    try:
      self.search_grasp_client.send_goal(goal)
      self.search_grasp_client.wait_for_result(rospy.Duration(7.0))
      resp_search_grasp = self.search_grasp_client.get_result()
    except:
      rospy.logerr("Could not get grasp from Phoxi")
      return False
    if resp_search_grasp:
      if resp_search_grasp.success:
        poses_in_bin = list()
        pose0 = geometry_msgs.msg.PointStamped()
        pose0.header.frame_id = "a_phoxi_m_sensor"
        number_of_pose_candidates = min(self.max_candidates_from_phoxi, resp_search_grasp.result_num)
        for i in range(number_of_pose_candidates):
          object_position = copy.deepcopy(pose0)
          object_position.point = geometry_msgs.msg.Point(
          resp_search_grasp.pos3D[i].x,
          resp_search_grasp.pos3D[i].y, 
          resp_search_grasp.pos3D[i].z)
          # TODO We should talk about how to use rotiqz which the two_finger approaches.
          rospy.logdebug("\nGrasp point in %s: (x, y, z) = (%f, %f, %f)", 
          object_position.header.frame_id, 
          object_position.point.x, 
          object_position.point.y, 
          object_position.point.z)
          obj_pose_in_camera = geometry_msgs.msg.PoseStamped()
          obj_pose_in_camera.header = object_position.header
          obj_pose_in_camera.pose.position = object_position.point
          obj_pose_in_camera.pose.orientation.w = 1.0
          # pose_in_bin = geometry_msgs.msg.PoseStamped()
          # pose_in_bin.header.frame_id = item.bin_name
          pose_in_bin = self.listener.transformPose(item.bin_name, obj_pose_in_camera)
          pose_in_bin.pose.orientation = self.downward_orientation
          if item.ee_to_use in ["precision_gripper_from_outside", "robotiq_gripper"]:
            pose_in_bin.pose.orientation = geometry_msgs.msg.Quaternion(
              *tf_conversions.transformations.quaternion_from_euler(0, pi/2, -resp_search_grasp.rotipz[i]))
          poses_in_bin.append(pose_in_bin)
        self.publish_marker(pose_in_bin, "aist_vision_result")
        rospy.loginfo("Calculated " + str(number_of_pose_candidates) + " candidates for item nr. " + str(item.part_id) + " in bin " + str(item.bin_name))
        rospy.logdebug(poses_in_bin)
        return poses_in_bin
    else:
      rospy.loginfo("Could not find a pose via phoxi.")
      return False

  def treat_screws_in_feeders(self):
    """ Picks the screws when they have been in the feeder long enough """
    self.screws_in_feeder = dict()
    self.screws_done["m4"] = True
    self.screws_done["m3"] = True
    self.screws_in_feeder["m3"] = False
    self.screws_in_feeder["m4"] = False
    for item in self.ordered_items:
      if not item.fulfilled:
        if item.part_id == 17:
          screw_size = 4
        elif item.part_id == 18:
          screw_size = 3
        else:
          continue
        rospy.loginfo("Found an undelivered screw in the order: item nr. " + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + ")")
        self.screws_done["m"+str(screw_size)] = False
        if item.in_feeder:
          self.screws_in_feeder["m"+str(screw_size)] = True
    if (not self.screws_done["m4"] and self.screws_in_feeder["m4"]) or (not self.screws_done["m3"] and self.screws_in_feeder["m3"]):
      if not self.screws_in_feeder["m4"] and not self.screws_in_feeder["m3"]:
        rospy.loginfo("No screws are in the feeders. Skipping.")
        return
      if (rospy.Time.now() - self.screw_delivery_time) > rospy.Duration(1):  #180
        self.go_to_named_pose("back", "a_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
        
        self.go_to_named_pose("suction_ready_back", "b_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
        for screw_size in [4,3]:
          if rospy.is_shutdown():
            return
          if not self.screws_done["m"+str(screw_size)]:
            rospy.loginfo("=== Picking m" + str(screw_size) + " screws from feeder")
            self.go_to_named_pose("tool_pick_ready", "c_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
            self.do_change_tool_action("c_bot", equip=True, screw_size=screw_size)
            for item in self.screws["m"+str(screw_size)]:
              if rospy.is_shutdown():
                return
              if item.in_feeder:
                if self.pick_screw_from_feeder(screw_size, attempts=2):
                  self.place_screw_in_tray(screw_size, item.set_number, self.screws_placed["m"+str(screw_size)][item.set_number]+1)
                  self.screws_placed["m"+str(screw_size)][item.set_number] += 1
                  self.fulfilled_items += 1
                  item.in_feeder = False
                  item.fulfilled = True
                else:
                  rospy.loginfo("Failed to pick an m" + str(screw_size) + " screw from the feeder")
                  break
            if rospy.is_shutdown():
              return
            self.go_to_named_pose("tool_pick_ready", "c_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
            self.do_change_tool_action("c_bot", equip=False, screw_size=screw_size)
        self.go_to_named_pose("back", "c_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
    if self.screws_done["m4"] and self.screws_done["m3"]:
      self.set_feeder_power(False)
    return

  def make_pose_safe_for_bin(self, pick_pose, item):
    """ This makes sure that the pick_pose is not outside the bin or would cause a collision."""
    if "bin1" in item.bin_name:
      bin_length = self.bin_1_width
      bin_width = self.bin_1_length
    elif "bin2" in item.bin_name:
      bin_length = self.bin_2_width
      bin_width = self.bin_2_length
    elif "bin3" in item.bin_name:
      bin_length = self.bin_3_width
      bin_width = self.bin_3_length

    safe_pose = copy.deepcopy(pick_pose)
    safe_pose.pose.position.x = clamp(pick_pose.pose.position.x, -bin_length/2 - .02, bin_length/2 - .02)
    safe_pose.pose.position.y = clamp(pick_pose.pose.position.y, -bin_width/2 - .02, bin_width/2 - .02)
    safe_pose.pose.position.z = clamp(pick_pose.pose.position.z, 0, 0.1)

    if safe_pose.pose.position.x != pick_pose.pose.position.x or safe_pose.pose.position.y != pick_pose.pose.position.y:
      rospy.loginfo("Pose was adjusted in make_pose_safe_for_bin. Before: " + 
                    str(pick_pose.pose.position.x) + ", " + 
                    str(pick_pose.pose.position.y) + ", " + 
                    str(pick_pose.pose.position.z) + ". After: " + 
                    str(safe_pose.pose.position.x) + ", " + 
                    str(safe_pose.pose.position.y) + ", " + 
                    str(safe_pose.pose.position.z) + ".")
    
    #TODO: Adjust the gripper orientation when close to the border
    return safe_pose

  def attempt_item(self, item, max_attempts = 5):
    """This function attempts to pick an item.
       It increases the item.attempts counter each time it does, 
       and sets item.fulfilled to True if item is delivered."""
    if "precision_gripper" in item.ee_to_use:
      robot_name = "a_bot"
    elif "robotiq_gripper" in item.ee_to_use:
      robot_name = "b_bot"
    elif item.ee_to_use == "suction":
      robot_name = "b_bot"

    if item.fulfilled:
      rospy.logerr("This item is already fulfilled. Something is going wrong.")
      return False

    take_new_image = False
    if not self.initial_phoxi_image_recorded:
      self.initial_phoxi_image_recorded = True
      take_new_image = True
    
    ### Vision is skipped
    # if item.ee_to_use == "suction" or item.ee_to_use == "robotiq_gripper":
    #   if self.grasp_candidates[item.part_id]["pick_was_successful"]:
    #     # If the pick was successful for an item, the scene has changed, so a new image needs to be taken
    #     rospy.loginfo("Resetting grasp candidates for item " + str(item.part_id))
    #     self.grasp_candidates[item.part_id]["pick_was_successful"] = False
    #     self.grasp_candidates[item.part_id]["vision_was_attempted"] = False
    #     self.grasp_candidates[item.part_id]["positions"] = []
    #     take_new_image = True
    #   if item.ee_to_use == "suction" or item.ee_to_use == "robotiq_gripper":    
    #     self.go_to_named_pose("suction_ready_back", "b_bot")
    #   elif "precision_gripper" in item.ee_to_use:
    #     self.go_to_named_pose("taskboard_intermediate_pose", "a_bot")
    #   if not self.grasp_candidates[item.part_id]["vision_was_attempted"]:
    #     # Vision is only attempted once, unless it succeeds in picking
    #     phoxi_res = self.get_grasp_candidates_from_phoxi(item, take_new_image)
    #     # self.grasp_candidates[item.part_id]["vision_was_attempted"] = True
    #     #### Line above is uncommented to force new vision result every time
    #     take_new_image = False
    #     if item.ee_to_use == "suction" or item.ee_to_use == "robotiq_gripper":    
    #       self.go_to_named_pose("suction_pick_ready", "b_bot")
    #     elif "precision_gripper" in item.ee_to_use:
    #       self.go_to_named_pose("kitting_pick_ready", "a_bot")
    #     if phoxi_res:
    #       self.grasp_candidates[item.part_id]["positions"].extend(phoxi_res)
    #       rospy.loginfo("self.grasp_candidates: ")
    #       rospy.loginfo(self.grasp_candidates[item.part_id]["positions"][0])

    # Go to preparatory pose
    if item.ee_to_use == "suction":
      rospy.loginfo("Going to preparatory pose before picking from bins")
      bin_center = geometry_msgs.msg.PoseStamped()
      bin_center.header.frame_id = item.bin_name
      bin_center.pose.orientation.w = 1.0
      bin_center_on_table = self.listener.transformPose("workspace_center", bin_center).pose.position
      if bin_center_on_table.y > .1:
        self.go_to_named_pose("suction_ready_right_bins", "b_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
      else:
        self.go_to_named_pose("suction_ready_left_bins", "b_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
    elif "precision_gripper" in item.ee_to_use:
      self.go_to_named_pose("kitting_pick_ready", "a_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
    elif "robotiq_gripper" in item.ee_to_use:
      self.go_to_named_pose("home", "b_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)

    attempts = 0
    grasp_candidate_came_from_realsense = False
    grasp_candidate_from_vision = False
    while attempts < max_attempts and not rospy.is_shutdown():
      attempts += 1
      item.attempts += 1
      rospy.loginfo("=== Attempting item nr." + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + "). Attempt nr. " + str(item.attempts))
      
      # Get the pick_pose for the item, either random or from vision
      pick_pose = self.get_random_pose_in_bin(item)
      grasp_candidate_from_vision = False

      ### Realsense vision result is skipped
      if item.ee_to_use == "precision_gripper_from_inside" and not item.bin_is_inclined and not item.part_id in [17, 18]:
        pick_pose.pose.position.z = self.precision_gripper_pick_heights["part_" + str(item.part_id)]
        rospy.loginfo("Assigned pick height: " + str(pick_pose.pose.position.z))
        # res_view_bin = self.view_bin(robot_name, item.bin_name, item.part_id)    
        # grasp_candidate_came_from_realsense = True
        
        # if res_view_bin:
        #   pick_pose = res_view_bin
        #   pick_pose.pose.position.z -= .025 # MAGIC NUMBER (to compensate for the Realsense calibration)

      if item.ee_to_use == "suction" or item.ee_to_use == "robotiq_gripper":
        # for test run: if grasp_candidates is 0, item picking is skipped.
        while self.grasp_candidates[item.part_id]["positions"]:
          pick_pose = self.grasp_candidates[item.part_id]["positions"].pop(0)
          rospy.logwarn("ADDING A MAGIC ADJUSTMENT TO THE PHOXI RESULT")
          pick_pose.pose.position.y += -.002
          if pick_pose.pose.position.z <= 0.005 and item.ee_to_use == "suction":
            rospy.loginfo("Discarded a grasp candidate:")
            rospy.loginfo(pick_pose.pose.position)
            pick_pose = self.get_random_pose_in_bin(item)  # Reset to a random pose if the grasp candidate was bad
            continue
          # pick_pose.pose.position.z -= self.insert_offsets["part_"+str(item.part_id)]
          rospy.loginfo("Got pose from phoxi grasp candidates:")
          rospy.loginfo(pick_pose.pose.position)
          grasp_candidate_from_vision = True

        if item.ee_to_use == "robotiq_gripper":
          self.go_to_named_pose("home", "b_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)

        if item.ee_to_use == "suction":
          # Orientation needs to be adjusted for suction tool
          bin_center = geometry_msgs.msg.PoseStamped()
          bin_center.header.frame_id = item.bin_name
          bin_center.pose.orientation.w = 1.0
          bin_center_on_table = self.listener.transformPose("workspace_center", bin_center).pose.position
          if bin_center_on_table.y > .1:
            pick_pose.pose.orientation = self.suction_orientation_right_bins
          else:
            pick_pose.pose.orientation = self.suction_orientation_left_bins
      else:   # Precision_gripper, robotiq_gripper
        if not item.bin_is_inclined:
          pick_pose.pose.orientation = self.downward_orientation_2
      if item.part_id == 6: # Belt
        pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/2))

      approach_height = 0.1
      speed_slow = 0.1
      speed_fast = 1.0
      if item.ee_to_use == "precision_gripper_from_inside":
        gripper_command = "inner_gripper_from_inside"
        approach_height = .05
      elif item.ee_to_use == "precision_gripper_from_outside":
        gripper_command = "inner_gripper_from_outside"
        approach_height = .05
      elif item.ee_to_use == "suction":
        gripper_command = "suction"
        approach_height = 0.1 - pick_pose.pose.position.z
        speed_slow = 0.05
        speed_fast = 1.0
      elif item.ee_to_use == "robotiq_gripper":
        approach_height = .15
        speed_slow = .2
        gripper_command = ""
      else:
        gripper_command = ""
         
      pick_pose = self.make_pose_safe_for_bin(pick_pose, item)
      
      if grasp_candidate_from_vision:
        self.log_to_debug_monitor("Picking pose result from vision", "subtask")
      elif grasp_candidate_came_from_realsense:
        self.log_to_debug_monitor("Picking pose result from realsense", "subtask")
      else:
        self.log_to_debug_monitor("Picking random pose", "subtask")

      if "precision_gripper" in item.ee_to_use:
        pose_above_bin = copy.deepcopy(pick_pose)
        pose_above_bin.pose.position.z += .15
        self.go_to_pose_goal("a_bot", pose_above_bin, speed=speed_fast, move_lin=True)

      item_picked = self.pick(robot_name, pick_pose, 0.0, speed_fast = speed_fast, speed_slow = .05, 
                        gripper_command=gripper_command, approach_height = approach_height)
      if not self.use_real_robot:
        item_picked = True
      
      if item_picked and grasp_candidate_from_vision:
        self.grasp_candidates[item.part_id]["pick_was_successful"] = True
      # TODO: Check grasp success via grasp width for robotiq gripper and vision for precision gripper
      if "precision_gripper" in item.ee_to_use:
        item_picked = self.check_pick(item.part_id)
      # Suction is checked inside the pick function
      if not item_picked:
        rospy.logerr("Failed an attempt to pick item nr." + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + "). Reattempting. Current attempts: " + str(attempts))
        if item.ee_to_use == "suction":
          self.suck(False)
        continue
      
      # Attempt to place the item
      place_pose = geometry_msgs.msg.PoseStamped()
      place_pose.header.frame_id = item.target_frame
      required_intermediate_pose = []
      
      if item.ee_to_use == "suction":
        # This is inconvenient to set up because the trays are rotated. tray_2s are rotated 180 degrees relative to set_1_tray_1
        # SUCTION_PREP_POSES
        place_pose.pose.orientation, required_intermediate_pose = self.get_tray_placement_orientation_for_suction_in_kitting(
                                                  set_number=item.set_number, 
                                                  tray_number=int(place_pose.header.frame_id[11]))
        if not place_pose.pose.orientation.w and not place_pose.pose.orientation.x and not place_pose.pose.orientation.y:
          rospy.logerr("SOMETHING WENT WRONG, ORIENTATION IS NOT ASSIGNED")
      else:   # Precision_gripper, robotiq_gripper
        place_pose.pose.orientation = self.downward_orientation
      
      approach_height = .05
      if "precision_gripper" in item.ee_to_use:
        self.go_to_named_pose("taskboard_intermediate_pose", "a_bot", speed=1.5, acceleration=1.0, force_ur_script=self.use_real_robot)
        approach_height = .03
      elif item.ee_to_use == "suction":
        if required_intermediate_pose:
          rospy.loginfo("Going to intermediate pose")
          self.go_to_named_pose(required_intermediate_pose, "b_bot", speed=1.5, acceleration=1.0, force_ur_script=self.use_real_robot)
        # Approach the place pose with controllrighacceleration value
        rospy.loginfo("Approaching place pose")
        if item.set_number in [2,3] and item.target_frame[11] == '1':  # Tray 1
          self.go_to_named_pose("suction_ready_left_bins", "b_bot", speed=.1)
          self.go_to_named_pose("suction_place_intermediate_pose_for_sets_2_and_3", "b_bot", speed=.1)
        self.force_moveit_linear_motion = True
        place_pose.pose.position.z += .08
        self.move_lin(robot_name, place_pose, speed = 0.3, acceleration = 0.08, end_effector_link = "b_bot_suction_tool_tip_link")
        place_pose.pose.position.z -= .08

      if item.part_id == 6:
        if pick_pose.pose.position.x > 0:
          place_pose.pose.position.y = -0.06
        else:
          place_pose.pose.position.y = 0.06
      
      self.place(robot_name, place_pose,grasp_height=item.dropoff_height,
                    speed_fast = 0.5, speed_slow = 0.02, gripper_command=gripper_command, approach_height = approach_height)

      if item.ee_to_use == "suction":
        self.force_moveit_linear_motion = False

      # If successful
      self.fulfilled_items += 1
      item.fulfilled = True
      rospy.loginfo("Delivered item nr." + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + ")! Total items delivered: " + str(self.fulfilled_items))
      return True
    rospy.logerr("Was not able to pick item nr." + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + ")! Total attempts: " + str(item.attempts))
    
    # TODO: Move up safely if not successful
    # self.move_lin(robot_name, place_pose, speed = 0.3, acceleration = 0.08, end_effector_link = "b_bot_suction_tool_tip_link")
    return False
  
  def kitting_task(self):
    # Strategy:
    # - Start with suction tool + m4 tool equipped
    # - Pick all the screws in the current set and put them in the feeders, so they will be ready to be picked
    # - Go through all items that require suction, so the tool is equipped only once does not have to be re-equipped
    # - Go through items that are picked by the a_bot or b_bot grippers
    # - When the screws are ready, pick and place them
    # - Try to pick and return any excess screws
    # Ideally, we can fit all the trays into the scene.

    self.go_to_named_pose("back", "a_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("back", "c_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("suction_ready_back", "b_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
    start_time = rospy.Time.now()
    self.start_task_timer()
    self.log_to_debug_monitor(text="Kitting", category="task")
    time_limit = rospy.Duration(1140) # 19 minutes
    self.fulfilled_items = 0
    self.screws_done = dict()
    self.screws_placed = dict()
    self.screws_done["m4"] = (len(self.screws["m4"]) == 0)
    self.screws_done["m3"] = (len(self.screws["m3"]) == 0)
    self.screws_placed["m4"] = {1:0, 2:0, 3:0}
    self.screws_placed["m3"] = {1:0, 2:0, 3:0}

    # ==== 1. First, do an initial attempt on picking all items, ordered by the tool that they are picked with (screws, suction, precision_gripper, robotiq)

    # self.go_to_named_pose("kitting_pick_ready", "a_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
    # for item in self.screw_items:
    #   self.log_to_debug_monitor("Pick screw no. " + str(item.part_id) + " and put into feeder", "subtask")
    #   if rospy.is_shutdown():
    #     break
    #   # item.in_feeder = True   # Used to skip the screws and enable the feeder pickup
    #   self.pick_screw_from_bin_and_put_into_feeder(item, 10)
    #   self.go_to_named_pose("kitting_pick_ready", "a_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
    # self.go_to_named_pose("back", "a_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
    # rospy.loginfo("==== Done with screw pickup pass")
    self.screw_delivery_time = rospy.Time.now()

    # rospy.sleep(2)
    self.treat_screws_in_feeders()

    for item in self.suction_items:
      self.log_to_debug_monitor("Pick and place item no. " + str(item.part_id), "subtask")
      if rospy.is_shutdown():
        break
      self.treat_screws_in_feeders()
      self.attempt_item(item, 3)
    # self.do_change_tool_action("b_bot", equip=False, screw_size=50)   # 50 = suction tool
    self.go_to_named_pose("suction_ready_back", "b_bot")
    rospy.loginfo("==== Done with first suction pass")

    for item in self.precision_gripper_items:
      self.log_to_debug_monitor("Pick and place item no. " + str(item.part_id), "subtask")
      if rospy.is_shutdown():
        break
      self.go_to_named_pose("taskboard_intermediate_pose", "a_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
      self.attempt_item(item, 3)
    self.go_to_named_pose("back", "a_bot")
    rospy.loginfo("==== Done with first precision gripper pass")

    self.treat_screws_in_feeders()

    self.treat_screws_in_feeders()

    # for item in self.robotiq_gripper_items:
    #   self.log_to_debug_monitor("Pick and place item no. " + str(item.part_id), "subtask")
    #   self.go_to_named_pose("home", "b_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
    #   if rospy.is_shutdown():
    #     break
    #   self.attempt_item(item, 3)
    # self.go_to_named_pose("back", "b_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)

    self.treat_screws_in_feeders()

    # ==== 2. Second, loop through all the items that were not successfully picked on first try, and place the screws when they are assumed to be ready.
    all_done = False
    # while False:
    while not all_done and not rospy.is_shutdown():
      rospy.loginfo("Entering the loop to recover failed items and pick screws")
      
      # Check all items, find which groups are done, then reattempt those that are unfinished
      all_done = True
      suction_done = True
      precision_gripper_done = True
      robotiq_gripper_done = True
      self.screws_done["m4"] = True
      self.screws_done["m3"] = True
      for item in self.ordered_items:
        if not item.fulfilled:
          rospy.loginfo("Found an undelivered item in the order: item nr. " + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + ")")
          all_done = False
          if "precision_gripper" in item.ee_to_use:
            precision_gripper_done = False
          elif "robotiq_gripper" in item.ee_to_use:
            robotiq_gripper_done = False
          elif item.ee_to_use == "suction":
            suction_done = False
          if item.part_id == 17:
            self.screws_done["m4"] = False
          elif item.part_id == 18:
            self.screws_done["m3"] = False

      self.treat_screws_in_feeders()

      if not precision_gripper_done:
        rospy.loginfo("Reattempting the remaining precision gripper items")
        self.go_to_named_pose("taskboard_intermediate_pose", "a_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
        for item in self.precision_gripper_items:
          self.log_to_debug_monitor("Pick and place item no. " + str(item.part_id), "subtask")
          if rospy.is_shutdown():
                  break
          self.attempt_item(item)
        self.go_to_named_pose("back", "a_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)

      self.treat_screws_in_feeders()

      # if not robotiq_gripper_done:
      #   rospy.loginfo("Reattempting the remaining robotiq gripper items")
      #   self.go_to_named_pose("home", "b_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
      #   for item in self.robotiq_gripper_items:
      #     self.log_to_debug_monitor("Pick and place item no. " + str(item.part_id), "subtask")
      #     if rospy.is_shutdown():
      #             break
      #     self.attempt_item(item)
      #   self.go_to_named_pose("back", "b_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)

      self.treat_screws_in_feeders()
      
      if not suction_done:
        self.reset_grasp_candidates()
        rospy.loginfo("Reattempting the remaining suction items")
        # self.do_change_tool_action("b_bot", equip=True, screw_size=50)   # 50 = suction tool
        for item in self.suction_items:
          self.log_to_debug_monitor("Pick and place item no. " + str(item.part_id), "subtask")
          if rospy.is_shutdown():
                  break
          if not item.fulfilled:
            self.go_to_named_pose("suction_pick_ready", "b_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
            self.attempt_item(item, 5)
        # self.do_change_tool_action("b_bot", equip=False, screw_size=50)   # 50 = suction tool
        self.go_to_named_pose("back", "b_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)

      if (rospy.Time.now()-start_time) > time_limit:
        rospy.logerr("Time limit reached! Breaking out of the loop.")
        break

    if all_done:
      rospy.loginfo("Completed the task.")
    else:
      rospy.loginfo("STOPPED THE TASK")
    return

if __name__ == '__main__':
  try:
    kit = KittingClass()
    i = 1
    while i:
      rospy.loginfo("Enter 1 to equip suction tool.")
      rospy.loginfo("Enter 11 to unequip suction tool.")
      rospy.loginfo("Enter 2 to move the robots home to starting positions.")
      rospy.loginfo("Enter 3 to suction attachment test with b_bot using suction tool.")
      rospy.loginfo("Enter 60 to pick screw from bin with a_bot.")
      rospy.loginfo("Enter 61 to hand over screw from a_bot to c_bot.")
      rospy.loginfo("Enter 62 to move robots back.")
      rospy.loginfo("Enter 71, 72... to test phoxi on item 1, 2...")
      rospy.loginfo("Enter 81, 82... to move b_bot suction to trays...")
      rospy.loginfo("Enter 90 to move a_bot to kitting_pick_ready.")
      rospy.loginfo("Enter 91, 92... to do view_bin on bin1_1, bin1_2...")
      rospy.loginfo("Enter START to start the task.")
      rospy.loginfo("Enter x to exit.")
      i = raw_input()
      if i == '1':
        kit.do_change_tool_action("b_bot", equip=True, screw_size=50)   # 50 = suction tool
      if i == '11':
        kit.do_change_tool_action("b_bot", equip=False, screw_size=50)   # 50 = suction tool
      if i == '2':
        kit.go_to_named_pose("back", "a_bot", force_ur_script=kit.use_real_robot)
        kit.go_to_named_pose("back", "c_bot", force_ur_script=kit.use_real_robot)
        kit.go_to_named_pose("suction_ready_back", "b_bot", force_ur_script=kit.use_real_robot)
      elif i == 'START' or i == 'start' or i == '5000':
        kit.kitting_task()
      elif i == "60":
        kit.go_to_named_pose("home", "a_bot", force_ur_script=kit.use_real_robot)
        kit.pick_screw_with_precision_gripper(kit.part_bin_list["part_17"], screw_size= 4)
      elif i == "61":
        kit.pick_screw_from_precision_gripper(screw_size=4)
      elif i == "62":
        kit.go_to_named_pose("screw_ready", "c_bot", force_ur_script=kit.use_real_robot)
        kit.go_to_named_pose("back", "a_bot", force_ur_script=kit.use_real_robot)
      elif i in ["71", "72", "73", "74", "75", "76", "77", "78", "79"]:
        item = kit.ordered_items[int(i)-71]
        rospy.loginfo("Checking for item id " + str(item.part_id) + " in " + item.bin_name)
        obj_pose = kit.get_grasp_candidates_from_phoxi(item, True)
      elif i == "80":
        kit.go_to_named_pose("suction_pick_ready", "b_bot", force_ur_script=kit.use_real_robot)
      elif i in ["81", "82", "83", "84"]:
        place_pose = geometry_msgs.msg.PoseStamped()
        if i == "81":
          place_pose.header.frame_id = "set_1_tray_2_partition_4"
        elif i == "82":
          place_pose.header.frame_id = "set_2_tray_2_partition_4"
        elif i == "83":
          place_pose.header.frame_id = "set_3_tray_2_partition_4"
        elif i == "84":
          place_pose.header.frame_id = "set_1_tray_1_partition_3"
        
        # SUCTION_PREP_POSES
        place_pose.pose.orientation = kit.get_tray_placement_orientation_for_suction_in_kitting(
                                                  set_number=int(place_pose.header.frame_id[4]), 
                                                  tray_number=int(place_pose.header.frame_id[11]))

        kit.go_to_named_pose("back", "c_bot")
        kit.go_to_named_pose("back", "b_bot")
        kit.go_to_named_pose("screw_ready", robot_name)
        place_pose.pose.position.z = .08
        kit.move_lin("b_bot", place_pose, .1, end_effector_link="b_bot_suction_tool_tip_link")
      elif i == "90":
        kit.go_to_named_pose("kitting_pick_ready", "a_bot", force_ur_script=kit.use_real_robot)
      elif i in ["91", "92", "93", "94", "95"]:
        if i == "91":
          part_id = 16
        elif i == "92":
          part_id = 9
        elif i == "93":
          part_id = 15
        elif i == "94":
          part_id = 17
        elif i == "95":
          part_id = 18

        kit.view_bin("a_bot", "bin1_" + i[1], part_id)
      elif i == "x":
        break
      
    print "============ Done!"
  except rospy.ROSInterruptException:
    pass

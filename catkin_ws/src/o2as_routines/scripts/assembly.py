#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Felix von Drigalski

import sys
import copy
import rospy
import geometry_msgs.msg
import tf_conversions
import tf
from math import pi
import math

from o2as_msgs.srv import *
import actionlib
import o2as_msgs.msg
import std_msgs.msg


from o2as_routines.base import O2ASBaseRoutines

import ur_modern_driver.msg
def is_program_running(topic_namespace = ""):
  """Checks if a program is running on the UR"""
  msg = []
  try:
    msg = rospy.wait_for_message(topic_namespace + "/ur_driver/robot_mode_state", ur_modern_driver.msg.RobotModeDataMsg, 1.0)
  except:
    pass

  if msg:
    return msg.is_program_running
  else:
    rospy.logerr("No message received from the robot. Is everything running? Is the namespace entered correctly with a leading slash?")
    return False
    # throw()
def wait_for_UR_program(topic_namespace = "", timeout_duration = rospy.Duration.from_sec(20.0)):
  rospy.logdebug("Waiting for UR program to finish.")
  # Only run this after sending custom URScripts and not the regular motion commands, or this call will not terminate before the timeout.
  rospy.sleep(1.0)
  t_start = rospy.Time.now()
  time_passed = rospy.Time.now() - t_start
  while is_program_running(topic_namespace):
    rospy.sleep(.05)
    time_passed = rospy.Time.now() - t_start
    if time_passed > timeout_duration:
      rospy.loginfo("Timeout reached.")
      return False
  rospy.logdebug("UR Program has terminated.")
  return True

class AssemblyClass(O2ASBaseRoutines):
  """
  This contains the routine used to run the taskboard task.
  """
  def __init__(self):
    super(AssemblyClass, self).__init__()
    self.set_up_item_parameters()
    
    # self.action_client.wait_for_server()
    rospy.sleep(.5)   # Use this instead of waiting, so that simulation can be used

    # MAGIC NUMBERS!
    # This list is not exhaustive, but it's a start
    self.idler_pin_handover_offset_x = -.015
    self.idler_pin_handover_offset_y = .002   # Positive moves a_bot to c_bot
    self.idler_pin_handover_offset_z = -.008  # Negative moves to b_bot

    self.magic_a_bot_pick_offset_tray_2_x = -.004  # negative moves towards a_bot
    # self.magic_a_bot_pick_offset_tray_2_y = -.004  # negative moves towards a_bot

    # For picking screws from the tray
    self.picked_screw_counter = dict()
    self.picked_screw_counter["m3"] = 1
    self.picked_screw_counter["m4"] = 1

    # Initialize debug monitor
    self.start_task_timer()
    self.log_to_debug_monitor(text="Init", category="task")
    self.log_to_debug_monitor(text="Init", category="subtask")
    self.log_to_debug_monitor(text="Init", category="operation")

    # For nut handover
    self.nut_on_table_a = geometry_msgs.msg.PoseStamped()
    self.nut_on_table_a.header.frame_id = "workspace_center"
    self.nut_on_table_a.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))
    self.nut_on_table_a.pose.position.x = -.25
    self.nut_on_table_a.pose.position.y = -.20

    self.nut_on_table_c = copy.deepcopy(self.nut_on_table_a)
    self.nut_on_table_c.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/4))

  def set_up_item_parameters(self):
    # TODO: Publish the items to the scene, or do something equivalent. 
    self.item_names = []
    downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    # 
    

  ################ ----- Routines  
  ################ 
  ################ 
  def pick_joshua(self, robotname, object_pose, grasp_height, speed_fast, speed_slow, gripper_command="", approach_height = 0.03, 
  end_effector_link="", spiral_search_for_idle_pulley=False, spiral_max_radius=.0035, lift_up_after_pick=True):
    #initial gripper_setup
    rospy.loginfo("Going above object to pick")
    object_pose.pose.position.z = approach_height
    self.go_to_pose_goal(robotname, object_pose, speed=self.speed_fast, acceleration=self.acc_fast, end_effector_link=end_effector_link, move_lin=True)

    if gripper_command=="complex_pick_from_inside":
      self.precision_gripper_inner_close()
    elif gripper_command=="complex_pick_from_outside":
      self.precision_gripper_inner_open()
    elif gripper_command=="easy_pick_only_inner" or gripper_command=="inner_gripper_from_inside":
      self.precision_gripper_inner_close()
    elif gripper_command=="easy_pick_outside_only_inner" or gripper_command=="inner_gripper_from_outside":
      self.precision_gripper_inner_open()
    elif gripper_command=="none":
      pass
    else: 
      self.send_gripper_command(gripper=robotname, command="open")

    rospy.loginfo("Moving down to object")
    object_pose.pose.position.z = grasp_height
    rospy.loginfo(grasp_height)
    self.go_to_pose_goal(robotname, object_pose, speed=speed_slow, high_precision=True,end_effector_link=end_effector_link, move_lin=True)

    if spiral_search_for_idle_pulley:
      self.horizontal_spiral_motion("a_bot", spiral_max_radius, radius_increment=.002)

    # W = raw_input("waiting for the gripper")
    #gripper open
    if gripper_command=="complex_pick_from_inside":
      self.precision_gripper_inner_open(this_action_grasps_an_object = True)
      self.precision_gripper_outer_close()
    elif gripper_command=="complex_pick_from_outside":
      self.precision_gripper_inner_close(this_action_grasps_an_object = True)
      self.precision_gripper_outer_close()
    elif gripper_command=="easy_pick_only_inner" or gripper_command=="inner_gripper_from_inside":
      self.precision_gripper_inner_open(this_action_grasps_an_object = True)
    elif gripper_command=="easy_pick_outside_only_inner" or gripper_command=="inner_gripper_from_outside":
      self.precision_gripper_inner_close(this_action_grasps_an_object = True)
    elif gripper_command=="none":
      pass
    else: 
      self.send_gripper_command(gripper=robotname, command="close")
      
    rospy.sleep(1)
    if lift_up_after_pick:
      rospy.loginfo("Going back up")
      object_pose.pose.position.z = (approach_height)
      self.go_to_pose_goal(robotname, object_pose, speed=self.speed_fast, acceleration=self.acc_fast,end_effector_link=end_effector_link, move_lin=True)

######

  def place_joshua(self,robotname, object_pose, place_height, speed_fast, speed_slow, gripper_command="", approach_height = 0.05, approach_axis="z" ,lift_up_after_place = True):
    rospy.loginfo("Going above place target")
    if approach_axis=="z":
      object_pose.pose.position.z = approach_height
    elif approach_axis=="y":
      object_pose.pose.position.y = approach_height
    elif approach_axis=="x":
      object_pose.pose.position.x = approach_height
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast)

    rospy.loginfo("Moving to place target")
    if approach_axis=="z":
      object_pose.pose.position.z = place_height
    elif approach_axis=="y":
      object_pose.pose.position.y = place_height
    elif approach_axis=="x":
      object_pose.pose.position.x = place_height
    self.go_to_pose_goal(robotname, object_pose, speed=speed_slow, high_precision=True)

    #gripper open
    if gripper_command=="complex_pick_from_inside":
      self.precision_gripper_outer_open()
      self.precision_gripper_inner_close()
    elif gripper_command=="complex_pick_from_outside":
      self.precision_gripper_outer_open()
      self.precision_gripper_inner_open()
    elif gripper_command=="easy_pick_only_inner" or gripper_command=="inner_gripper_from_inside":
      self.precision_gripper_inner_close()
    elif gripper_command=="none":
      pass
    else: 
      self.send_gripper_command(gripper=robotname, command="open")
    
    if lift_up_after_place:
      rospy.loginfo("Moving back up")
      if approach_axis=="z":
        object_pose.pose.position.z = approach_height
      elif approach_axis=="y":
        object_pose.pose.position.y = approach_height
      elif approach_axis=="x":
        object_pose.pose.position.x = approach_height
      self.go_to_pose_goal(robotname, object_pose, speed=self.speed_fast, acceleration=self.acc_fast)  

  def pick_screw(self, robot_name, screw_size = 4, screw_number = 1):
    goal = o2as_msgs.msg.pickGoal()
    goal.robot_name = robot_name
    goal.tool_name = "screw_tool"
    goal.screw_size = screw_size

    if screw_number == "auto":
      screw_number = self.picked_screw_counter["m"+str(screw_size)]
      self.picked_screw_counter["m"+str(screw_size)] += 1
      if screw_size == 4 and self.picked_screw_counter["m"+str(screw_size)] > 9:
        rospy.logwarn("There are no more screws to pick! Resetting counter")
        picked_screw_counter["m"+str(screw_size)] = 1
      if screw_size == 3 and self.picked_screw_counter["m"+str(screw_size)] > 6:
        rospy.logwarn("There are no more screws to pick! Resetting counter")
        picked_screw_counter["m"+str(screw_size)] = 1
      if rospy.is_shutdown():
        return False
    pscrew = geometry_msgs.msg.PoseStamped()
    pscrew.header.frame_id = "tray_2_screw_m" + str(screw_size) + "_" + str(screw_number)
    pscrew.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi*11/12, 0, 0))
    goal.item_pose = pscrew
    rospy.loginfo("Sending pick action goal")
    rospy.loginfo(goal)

    self.pick_client.send_goal(goal)
    rospy.loginfo("Waiting for result")
    self.pick_client.wait_for_result()
    rospy.loginfo("Getting result")
    self.pick_client.get_result()

    # Check the pick success again
    rospy.sleep(.5)
    screw_picked = self.screw_is_suctioned["m"+str(screw_size)]
    print(screw_picked)
    if screw_picked:
      rospy.loginfo("Successfully picked the screw. Exiting pick_screw.")
      return True
    else:
      rospy.logwarn("The screw must have gotten stuck in the tray! Retrying the next screw!")
      return self.pick_screw(robot_name, screw_size, screw_number="auto")
    if not self.use_real_robot:
      rospy.loginfo("Pretending the screw is picked, because this is simulation.")
      return True

  def recenter_plate_3_on_base_plate(self):
    # c_bot_current_pose is the pose that the gripper is at while it is holding the tool
    # TODO!
    # 1. Touch with c_bot, move the plate to b_bot slightly, then release and go back to recenter pose

    # b_bot has to be at screw_plate_ready
  
    # 1. Reposition the plate a bit towards b_bot incase it moved away
    if self.c_hold_plate_3:
      c_reposition_plate = copy.deepcopy(self.c_hold_plate_3)
      c_reposition_plate.pose.position.z += 0.001
      c_reposition_plate.pose.position.y -= .01
      self.send_gripper_command("c_bot", "close")
      self.go_to_pose_goal("c_bot", c_reposition_plate, speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)
      self.send_gripper_command("c_bot", 0.008)
      rospy.sleep(1.0)
      self.send_gripper_command("c_bot", 0.02)
      rospy.sleep(1.0)
      self.go_to_pose_goal("c_bot", self.c_hold_plate_3, speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)
      self.send_gripper_command("c_bot", 0.008)

    intermediate_b_pose_1 = [-0.8361886183368128, -2.2852214018451136, 2.259411334991455, -0.0760872999774378, -0.7902057806598108, -3.0508933703051966]
    intermediate_b_pose_2 = [-0.8362606207477015, -2.28513747850527, 2.2593636512756348, 0.04411733150482178, 0.9355313181877136, -3.050953213368551]

    intermediate_b_pose_3 = [0.6182253360748291, -2.1539786497699183, 2.2336840629577637, -0.057644192372457326, 2.235957145690918, -3.1786747614489954]
    before_plate_c_push = [1.61185884475708, -1.8193615118609827, 2.4348952770233154, -0.5505860487567347, 2.7279043197631836, -3.0912707487689417]
    plate_c_pushed = [1.6026942729949951, -1.6820648352252405, 2.330822467803955, -0.5862844626056116, 2.719031572341919, -3.0942660013781946]

    self.move_joints("b_bot", intermediate_b_pose_1)
    self.move_joints("b_bot", intermediate_b_pose_2)
    self.move_joints("b_bot", intermediate_b_pose_3)
    self.move_joints("b_bot", before_plate_c_push)
    self.confirm_to_proceed("move to push plate c?")
    self.send_gripper_command("c_bot", "close")
    rospy.sleep(.5)

    self.move_joints("b_bot", plate_c_pushed)
    self.move_joints("b_bot", before_plate_c_push)
    self.move_joints("b_bot", intermediate_b_pose_3)
    self.move_joints("b_bot", intermediate_b_pose_2)
    self.move_joints("b_bot", intermediate_b_pose_1)
    self.go_to_named_pose("screw_plate_ready", "b_bot")
    self.send_gripper_command("c_bot", 0.008)
    rospy.sleep(1.0)
    self.send_gripper_command("c_bot", 0.02)
    return


  def place_plate_3_and_screw(self, place_plate_only=False, screw_first_only=False, reverse_placement_only=False):
    # Requires the screw tool to be equipped on b_bot
    self.log_to_debug_monitor("Home position", "operation")
    self.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("back", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("screw_ready_back", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    # Success check variables
    first_screw_fastened = False
    second_screw_fastened = False
    plate_correctly_fastened = False

    if not screw_first_only:
      rospy.loginfo("Going to pick up plate_3 with c_bot")
      # TODO: Attach a spawned object, use its frames to plan the next motion
      # TEMPORARY WORKAROUND: Use initial+assembled position. This does not do collision avoidance!!
      self.send_gripper_command("c_bot", "open")
      c_approach = geometry_msgs.msg.PoseStamped()
      c_approach.header.frame_id = "initial_assy_part_03_pulley_ridge_bottom" # The top corner of the big plate
      c_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, pi/2, -pi/2))
      c_approach.pose.position.x = 0.0025  # Offset by half the plate thickness, so pose is in the center of the material
      c_approach.pose.position.y = -0.02
      c_approach.pose.position.z = 0.05
      c_pickup = copy.deepcopy(c_approach)
      c_pickup.pose.position.z = -0.03    
      c_high = copy.deepcopy(c_approach)
      c_high.pose.position.z = 0.13
      c_place = copy.deepcopy(c_pickup)
      c_place.header.frame_id = "assembled_assy_part_03_pulley_ridge_bottom"
      c_place.pose.position.z += .001
      # c_place.pose.position.x += .00 # MAGIC NUMBER!!  positive points towards c_bot
      c_shake_off = copy.deepcopy(c_place)
      c_shake_off.pose.position.z -= .002
      c_move_away = copy.deepcopy(c_place)
      c_move_away.pose.position.y += .08
      self.c_hold_plate_3 = copy.deepcopy(c_place)
      self.c_hold_plate_3.pose.position.y += .02

      if reverse_placement_only: # This is for finding the right initial pose of the plate
        self.log_to_debug_monitor("Find the right initial pose of the plate", "operation")
        self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
        self.send_gripper_command("c_bot", "close")
        self.move_lin("c_bot", c_move_away, self.speed_fast, acceleration=self.acc_fast)
        self.send_gripper_command("c_bot", "open")
        self.move_lin("c_bot", c_place, .2)
        self.send_gripper_command("c_bot", "close")
        self.move_lin("c_bot", c_high, self.speed_fast, acceleration=self.acc_fast)
        self.move_lin("c_bot", c_approach, self.speed_fast, acceleration=self.acc_fast)
        c_pickup.pose.position.z = 0.0015
        self.move_lin("c_bot", c_pickup, 0.2)
        rospy.loginfo("Done")
        return
      
      self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.move_lin("c_bot", c_approach, self.speed_fast, acceleration=self.acc_fast)
      self.move_lin("c_bot", c_pickup, self.speed_fast, acceleration=self.acc_fast)
      self.send_gripper_command("c_bot", "close")
      rospy.sleep(1)

    # Pick up screw while the plate is grasped, so the cable does not interfere
    self.log_to_debug_monitor("Pickup screw", "operation")
    if not place_plate_only:
      self.go_to_named_pose("screw_pick_ready", "b_bot")
      self.pick_screw("b_bot", screw_size=4, screw_number="auto")
      self.go_to_named_pose("screw_ready_back", "b_bot")

    if not screw_first_only:
      # Deliver the plate to its assembled position
      self.log_to_debug_monitor("Fasten the plate", "operation")
      self.move_lin("c_bot", c_high, self.speed_fast, acceleration=self.acc_fast)
      self.move_lin("c_bot", c_place, .2)
      self.send_gripper_command("c_bot", 0.008)
      rospy.sleep(1.0)  # After opening, the gripper surface is often stuck to the object. This gives it time to detach.
      self.move_lin("c_bot", c_shake_off, .1)
      self.send_gripper_command("c_bot", 0.015)
      # Move to the side for the screw tool to pass
      self.move_lin("c_bot", c_move_away, self.speed_fast, acceleration=self.acc_fast)

      if place_plate_only:
        rospy.loginfo("Done placing the plate.")
        return True
    self.confirm_to_proceed("waiting here (in the middle of large plate)")
    
    
    
    ###### ==========
    # Move b_bot to the first hole and screw    
    first_screw_attempts = 0
    while not first_screw_fastened:
      first_screw_attempts += 1
      if first_screw_attempts > 3:
        rospy.logwarn("Too many unsuccessful attempts. Aborting.")
        self.go_to_named_pose("screw_plate_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
        self.send_gripper_command("c_bot", "open")
        self.go_to_named_pose("screw_ready_back", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
        # Screw is not discarded because the next task is the motor plate
        return False
      self.log_to_debug_monitor("Screw the plate", "operation")
      self.go_to_named_pose("screw_plate_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      if not screw_first_only:
        self.move_lin("c_bot", self.c_hold_plate_3, .1)    # Move c_bot to the plate so it does not move too much
        # self.send_gripper_command("c_bot", 0.008)

      pscrew = geometry_msgs.msg.PoseStamped()
      pscrew.header.frame_id = "assembled_assy_part_03_bottom_screw_hole_1"
      # pscrew.pose.position.y = .00   # MAGIC NUMBER (negative goes towards c_bot)
      pscrew.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(-pi/4, 0,0))
      pscrew_approach = copy.deepcopy(pscrew)
      pscrew_approach.pose.position.y -= .02
      pscrew_approach.pose.position.x -= .01
      self.move_lin("b_bot", pscrew_approach, speed=1.0, acceleration=0.5, end_effector_link="b_bot_screw_tool_m4_tip_link")

      pscrew_approach_2 = copy.deepcopy(pscrew_approach) # This one moves horizontally to the plate so the approach happens from the side, not the top
      pscrew_approach_2.pose.position.y += .02
      self.move_lin("b_bot", pscrew_approach_2, speed=0.1, acceleration=0.1, end_effector_link="b_bot_screw_tool_m4_tip_link")
      self.do_screw_action("b_bot", pscrew, screw_height = 0.002, screw_size = 4)
      self.go_to_named_pose("screw_plate_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

      # if self.screw_is_suctioned["m4"]:
      #   rospy.logerr("Fastening the first screw has failed! Try again after repositioning the plate")
      #   self.recenter_plate_3_on_base_plate()
      # else:
      #   rospy.loginfo("Assuming that the first screw was fastened")
      #   first_screw_fastened = False
      rospy.loginfo("Pretending that the screw is fastened")
      first_screw_fastened = True
    self.go_to_named_pose("screw_ready_back", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    if not screw_first_only:
      ###### ========== 
      # Recenter the plate with c_bot and then move away
      self.log_to_debug_monitor("Recenter the plate with c_bot and then move away", "operation")
      self.send_gripper_command("c_bot", "close", force = 10.0, velocity=.013)
      rospy.sleep(0.2)
      self.send_gripper_command("c_bot", 0.008, velocity=.013)
      rospy.sleep(1.0)
      self.send_gripper_command("c_bot", "open")
      rospy.sleep(0.5)

      self.move_lin("c_bot", c_move_away, self.speed_fast, acceleration=self.acc_fast)
      self.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

      ##### ========== Pick another screw with b_bot and fix the plate
      # Pick up screw from tray
      self.log_to_debug_monitor("Pick up screw from tray", "operation")
      self.go_to_named_pose("screw_pick_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.pick_screw("b_bot", screw_size=4, screw_number="auto")
      self.go_to_named_pose("screw_pick_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      
      self.go_to_named_pose("screw_plate_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      pscrew_2 = geometry_msgs.msg.PoseStamped()
      pscrew_2.header.frame_id = "assembled_assy_part_03_bottom_screw_hole_2"
      # pscrew_2.pose.position.y = -.001   # MAGIC NUMBER  (negative goes towards c_bot)
      pscrew_2.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(-pi/4, 0,0))
      pscrew_2_approach = copy.deepcopy(pscrew_2)
      pscrew_2_approach.pose.position.y -= .02
      pscrew_2_approach.pose.position.x -= .01
      self.move_lin("b_bot", pscrew_2_approach, speed=self.speed_fast, acceleration=self.acc_fast, end_effector_link="b_bot_screw_tool_m4_tip_link")
      self.do_screw_action("b_bot", pscrew_2, screw_height = 0.002, screw_size = 4)
      self.go_to_named_pose("screw_plate_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

  def place_plate_2_and_screw(self):
    # Requires the tool to be equipped on b_bot
    rospy.loginfo("Going to pick up screw with b_bot")
    self.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("back", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    ### --- b_bot
    # Pick first screw with b_bot
    self.log_to_debug_monitor("Pick up screw with b_bot", "operation")
    self.go_to_named_pose("screw_pick_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.pick_screw("b_bot", screw_size=4, screw_number="auto")
    self.go_to_named_pose("screw_pick_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("screw_ready_back", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    ### --- c_bot
    # Place plate and hold
    # rospy.loginfo("Going to pick up and place motor plate with c_bot")
    self.log_to_debug_monitor("Pick up and place motor plate with c_bot", "operation")
    self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.send_gripper_command("c_bot", "open")
    ps_approach = geometry_msgs.msg.PoseStamped()
    ps_approach.header.frame_id = "initial_assy_part_02_back_hole" # The top corner of the motor plate
    ps_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, pi/2, pi/2))
    ps_approach.pose.position.x = 0.0025
    ps_approach.pose.position.y = 0.0
    ps_approach.pose.position.z = 0.05
    ps_pickup = copy.deepcopy(ps_approach)
    ps_pickup.pose.position.z = -0.03
    ps_high = copy.deepcopy(ps_approach)
    ps_high.pose.position.z = 0.13
    ps_place = copy.deepcopy(ps_pickup)
    ps_place.header.frame_id = "assembled_assy_part_02_back_hole"
    ps_place.pose.position.z += .001
    # ps_place.pose.position.x += .000   # MAGIC NUMBER
    ps_hold = copy.deepcopy(ps_place)
    ps_hold.pose.position.y -= .035
    ps_hold.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_multiply(
                            tf.transformations.quaternion_from_euler(0, pi/2, pi/2), 
                            tf.transformations.quaternion_from_euler(0, -pi*6/180, 0) ))
    
    self.move_lin("c_bot", ps_approach, self.speed_fast, acceleration=self.acc_fast)
    self.move_lin("c_bot", ps_pickup, self.speed_fast, acceleration=self.acc_fast)
    self.send_gripper_command("c_bot", "close")
    self.move_lin("c_bot", ps_high, self.speed_fast, acceleration=self.acc_fast)

    # Go to the same pose at the assembly position
    self.move_lin("c_bot", ps_place, self.speed_fast, acceleration=self.acc_fast)
    self.send_gripper_command("c_bot", 0.008)
    rospy.sleep(1.0)  # After opening, the gripper surface is often stuck to the object. This gives it time to detach.
    self.send_gripper_command("c_bot", 0.08)
    self.move_lin("c_bot", ps_hold, self.speed_fast, acceleration=self.acc_fast)
    self.send_gripper_command("c_bot", 0.008)
    
    ### --- b_bot
    # Fasten motor plate with first screw
    self.log_to_debug_monitor("Fasten motor plate with first screw", "operation")
    self.go_to_named_pose("screw_plate_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    pscrew = geometry_msgs.msg.PoseStamped()
    pscrew.header.frame_id = "assembled_assy_part_02_bottom_screw_hole_1"
    # pscrew.pose.position.y = -.002   # MAGIC NUMBER (negative goes towards c_bot)
    pscrew.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(-pi/4, 0,0))
    pscrew_approach = copy.deepcopy(pscrew)
    pscrew_approach.pose.position.y -= .02
    pscrew_approach.pose.position.x -= .03
    self.move_lin("b_bot", pscrew_approach, speed=self.speed_fast, acceleration=self.acc_fast, end_effector_link="b_bot_screw_tool_m4_tip_link")
    self.do_screw_action("b_bot", pscrew, screw_height = 0.002, screw_size = 4)
    self.move_lin("b_bot", pscrew_approach, speed=0.1, acceleration=0.1, end_effector_link="b_bot_screw_tool_m4_tip_link")
    self.go_to_named_pose("screw_plate_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    # Pick second screw
    self.log_to_debug_monitor("Pick up second screw", "operation")
    self.go_to_named_pose("screw_pick_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.pick_screw("b_bot", screw_size=4, screw_number="auto")
    self.go_to_named_pose("screw_pick_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("screw_ready_back", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    ### --- c_bot
    # Recenter the motor plate
    self.log_to_debug_monitor("Recenter motor plate", "operation")
    ps_recenter = copy.deepcopy(ps_place)
    ps_recenter.pose.position.y -= .03
    self.move_lin("c_bot", ps_recenter, 0.2)
    self.send_gripper_command("c_bot", "close", force = 10.0, velocity=0.013)
    rospy.sleep(0.2)
    self.send_gripper_command("c_bot", .008, velocity=0.013)
    rospy.sleep(1.0)
    self.send_gripper_command("c_bot", "open")
    rospy.sleep(0.5)
    self.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    ### --- b_bot
    # Fasten motor plate with second screw
    self.log_to_debug_monitor("Fasten motor plate with second screw", "operation")
    self.go_to_named_pose("screw_plate_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    pscrew_2 = geometry_msgs.msg.PoseStamped()
    pscrew_2.header.frame_id = "assembled_assy_part_02_bottom_screw_hole_2"
    # pscrew_2.pose.position.y = -.003   # MAGIC NUMBER (negative goes towards c_bot)
    pscrew_2.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(-pi*80/180, 0,0))
    pscrew_2_approach = copy.deepcopy(pscrew_2)
    pscrew_2_approach.pose.position.y -= .02
    pscrew_2_approach.pose.position.x -= .03
    self.move_lin("b_bot", pscrew_2_approach, speed=self.speed_fast, acceleration=self.acc_fast, end_effector_link="b_bot_screw_tool_m4_tip_link")
    self.do_screw_action("b_bot", pscrew_2, screw_height = 0.002, screw_size = 4)
    self.go_to_named_pose("screw_plate_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.set_suction("screw_tool_m4", suction_on=False, eject=False)
    return

  # ============================================ IDLER PIN SUBTASK

  def pick_retainer_pin_from_tray_and_keep_it(self, do_centering=True):
    # rospy.loginfo("============ Pick up the retainer pin using b_bot ============")
    self.log_to_debug_monitor("Pick up the retainer pin using b_bot", "operation")
    self.go_to_named_pose("back", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.send_gripper_command("b_bot", "open")

    pick_pose = geometry_msgs.msg.PoseStamped()
    pick_pose.header.frame_id = "tray_2_partition_2_pickup"
    pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/2))
    pick_pose.pose.position.z = 0.025  # MAGIC NUMBER (actually it gets ignored by pick_joshua)

    self.pick_joshua("b_bot", pick_pose, grasp_height=0.015, speed_fast=self.speed_fastest, speed_slow=.1, gripper_command="", approach_height=.05)

    if do_centering:
      self.adjust_centering(go_fast=True)
    return
  
  def pick_retainer_pin_from_tray_and_place_in_holder(self, do_centering=False):
    # rospy.loginfo("============ Pick up the retainer pin using b_bot ============")
    self.log_to_debug_monitor("Pick up the retainer pin using b_bot", "operation")
    self.go_to_named_pose("back", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.send_gripper_command("b_bot", "open")

    pick_pose = geometry_msgs.msg.PoseStamped()
    pick_pose.header.frame_id = "tray_2_partition_2_pickup"
    pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/2))
    pick_pose.pose.position.z = 0.025  # MAGIC NUMBER (actually it gets ignored by pick_joshua)

    self.pick_joshua("b_bot", pick_pose, grasp_height=0.015, speed_fast=self.speed_fastest, speed_slow=.1, gripper_command="", approach_height=.05)

    if do_centering:
      self.adjust_centering(go_fast=True)

    place_pose = geometry_msgs.msg.PoseStamped()
    place_pose.header.frame_id = "retainer_pin_holder_link"
    place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/2))
    
    self.place_joshua("b_bot", place_pose, place_height=0.01, speed_fast=self.speed_fastest, speed_slow=.1, gripper_command="", approach_height=.05)
    return


  def pick_retainer_pin_from_holder(self):
    # rospy.loginfo("============ Pick up the retainer pin from holder using b_bot ============")
    self.log_to_debug_monitor("Pick up the retainer pin from holder using b_bot", "operation")
    self.go_to_named_pose("back", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.send_gripper_command("b_bot", "open")

    pick_pose = geometry_msgs.msg.PoseStamped()
    pick_pose.header.frame_id = "retainer_pin_holder_link"
    pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
    pick_pose.pose.position.z = 0.01  # MAGIC NUMBER (increasing it makes the gripper pick the pin closer to the head)

    self.pick_joshua("b_bot", pick_pose, grasp_height=-0.003, speed_fast=self.speed_fast, speed_slow=.1, gripper_command="", approach_height=.05)
    return

  def rotate_hand_facing_the_sky(self):
    # rospy.loginfo("============ making b_bot end effector face the sky ============")
    self.log_to_debug_monitor("Making b_bot end effector face the sky", "operation")
    
    self.move_joints("b_bot", [1.7835410833358765, -0.8952229658709925, 1.468522548675537, -2.1444996039019983, 1.5616456270217896, -0.21871883073915654], 
                    speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    
    intermediate_retainer_pin_tip = geometry_msgs.msg.PoseStamped()
    intermediate_retainer_pin_tip.header.frame_id = "intermediate_assy_part_14_screw_head"
    intermediate_retainer_pin_tip.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    intermediate_retainer_pin_tip.pose.position.z -= 0.0  # points towards b_bot 
    
    self.go_to_pose_goal("b_bot", intermediate_retainer_pin_tip,speed=self.speed_fastest, acceleration=self.acc_fastest, move_lin = True)
    return

  def pick_retainer_pin_spacer(self, robot_name = "a_bot"):
    # rospy.loginfo("============ Picking up a retainer pin spacer using a_bot ============")
    self.log_to_debug_monitor("Picking up a retainer pin spacer using a_bot", "operation")
    self.go_to_named_pose("home", robot_name, speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_partition_3_pickup"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x += self.magic_a_bot_pick_offset_tray_2_x  # MAGIC NUMBER (this moves the bot in +y in the workspace)
    pose0.pose.position.y += .002
    pose0.pose.position.z = 0.01

    self.pick_joshua("a_bot",pose0,-0.015,
                                speed_fast = self.speed_fast, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.05)

    self.go_to_named_pose("home", robot_name, speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    return

  def place_retainer_pin_spacer(self, robot_name = "a_bot"):
    intermediate_facing_sky = geometry_msgs.msg.PoseStamped()
    intermediate_facing_sky.header.frame_id = "intermediate_assy_part_14_screw_head"
    intermediate_facing_sky.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))

    self.go_to_pose_goal("b_bot", intermediate_facing_sky,speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)
    
    intermediate_retainer_pin_tip = geometry_msgs.msg.PoseStamped()
    intermediate_retainer_pin_tip.header.frame_id = "intermediate_assy_part_14_screw_tip"
    intermediate_retainer_pin_tip.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi, 0))
    intermediate_retainer_pin_tip.pose.position.y = self.idler_pin_handover_offset_y
    intermediate_retainer_pin_tip.pose.position.z += self.idler_pin_handover_offset_z

    self.place_joshua("a_bot",intermediate_retainer_pin_tip,self.idler_pin_handover_offset_x-.002,
                                speed_fast = self.speed_fast, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0,approach_axis="x", lift_up_after_place = False)
    rospy.sleep(0.5)
    if self.use_real_robot:
      self.horizontal_spiral_motion("a_bot", .004)
      rospy.loginfo("doing spiral motion")

    intermediate_facing_sky.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    intermediate_facing_sky.pose.position.x = 0.
    intermediate_facing_sky.pose.position.y = 0.
    intermediate_facing_sky.pose.position.z = -0.4
    self.go_to_pose_goal("b_bot", intermediate_facing_sky,speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)
    return

  def pick_idle_pulley(self, robot_name = "a_bot", magic_offset=.001, spiral_motion_radius=.0035):
    # rospy.loginfo("============ Picking up the idle pulley using a_bot ============")
    self.log_to_debug_monitor("Picking up the idle pulley using a_bot", "operation")
    self.go_to_named_pose("home", robot_name, speed=.5, acceleration=.5, force_ur_script=self.use_real_robot)

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_1_partition_5_pickup"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
    pose0.pose.position.x = .001 # MAGIC NUMBER
    pose0.pose.position.y = .0005 # MAGIC NUMBER
    pose0.pose.position.z = 0.02

    self.send_gripper_command("a_bot", "close")
    self.pick_joshua("a_bot",pose0,-0.008,
                                speed_fast = self.speed_fast, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.02, spiral_search_for_idle_pulley=True, spiral_max_radius=spiral_motion_radius)

    self.go_to_named_pose("home", robot_name, speed=.5, acceleration=.5, force_ur_script=self.use_real_robot)
    return

  def reset_idle_pulley(self):
    # rospy.loginfo("============ Picking up the idle pulley using a_bot ============")
    self.log_to_debug_monitor("Picking up the idle pulley using a_bot", "operation")
    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_1_partition_5_pickup"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/4))
    pose0.pose.position.x = .015  # offset
    pose0.pose.position.y = .015  # offset
    pose0.pose.position.z = -0.008

    pose1 = copy.deepcopy(pose0)
    pose1.pose.position.x = .01  # offset after pushing it into the corner
    pose1.pose.position.y = .01  # offset after pushing it into the corner

    pose_approach = copy.deepcopy(pose0)
    pose_approach.pose.position.z = .05  # offset

    self.send_gripper_command("a_bot", "open")
    self.go_to_pose_goal("a_bot", pose_approach,speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)
    self.go_to_pose_goal("a_bot", pose0,speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)
    self.go_to_pose_goal("a_bot", pose1,speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)
    self.go_to_pose_goal("a_bot", pose0,speed=.02, move_lin = True)

    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    return
  

  def place_idle_pulley(self, robot_name = "a_bot"):
    intermediate_facing_sky = geometry_msgs.msg.PoseStamped()
    intermediate_facing_sky.header.frame_id = "intermediate_assy_part_14_screw_head"
    intermediate_facing_sky.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))

    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_pose_goal("b_bot", intermediate_facing_sky,speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)
    
    intermediate_retainer_pin_tip = geometry_msgs.msg.PoseStamped()
    intermediate_retainer_pin_tip.header.frame_id = "intermediate_assy_part_14_screw_tip"
    intermediate_retainer_pin_tip.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi, 0))
    intermediate_retainer_pin_tip.pose.position.y += self.idler_pin_handover_offset_y
    intermediate_retainer_pin_tip.pose.position.z += self.idler_pin_handover_offset_z

    self.place_joshua("a_bot",intermediate_retainer_pin_tip,self.idler_pin_handover_offset_x-0.001,
                                speed_fast = self.speed_fast, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0,approach_axis="x", lift_up_after_place = False)
    rospy.sleep(0.5)

    self.confirm_to_proceed("Is a_bot aligned above b_bot?")
    
    if self.use_real_robot:
      self.horizontal_spiral_motion("a_bot", .004, radius_increment=.002)
      rospy.loginfo("doing spiral motion")

    self.confirm_to_proceed("Can a_bot go home?")

    retreat_pose = copy.deepcopy(intermediate_retainer_pin_tip)
    retreat_pose.pose.position.x += -0.03
    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    intermediate_facing_sky.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    intermediate_facing_sky.pose.position.x = 0.
    intermediate_facing_sky.pose.position.y = 0.
    intermediate_facing_sky.pose.position.z = -0.4
    self.go_to_pose_goal("b_bot", intermediate_facing_sky,speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)
    return
  
  def push_idler_pulley_down(self):
    self.go_to_named_pose("pulley_push_ready_high", "c_bot",speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.send_gripper_command(gripper="c_bot",command = 0.022, wait=False)

    b_bot_face_up_near_c = geometry_msgs.msg.PoseStamped()
    b_bot_face_up_near_c.header.frame_id = "intermediate_assy_part_14_screw_head"
    b_bot_face_up_near_c.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    b_bot_face_up_near_c.pose.position.x = 0.
    b_bot_face_up_near_c.pose.position.y = 0.4
    b_bot_face_up_near_c.pose.position.z = -0.4
    self.go_to_pose_goal("b_bot", b_bot_face_up_near_c,speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)

    c_above_pin = geometry_msgs.msg.PoseStamped()
    c_above_pin.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, 0, -pi/2))
    c_above_pin.header.frame_id = "b_bot_robotiq_85_tip_link"
    c_above_pin.pose.position.x = 0.04
    c_above_pin.pose.position.y = -0.015
    
    c_push = copy.deepcopy(c_above_pin)
    c_push.pose.position.x = 0.02


    self.go_to_pose_goal("c_bot", c_above_pin,speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)
    self.confirm_to_proceed("Is the gripper positioned right for pushing down?")
    self.go_to_pose_goal("c_bot", c_push, speed=0.01, move_lin = True)
    self.confirm_to_proceed("Is the push deep enough?")
    self.go_to_pose_goal("c_bot", c_above_pin,speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)
    self.confirm_to_proceed("Did the push work? c_bot will move away")

    b_bot_face_sky_pose = copy.deepcopy(b_bot_face_up_near_c)
    b_bot_face_sky_pose.pose.position.y = 0.0
    self.go_to_pose_goal("b_bot", b_bot_face_sky_pose,speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)

    self.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)


  def pick_retainer_pin_nut(self):
    # rospy.loginfo("============ Picking up the retainer pin nut using a_bot ============")
    self.log_to_debug_monitor("Picking up the retainer pin nut using a_bot", "operation")
    self.go_to_named_pose("home", "a_bot", speed=self.speed_fast, acceleration=self.acc_fast, force_ur_script=self.use_real_robot)

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_partition_5_pickup"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x += self.magic_a_bot_pick_offset_tray_2_x  # MAGIC NUMBER (this moves the bot in +y in the workspace)
    pose0.pose.position.z = 0.02

    self.pick_joshua("a_bot",pose0,-0.006,
                                speed_fast = self.speed_fast, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.05)

    self.go_to_named_pose("home", "a_bot")
    return

  def place_retainer_pin_nut_on_table(self):
    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    self.place_joshua("a_bot",self.nut_on_table_a,0.0,
                                speed_fast = self.speed_fast, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.05,approach_axis="z", lift_up_after_place = True)
    self.go_to_named_pose("back", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    return
  
  def pick_retainer_pin_nut_from_table(self):
    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("screw_ready", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    self.go_to_named_pose("tool_pick_ready", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.do_change_tool_action("c_bot", equip=True, screw_size=66)
    self.go_to_named_pose("screw_ready", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.pick_nut_from_table("c_bot", object_pose=self.nut_on_table_c,end_effector_link="c_bot_nut_tool_m6_tip_link")
    self.go_to_named_pose("screw_ready", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    return

  def pick_retainer_pin_washer(self, number=1):
    self.log_to_debug_monitor("Picking up the retainer pin washer " + str(number) + " using a_bot", "operation")

    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_partition_8_pickup_" + str(number)
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x += self.magic_a_bot_pick_offset_tray_2_x  # MAGIC NUMBER (this moves the bot in +y in the workspace)
    if number == 1:
      pose0.pose.position.x -= .001  # MAGIC NUMBER (axis points towards c_bot)
    pose0.pose.position.z = 0.01 # Gets ignored by pick_joshua

    self.pick_joshua("a_bot",pose0, -0.01,
                                speed_fast = self.speed_fast, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.05)

    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    return

  def place_retainer_pin_washer_1(self, robot_name = "a_bot"):
    # This places the first retainer pin washer on the pin
    intermediate_facing_sky = geometry_msgs.msg.PoseStamped()
    intermediate_facing_sky.header.frame_id = "intermediate_assy_part_14_screw_head"
    intermediate_facing_sky.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))

    self.go_to_pose_goal("b_bot", intermediate_facing_sky, speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)
    
    intermediate_retainer_pin_tip = geometry_msgs.msg.PoseStamped()
    intermediate_retainer_pin_tip.header.frame_id = "intermediate_assy_part_14_screw_tip"
    intermediate_retainer_pin_tip.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi, 0))
    intermediate_retainer_pin_tip.pose.position.y = self.idler_pin_handover_offset_y
    intermediate_retainer_pin_tip.pose.position.z += self.idler_pin_handover_offset_z - .002 # MAGIC NUMBER (positive towards a_bot)

    self.place_joshua("a_bot",intermediate_retainer_pin_tip,self.idler_pin_handover_offset_x-0.003,
                                speed_fast = self.speed_fast, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0,approach_axis="x", lift_up_after_place = False)
    rospy.sleep(0.5)
    if self.use_real_robot:
      self.horizontal_spiral_motion("a_bot", .005, radius_increment=.0015)
      rospy.loginfo("doing spiral motion")

    intermediate_facing_sky.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    intermediate_facing_sky.pose.position.x = 0.
    intermediate_facing_sky.pose.position.y = 0.
    intermediate_facing_sky.pose.position.z = -0.4
    self.go_to_pose_goal("b_bot", intermediate_facing_sky,speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)
    return

  def place_retainer_pin_washer_on_table(self):
    place_pose = geometry_msgs.msg.PoseStamped()
    place_pose.header.frame_id = "workspace_center"
    place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))
    place_pose.pose.position.x = -.13
    place_pose.pose.position.y = -.21
    place_pose.pose.position.z = .004 # includes foam

    self.place_joshua("a_bot",place_pose, -0.001,
                                speed_fast = self.speed_fast, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.1,approach_axis="z", lift_up_after_place = False)

    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    return

  def pick_retainer_pin_washer_from_table(self):
    pick_pose = geometry_msgs.msg.PoseStamped()
    pick_pose.header.frame_id = "workspace_center"
    pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))
    pick_pose.pose.position.x = -.13 + 0.001 # MAGIC NUMBER!! 
    pick_pose.pose.position.y = -.21 + 0.002 # MAGIC NUMBER!! Probably depends on the gripper
    pick_pose.pose.position.z = .004 # includes foam

    self.pick_joshua("a_bot",pick_pose, -0.001,
                                speed_fast = self.speed_fast, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.02, lift_up_after_pick=False)

    # Trial 
    # if self.use_real_robot:
    #   self.horizontal_spiral_motion("a_bot", .004, radius_increment=.004)
    #   rospy.loginfo("doing spiral motion")

    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    return

  def place_retainer_pin_washer_final(self):
    assembled_retainer_pin_tip = geometry_msgs.msg.PoseStamped()
    assembled_retainer_pin_tip.header.frame_id = "assembled_assy_part_14_screw_tip"
    assembled_retainer_pin_tip.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi*80/180,pi))
    assembled_retainer_pin_tip.pose.position.x = -0.005  # negative goes closer to the plate

    # GRASP_HEIGHT = MAGIC NUMBER 
    self.place_joshua("a_bot",assembled_retainer_pin_tip,0.005,  
                                speed_fast = self.speed_fast, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.08,approach_axis="z", lift_up_after_place = False)

    self.horizontal_spiral_motion("a_bot", .004)
    a_bot_retreat = copy.deepcopy(assembled_retainer_pin_tip)
    a_bot_retreat.pose.position.z += .1
    self.go_to_pose_goal("a_bot", a_bot_retreat, speed=.1, move_lin = True)

    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    return

  def insert_retainer_pin_to_base(self):
    # rospy.loginfo("============ inserting retainer pin to the base plate ============")
    self.log_to_debug_monitor("Insert retainer pin to the base plate", "operation")
    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    
    assembled_retainer_pin_head = geometry_msgs.msg.PoseStamped()
    assembled_retainer_pin_head.header.frame_id = "assembled_assy_part_14_screw_head"
    assembled_retainer_pin_head.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    assembled_retainer_pin_head.pose.position.x = 0.00
    # assembled_retainer_pin_head.pose.position.y = -0.0024  # MAGIC NUMBER
    assembled_retainer_pin_head.pose.position.z = 0.015  # Vertical offset 
    
    self.move_joints("b_bot", [1.315, -1.24, 1.28, -0.00, 0.23, -1.55], speed=self.speed_fast, acceleration=self.acc_fast, force_ur_script=self.use_real_robot)
    self.move_joints("b_bot", [1.924, -0.998, 1.4937, -0.496, -2.775, -3.14], speed=self.speed_fast, acceleration=self.acc_fast, force_ur_script=self.use_real_robot)

    self.go_to_pose_goal("b_bot", assembled_retainer_pin_head,speed=.05, move_lin = True)

    self.do_linear_push("b_bot", 6, wait = True)

    # assembled_retainer_pin_head_final=copy.deepcopy(assembled_retainer_pin_head)
    # assembled_retainer_pin_head_final.pose.position.x = 0.002 # MAGIC NUMBERS
    # self.go_to_pose_goal("b_bot", assembled_retainer_pin_head_final,speed=.05, move_lin = True)
    return
  
  def hold_idle_pulley_with_a_bot(self):
    self.log_to_debug_monitor("Hold idle pulley with a_bot", "operation")
    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    hold_pose_approach = geometry_msgs.msg.PoseStamped()
    hold_pose_approach.header.frame_id = "assembled_assy_part_14_screw_head"
    hold_pose_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    hold_pose_approach.pose.position.x = -0.01   # Points away from plate  
    # hold_pose_approach.pose.position.y = -0.0024  # MAGIC NUMBER
    hold_pose_approach.pose.position.z = .01
    
    hold_pose_approach_high = copy.deepcopy(hold_pose_approach)
    hold_pose_approach_high.pose.position.z += 0.04

    hold_pose = copy.deepcopy(hold_pose_approach)
    hold_pose.pose.position.x = 0.002
    if self.use_real_robot:
      self.send_gripper_command("a_bot", "close")
      self.go_to_pose_goal("a_bot", hold_pose_approach_high, speed=.2, move_lin = True)
    self.go_to_pose_goal("a_bot", hold_pose_approach, speed=.04, move_lin = True)
    self.go_to_pose_goal("a_bot", hold_pose, speed=.02, move_lin = True)
    return
  
  def release_and_push_with_b_bot(self):
    self.log_to_debug_monitor("Release and push with b_bot", "operation")
    self.send_gripper_command("b_bot", "close")
    rospy.sleep(1.0)
    self.send_gripper_command("b_bot", "open")
    rospy.sleep(1.0)

    b_bot_going_back = geometry_msgs.msg.PoseStamped()
    b_bot_going_back.header.frame_id = "assembled_assy_part_14_screw_head"
    b_bot_going_back.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    b_bot_going_back.pose.position.x = -0.03
    # b_bot_going_back.pose.position.y = -0.0024 # MAGIC NUMBER
    b_bot_going_back.pose.position.z = 0.01


    b_bot_going_back_below = copy.deepcopy(b_bot_going_back)
    b_bot_going_back_below.pose.position.z -= 0.008

    self.go_to_pose_goal("b_bot", b_bot_going_back, speed=.2, move_lin = True)
    self.send_gripper_command("b_bot", "close")
    self.send_gripper_command("b_bot", "close")
    self.send_gripper_command("b_bot", "close")
    self.send_gripper_command("b_bot", "close")
    self.send_gripper_command("b_bot", "close")
    rospy.sleep(0.5)
    self.go_to_pose_goal("b_bot", b_bot_going_back_below, speed=0.5, move_lin = True)
    self.do_linear_push("b_bot", 5, wait = True)

    req = o2as_msgs.srv.sendScriptToURRequest()
    req.program_id = "lin_move_rel"
    req.robot_name = "b_bot"
    req.relative_translation.z -= .002
    req.velocity = .1
    res = self.urscript_client.call(req)
    wait_for_UR_program("/b_bot_controller", rospy.Duration.from_sec(4.0))
    return

  def release_idle_pulley_from_a_bot(self):
    self.log_to_debug_monitor("Release idle pulley from a_bot", "operation")
    assembled_retainer_pin_head_retreat = geometry_msgs.msg.PoseStamped()
    assembled_retainer_pin_head_retreat.header.frame_id = "assembled_assy_part_14_screw_head"
    assembled_retainer_pin_head_retreat.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    assembled_retainer_pin_head_retreat.pose.position.x = 0
    # assembled_retainer_pin_head_retreat.pose.position.y = -0.0024 # MAGIC NUMBER
    assembled_retainer_pin_head_retreat.pose.position.z = 0.045
    self.go_to_pose_goal("a_bot", assembled_retainer_pin_head_retreat, speed=.1, move_lin = True)

    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    return

  def fasten_retainer_pin_nut(self):
    nut_approach = geometry_msgs.msg.PoseStamped()
    nut_approach.header.frame_id = "assembled_assy_part_14_screw_tip"
    nut_approach.pose.position.x = 0.01
    nut_approach.pose.position.z = 0.007 # MAGIC NUMBER (upwards, to allow for inclined pin)
    nut_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi*180/180, pi, 0))

    at_pin_tip = copy.deepcopy(nut_approach)
    at_pin_tip.pose.position.x = 0.0

    at_pin_end = copy.deepcopy(at_pin_tip)
    at_pin_end.pose.position.x = -0.01
    self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_pose_goal("c_bot", nut_approach, speed=.1, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
    self.confirm_to_proceed("approach pose 1")
    self.go_to_pose_goal("c_bot", at_pin_tip, speed=.08, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
    self.confirm_to_proceed("at tip")
    self.set_motor("nut_tool_m6", direction = "tighten", wait=False, speed = 500, duration = 15)
    self.do_linear_push("c_bot", 10,direction="c_bot_diagonal", wait = True)
    self.horizontal_spiral_motion("c_bot", .004, radius_increment = .002, speed = 0.02, spiral_axis="YZ")
    self.go_to_pose_goal("c_bot", at_pin_end, speed=.08, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
    self.confirm_to_proceed("at tip full")
    self.horizontal_spiral_motion("c_bot", .004, radius_increment = .002, speed = 0.02, spiral_axis="YZ")
    rospy.sleep(3)
    self.go_to_pose_goal("c_bot", at_pin_tip, speed=.05, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
    self.go_to_pose_goal("c_bot", nut_approach, speed=.1, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
    self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

  # ================================================= End of IDLE PULLEY subtask

  def pick_retainer_pin_and_place_in_holder(self):
    self.log_to_debug_monitor("Pick retainer pin and place in holder", "operation")
    self.confirm_to_proceed("pick_retainer_pin")
    self.send_gripper_command("b_bot", "open")
    self.pick_retainer_pin()
    self.confirm_to_proceed("adjust_centering")
    self.adjust_centering(go_fast=True)

    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    place_pose = geometry_msgs.msg.PoseStamped()
    place_pose.header.frame_id = "retainer_pin_holder_link"
    place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    self.place_joshua("b_bot", place_pose, place_height=.01, speed_fast=self.speed_fast, speed_slow=0.5, approach_height=.05)
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    return

  # =================

  # ===================START OF MOTOR ROUTINE (subtask )======================================
  def pick_motor(self):
    self.log_to_debug_monitor("Pick motor", "operation")
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.send_gripper_command("b_bot", "open")
    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_1_partition_4"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = -0.037016
    pose0.pose.position.y = -0.010987
    pose0.pose.position.z = 0.029001
    pose0.pose.orientation.x = 0.10824
    pose0.pose.orientation.y = 0.77248
    pose0.pose.orientation.z = -0.16552
    pose0.pose.orientation.w = 0.60346

    self.pick("b_bot",pose0,0.0,
                                speed_fast = self.speed_fast, speed_slow = 0.05, gripper_command="",
                                approach_height = 0.05, lift_up_after_pick=False)
    self.confirm_to_proceed("Grasped motor. Proceed?")
    pose_retreat = copy.deepcopy(pose0)
    pose_retreat.pose.position.z += .05
    pose_retreat.pose.position.x += .02
    pose_retreat.pose.position.y += .02
    self.go_to_pose_goal("b_bot", pose_retreat,speed=.05, move_lin = True)
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    return
  
  def handover_motor(self):
    self.log_to_debug_monitor("Handover motor", "operation")
    pose1 = geometry_msgs.msg.PoseStamped()
    pose1.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, -pi/2, 0))
    pose1.header.frame_id = "b_bot_robotiq_85_tip_link"
    pose1.pose.position.y = 0.05
    pose1.pose.position.x = 0.115
    pose1.pose.position.z = 0.26

    pose2 = copy.deepcopy(pose1)
    pose2.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi, pi/2, 0))
    pose2.pose.position.x = 0.02 # Distance away from b_bot gripper
    pose2.pose.position.y = 0.0   
    pose2.pose.position.z = 0.02 # How far into the object c_bot grasps?

    self.go_to_pose_goal("b_bot", pose1,speed=.3, move_lin = True)
    rospy.sleep(1)
    self.go_to_pose_goal("c_bot", pose2,speed=.3, move_lin = True)

    self.send_gripper_command(gripper="c_bot",command = "close")
    rospy.sleep(2)
    for i in range(2):
      self.send_gripper_command(gripper="b_bot",command = "open")

    self.go_to_named_pose("back", "c_bot")
    self.go_to_named_pose("home", "b_bot")
    return

  def insert_motor(self):
    self.log_to_debug_monitor("Insert motor", "operation")
    self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    pre_insertion = geometry_msgs.msg.PoseStamped()
    pre_insertion.header.frame_id = "assembled_assy_part_04_inserted_13"
    pre_insertion.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi, -pi/2, 0))
    pre_insertion.pose.position.x = -0.1
    pre_insertion.pose.position.y = 0.0  # points right?
    pre_insertion.pose.position.z = 0.017  # points downward
    self.go_to_pose_goal("c_bot", pre_insertion, speed=.3, move_lin = True)
    
    # self.confirm_to_proceed("Do horizontal insertion?")
    # self.do_insertion(robot_name="c_bot", wait= True, horizontal=True, peck_mode=True)
    x_offsets = [0, -.002, 0, .002]
    y_offsets = [-.002, 0, .002, 0]

    for i in range(1,4):
      self.do_linear_push("c_bot", 10, direction="Y-", wait = True)
      # Do brute force insertion/search
      req = o2as_msgs.srv.sendScriptToURRequest()
      req.program_id = "lin_move_rel"
      req.robot_name = "c_bot"
      req.velocity = .05
      req.relative_translation.y = -.002
      res = self.urscript_client.call(req)
      wait_for_UR_program("/c_bot_controller", rospy.Duration.from_sec(3.0))

    self.do_linear_push("c_bot", 10, direction="Y-", wait = True)
    req.relative_translation.y = -.002
    res = self.urscript_client.call(req)
    wait_for_UR_program("/c_bot_controller", rospy.Duration.from_sec(3.0))
    return
  
  def insert_clamping_shaft(self):
    self.log_to_debug_monitor("Insert clamping shaft", "operation")
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    # pre_insertion = geometry_msgs.msg.PoseStamped()
    # pre_insertion.header.frame_id = "tray_1_partition_2"
    # pre_insertion.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    # pre_insertion.pose.position.z = 0.07
    # self.go_to_pose_goal("b_bot", pre_insertion, speed=1.0, move_lin = True)

    pre_insertion_pose_above_tray = [1.566275715827942, -1.1347106138812464, 1.3280940055847168, -1.7666509787188929, -1.5791609922992151, 0.0038237168919295073]
    self.move_joints("b_bot", pre_insertion_pose_above_tray)
    self.do_linear_push("b_bot", 30, direction="Z+", wait = True)
    # print "inserting using linear_push in Y negative direction, please ask Felix how to call linear push for different axis using his o2as_skills server"
    # impedance control may not be necessary in this case, it is not as difficult
    # self.do_insertion(robot_name="b_bot", wait=True, horizontal=False)
    
    self.send_gripper_command("b_bot", "open")
    rospy.sleep(1.0)
    self.send_gripper_command("b_bot", "open")
    rospy.sleep(1.0)

    req = o2as_msgs.srv.sendScriptToURRequest()
    req.program_id = "lin_move_rel"
    req.robot_name = "b_bot"
    req.relative_translation.z -= .05
    req.velocity = .5
    res = self.urscript_client.call(req)
    wait_for_UR_program("/b_bot_controller", rospy.Duration.from_sec(4.0))

    self.send_gripper_command("b_bot", "close")
    rospy.sleep(1.0)
    self.send_gripper_command("b_bot", "close")
    rospy.sleep(1.0)
    self.do_linear_push("b_bot", 30, direction="Z+", wait = True)
    return

  def fasten_motor_screw(self, screw_hole_number):#for picking up and fastening a screw. need to expand this for 6 screws. I think just make 6 different functions.
    self.log_to_debug_monitor("Fasten motor screw", "operation")
    pose1 = geometry_msgs.msg.PoseStamped()
    pose1.header.frame_id = "assembled_assy_part_02_motor_screw_hole_"+str(screw_hole_number)
    pose1.pose.orientation.w = 0.707
    pose1.pose.orientation.x = -0.707
    pose1.pose.orientation.y = 0
    pose1.pose.orientation.z = 0    
    
    self.go_to_named_pose("screw_pick_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.pick_screw("b_bot", screw_size=3, screw_number="auto") # I commented this because this takes a long time in simulation
    self.go_to_named_pose("horizontal_screw_ready", "b_bot")
    self.go_to_pose_goal("b_bot", pose1, speed=0.3,end_effector_link="b_bot_screw_tool_m3_tip_link", move_lin=True)
    self.go_to_named_pose("horizontal_screw_ready", "b_bot")
    # TODO: add fastening action
    return

  def pick_motor_pulley(self):
    self.log_to_debug_monitor("Pick motor pulley", "operation")
    self.go_to_named_pose("home", "b_bot")

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_partition_6"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    pose0.pose.position.y -= 0.005
    pose0.pose.position.z = 0.07
    self.send_gripper_command(gripper="b_bot",command = 0.044)
    self.send_gripper_command(gripper="b_bot",command = 0.044)
    self.pick_joshua("b_bot", pose0, grasp_height=-0.01, speed_fast=self.speed_fast, speed_slow=.1, gripper_command="none", approach_height=.05, lift_up_after_pick=False)
    self.send_gripper_command(gripper="b_bot",command = 0.01)
    self.send_gripper_command(gripper="b_bot",command = "close")
    rospy.sleep(1.0)

    pose0.pose.position.z += 0.05
    self.go_to_pose_goal("b_bot", pose0, speed=0.3,end_effector_link="", move_lin=True)
    self.go_to_named_pose("home", "b_bot")
    return

  def insert_motor_pulley(self):
    self.log_to_debug_monitor("Insert motor pulley", "operation")

    hold_pulley_joints = []

    pre_insertion = geometry_msgs.msg.PoseStamped()
    pre_insertion.header.frame_id = "c_bot_robotiq_85_tip_link"
    pre_insertion.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi, 0))
    pre_insertion.pose.position.x = 0.05
    self.go_to_pose_goal("b_bot", pre_insertion, speed=.3, move_lin = True)
    self.do_insertion(robot_name="b_bot", wait= True, horizontal=False)
    return

  def pick_bearing(self):
    self.log_to_debug_monitor("Pick bearing", "operation")
    self.go_to_named_pose("home", "b_bot")

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_1_partition_2"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    pose0.pose.position.z = 0.07

    self.do_pick_action("b_bot", pose0, z_axis_rotation = 0.0, use_complex_planning = False)
    return

  def pick_end_cap(self):
    self.log_to_debug_monitor("Pick end cap", "operation")
    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.send_gripper_command("b_bot", "open")

    self.go_to_named_pose("home", "b_bot")

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_partition_4"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x += self.magic_a_bot_pick_offset_tray_2_x  # MAGIC NUMBER (this moves the bot in +y in the workspace)
    pose0.pose.position.z = 0.07
    self.send_gripper_command(gripper="b_bot",command = 0.04)
    self.pick_joshua("b_bot", pose0, grasp_height=0.02, speed_fast=self.speed_fast, speed_slow=.1, gripper_command="", approach_height=.05)
    self.send_gripper_command(gripper="b_bot",command = "close")
    self.go_to_named_pose("home", "b_bot")
      
    handover_b = geometry_msgs.msg.PoseStamped()
    handover_b.header.frame_id = "workspace_center"
    handover_b.pose.position.x = 0.2
    handover_b.pose.position.y = .0
    handover_b.pose.position.z = 0.7
    handover_b.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/2, 0, -pi/2))
      

    handover_a_approach = copy.deepcopy(handover_b)
    handover_a_approach.pose.position.y -= 0.05
    handover_a_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, pi/2))
    
    self.go_to_pose_goal("b_bot", handover_b, speed=0.2)
    self.go_to_pose_goal("a_bot", handover_a_approach, speed=0.12)

    # Applied to the a_bot in workspace_center coordinates
    magic_x_offset = .001  
    magic_z_offset = .001

    handover_a  = copy.deepcopy(handover_a_approach)
    handover_a.pose.position.y += 0.05
    handover_a.pose.position.y += 0.023  # This is how much the gripper is pushed in
    handover_a.pose.position.x += magic_x_offset # MAGIC
    handover_a.pose.position.z += magic_z_offset # MAGIC

    self.send_gripper_command(gripper="a_bot", command="close")
    self.go_to_pose_goal("a_bot", handover_a, speed=0.03, move_lin= True)
    self.confirm_to_proceed("Is the gripper centered and in the end cap?")
    self.horizontal_spiral_motion("a_bot", .003, radius_increment = .001)
    self.send_gripper_command(gripper="a_bot", command="open")
      
    handover_b_retreat = copy.deepcopy(handover_a)
    handover_b_retreat.pose.position.y -= 0.017
    handover_b_retreat.pose.position.x -= magic_x_offset # MAGIC
    handover_b_retreat.pose.position.z -= magic_z_offset #MAGIC
    handover_b_retreat.pose.position.y += 0.03
    handover_b_retreat.pose.orientation = handover_b.pose.orientation
      
    self.send_gripper_command(gripper="b_bot", command="open")
    rospy.sleep(1.0)
    self.go_to_pose_goal("b_bot", handover_b_retreat, speed=0.02, move_lin= True)
    self.go_to_named_pose("back", "b_bot")
    self.go_to_named_pose("home", "a_bot")

  def handover_and_place_idle_pulley_in_a_safe_place(self):
    self.go_to_named_pose("pulley_push_ready_high", "c_bot",speed=self.speed_fast, acceleration=self.acc_fast)
    self.send_gripper_command(gripper="c_bot",command = "open", wait=False)

    b_bot_original_pose = geometry_msgs.msg.PoseStamped()
    b_bot_original_pose.header.frame_id = "intermediate_assy_part_14_screw_head"
    b_bot_original_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    b_bot_original_pose.pose.position.x = 0.
    b_bot_original_pose.pose.position.z = -0.4
    self.go_to_pose_goal("b_bot", b_bot_original_pose,speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)

    b_bot_face_up_near_c = geometry_msgs.msg.PoseStamped()
    b_bot_face_up_near_c.header.frame_id = "intermediate_assy_part_14_screw_head"
    b_bot_face_up_near_c.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    b_bot_face_up_near_c.pose.position.x = 0.
    b_bot_face_up_near_c.pose.position.y = 0.4
    b_bot_face_up_near_c.pose.position.z = -0.4
    self.go_to_pose_goal("b_bot", b_bot_face_up_near_c,speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)

    c_above_pin = geometry_msgs.msg.PoseStamped()
    c_above_pin.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, 0, -pi/2))
    c_above_pin.header.frame_id = "b_bot_robotiq_85_tip_link"
    c_above_pin.pose.position.x = 0.04
    c_above_pin.pose.position.y = -0.015
    
    c_pick = copy.deepcopy(c_above_pin)
    c_pick.pose.position.x = 0.03

    self.go_to_pose_goal("c_bot", c_above_pin,speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)
    self.go_to_pose_goal("c_bot", c_pick, speed=0.01, move_lin = True)
    self.confirm_to_proceed("Is the pick at the right height?")
    # self.go_to_pose_goal("c_bot", c_above_pin,speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)
    # self.confirm_to_proceed("Did the push work? c_bot will move away")

    self.send_gripper_command(gripper="c_bot",command = "close", wait=True)
    rospy.sleep(0.5)
    self.send_gripper_command(gripper="b_bot",command = "open", wait=True)
    self.send_gripper_command(gripper="b_bot",command = "open", wait=True)
    b_bot_face_sky_pose = copy.deepcopy(b_bot_face_up_near_c)
    b_bot_face_sky_pose.pose.position.y = 0.0
    self.go_to_pose_goal("b_bot", b_bot_face_sky_pose,speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)
    self.confirm_to_proceed("Did the pick succeed and b_bot move away?")

    place_joint_pose1 = [2.426513910293579, -1.534126106892721, 1.955094814300537, -3.1913464705096644, -3.1729450861560267, 0.38527336716651917]
    place_joint_pose2 = [2.0771114826202393, -0.8511574904071253, 1.8778533935546875, -2.630984608327047, -3.112194840108053, 1.5006953477859497]
    self.move_joints("c_bot", place_joint_pose1)
    self.move_joints("c_bot", place_joint_pose2)
    self.send_gripper_command(gripper="c_bot",command = "open", wait=True)
    rospy.sleep(1.0)
    self.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)


  def place_end_cap(self):
    rospy.logerr("function place_end_cap is not implemented.aborting")
    return
    self.place_joshua("a_bot", self.place_poses[i-1],self.item_place_heights[i-1],
                              speed_fast = self.speed_fast, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                              approach_height = 0.05, lift_up_after_place = False)

    #  self.send_gripper_command(gripper="precision_gripper_inner", command="open")
    self.horizontal_spiral_motion("a_bot", .001, radius_increment = .001)
    p = copy.deepcopy(self.place_poses[i-1])
    p.pose.position.z += 0.02
    self.go_to_pose_goal("a_bot", p, speed=0.02, move_lin= True)
    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

  def pick_clamping_shaft(self):
    self.log_to_debug_monitor("Pick clamping shaft", "operation")
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_partition_1"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = -.025
    pose0.pose.position.y = -0.05
    pose0.pose.position.z = 0.0
    self.send_gripper_command(gripper="b_bot",command = "open")
    self.pick_joshua("b_bot", pose0, grasp_height=0.03, speed_fast=self.speed_fast, speed_slow=.1, gripper_command="", approach_height=.07)
    # self.send_gripper_command(gripper="b_bot",command = "close")
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    return

  def insert_bearing(self):
    self.log_to_debug_monitor("Insert bearing", "operation")
    pre_insertion = geometry_msgs.msg.PoseStamped()
    pre_insertion.header.frame_id = "assembled_assy_part_11_front_hole"
    pre_insertion.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    pre_insertion.pose.position.x = -0.04
    self.go_to_pose_goal("b_bot", pre_insertion, speed=.3, move_lin = True)

  def subtask_f(self):
    # rospy.loginfo("======== SUBTASK F (motor plate) ========")
    self.log_to_debug_monitor("SUBTASK F (motor plate)", "subtask")
    self.log_to_debug_monitor("=== Subtask F start ===", "operation")
    self.place_plate_2_and_screw()
    self.log_to_debug_monitor("=== Subtask F end ===", "operation")

  def subtask_g(self):
    # ===== SUBTASK G (Placing and fastening the output (large) plate, for idle pulley set and clamping pulley set) =========
    # rospy.loginfo("======== SUBTASK G (large plate) ========")
    self.log_to_debug_monitor("SUBTASK G (large plate)", "subtask")
    self.log_to_debug_monitor("=== Subtask G (large plate) start ===", "operation")
    success = self.place_plate_3_and_screw()
    self.log_to_debug_monitor("=== Subtask G (large plate) end ===", "operation")
    return success

  def subtask_a(self, do_screwing=False):
    # ============= SUBTASK A (picking and inserting and fastening the motor shaft) =======================
    # rospy.loginfo("======== SUBTASK A (motor) ========")
    self.log_to_debug_monitor("SUBTASK A (motor)", "subtask")
    self.log_to_debug_monitor("=== Subtask A (motor) start ===", "operation")
    self.pick_motor()
    self.adjust_centering(go_fast=True, handover_motor_to_c_bot=True)
    self.insert_motor() # Joshua thinks this may be possible to do without impedance control, otherwise use insertion script in Y negative direction

    if do_screwing:
      self.do_change_tool_action("b_bot", equip=True, screw_size=3)
      self.go_to_named_pose("screw_ready", "b_bot")
      for i in range(1,7):
        self.fasten_motor_screw(screw_hole_number=i) # please ask Felix about the pick_screw function, I am not sure how he defined it
        self.do_change_tool_action("b_bot", equip=False, screw_size=3)
    self.log_to_debug_monitor("=== Subtask A (motor) end ===", "operation")

  def subtask_b(self):
    # ================================= SUBTASK B (motor pulley) ===========================================
    # rospy.loginfo("======== SUBTASK B (motor pulley) ========")
    self.log_to_debug_monitor("SUBTASK B (motor pulley)", "subtask")
    self.log_to_debug_monitor("=== Subtask B (motor pulley) start ===", "operation")
    self.pick_motor_pulley()
    self.insert_motor_pulley()
    rospy.loginfo("todo: fasten motor pulley") # With the set screw
    self.log_to_debug_monitor("=== Subtask B (motor pulley) end ===", "operation")
    return

  def subtask_e(self, pick_from_holder=False, exit_before_spacer=False, force_continue_task_to_the_end=False, exit_on_level_2=False):
    ### ======================== SUBTASK E (The idler pin) ============================================
    rospy.loginfo("======== SUBTASK E ========")
    self.log_to_debug_monitor("SUBTASK E (idler pin)", "subtask")
    self.log_to_debug_monitor("=== Subtask E (idler pin) start ===", "operation")

    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.confirm_to_proceed("pick_retainer_pin_nut")
    if not exit_on_level_2:
      self.pick_retainer_pin_nut()
      nut_picked = self.check_pick()
      self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      if nut_picked:
        self.confirm_to_proceed("place_retainer_pin_nut_on_table")
        self.place_retainer_pin_nut_on_table()
    
    nut_picked = False
    
    self.confirm_to_proceed("pick_idle_pulley")
    self.pick_idle_pulley()
    # if not self.check_pick():
    #   self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    #   self.reset_idle_pulley()
    #   self.pick_idle_pulley()
    if pick_from_holder:
      # ====== (This would have to be done before.)
      # self.confirm_to_proceed("pick_retainer_pin_from_tray_and_place_in_holder")
      # self.pick_retainer_pin_from_tray_and_place_in_holder()
      # ====== 
      self.confirm_to_proceed("pick_retainer_pin_from_holder")
      self.pick_retainer_pin_from_holder()
      self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.confirm_to_proceed("adjust_centering")
      self.adjust_centering(go_fast=True)
    else:
      self.confirm_to_proceed("pick_retainer_pin_from_tray")
      self.pick_retainer_pin_from_tray_and_keep_it()      
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.confirm_to_proceed("rotate_hand_facing_the_sky")
    self.rotate_hand_facing_the_sky()
    self.confirm_to_proceed("place_idle_pulley")
    self.place_idle_pulley()
    self.confirm_to_proceed("push idle pulley")
    self.push_idler_pulley_down()
    if exit_before_spacer:
      self.go_to_named_pose("suction_ready_back", "b_bot")
      self.send_gripper_command("b_bot", "open")
      rospy.sleep(1.0)
      return True
    self.confirm_to_proceed("pick_retainer_pin_spacer")
    self.pick_retainer_pin_spacer()
    spacer_picked = self.check_pick()
    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    if not spacer_picked:
      self.log_to_debug_monitor("Nut was not picked. Placing the pulley to exit the task.", "operation")
      self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.handover_and_place_idle_pulley_in_a_safe_place()
      rospy.sleep(1.0)
      return False
    self.confirm_to_proceed("place_retainer_pin_spacer")
    self.place_retainer_pin_spacer()
    self.confirm_to_proceed("pick_retainer_pin_washer(1)")
    self.pick_retainer_pin_washer(1)
    washer_picked = self.check_pick()
    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    if not washer_picked:
      self.log_to_debug_monitor("Nut was not picked. Placing the pulley to exit the task.", "operation")
      self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.handover_and_place_idle_pulley_in_a_safe_place()
      return False
    self.confirm_to_proceed("place_retainer_pin_washer_1")
    self.place_retainer_pin_washer_1()
    
    if not nut_picked:
      self.log_to_debug_monitor("Nut was not picked. Placing the pulley to exit the task.", "operation")
      self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.handover_and_place_idle_pulley_in_a_safe_place()
      return False
    
    if exit_on_level_2:
      self.log_to_debug_monitor("Nut was not picked. Placing the pulley to exit the task.", "operation")
      self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
      self.handover_and_place_idle_pulley_in_a_safe_place()
      return False
    self.confirm_to_proceed("pick_retainer_pin_nut_from_table")
    self.pick_retainer_pin_nut_from_table()
    self.confirm_to_proceed("pick_retainer_pin_washer")
    self.pick_retainer_pin_washer(2)
    second_washer_picked = self.check_pick()
    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    if second_washer_picked:
      self.confirm_to_proceed("place_retainer_pin_washer_on_table")
      self.place_retainer_pin_washer_on_table()
    self.confirm_to_proceed("insert_retainer_pin_to_base")
    self.insert_retainer_pin_to_base()
    self.confirm_to_proceed("hold_idle_pulley_with_a_bot")
    self.hold_idle_pulley_with_a_bot()
    self.confirm_to_proceed("release_and_push_with_b_bot")
    self.release_and_push_with_b_bot()
    self.confirm_to_proceed("release_idle_pulley_from_a_bot")
    self.release_idle_pulley_from_a_bot()
    if second_washer_picked:
      self.confirm_to_proceed("pick_retainer_pin_washer_from_table")
      self.pick_retainer_pin_washer_from_table()
      self.confirm_to_proceed("place_retainer_pin_washer_final")
      self.place_retainer_pin_washer_final()
    self.confirm_to_proceed("fasten_retainer_pin_nut")
    self.fasten_retainer_pin_nut()
    self.confirm_to_proceed("Unequip nut tool")
    self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    # self.go_to_named_pose("tool_pick_ready", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.do_change_tool_action("c_bot", equip=False, screw_size=66)
    self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    self.confirm_to_proceed("Move b_bot back home")
    b_bot_retreat = geometry_msgs.msg.PoseStamped()
    b_bot_retreat.header.frame_id = "assembled_assy_part_14_screw_head"
    b_bot_retreat.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    b_bot_retreat.pose.position.x = -0.08
    b_bot_retreat.pose.position.z = 0.015  # Offset 
    
    self.go_to_pose_goal("b_bot", b_bot_retreat, speed=self.speed_fast, acceleration=self.acc_fast, move_lin = True)
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    self.log_to_debug_monitor("=== Subtask E end ===", "operation")
    return True

  def subtask_c(self):
    # ==== SUBTASK C (clamping pulley set, everything but inserting and fastening clamping pulley) =================
    # rospy.loginfo("======== SUBTASK C (bearing + shaft) ========")
    self.log_to_debug_monitor("SUBTASK C (bearing + shaft)", "subtask")
    self.log_to_debug_monitor("=== Subtask C start ===", "operation")

    self.pick_clamping_shaft()
    self.adjust_centering(go_fast=True)

    self.insert_clamping_shaft()   

    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.log_to_debug_monitor("=== Subtask C end ===", "operation")


  def real_assembly_task(self):
    self.start_task_timer()
    self.log_to_debug_monitor(text="Assembly", category="task")
    self.picked_screw_counter["m4"] = 1
    self.picked_screw_counter["m3"] = 1

    self.subtask_c() # bearing shaft insertion

    # To prepare subtask E
    self.pick_retainer_pin_from_tray_and_place_in_holder(do_centering=False)
    self.confirm_to_proceed("press enter to proceed")
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    ## Equip screw tool for subtasks G, F
    self.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.do_change_tool_action("b_bot", equip=True, screw_size=4)
    self.go_to_named_pose("screw_pick_ready", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    self.confirm_to_proceed("press enter to proceed to subtask_g")
    self.subtask_g()  # Large plate
    self.confirm_to_proceed("press enter to proceed to subtask_f")
    self.subtask_f() # motor plate

    assy.go_to_named_pose("back", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=assy.use_real_robot)
    assy.do_change_tool_action("b_bot", equip=False, screw_size=4)

    self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    self.subtask_e(pick_from_holder=True) #idle pulley

    self.go_to_named_pose("home", "c_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("home", "b_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("home", "a_bot", speed=self.speed_fastest, acceleration=self.acc_fastest, force_ur_script=self.use_real_robot)

    self.subtask_a() # motor
    
    # self.subtask_c() # bearing, clamping pulley set
    return

if __name__ == '__main__':
  try:
    rospy.loginfo("Please refer to this page for details of each subtask. https://docs.google.com/spreadsheets/d/1Os2CfH80A7vzj6temt5L8BYpLvHKBzWT0dVuTvpx5Mk/edit#gid=1216221803")
    assy = AssemblyClass()
    assy.set_up_item_parameters()
    i = 1
    while i:
      rospy.loginfo("Enter 11 (12) to equip (unequip) m4 tool (b_bot).")
      rospy.loginfo("Enter 13 (14) to equip (unequip) m3 tool (b_bot).")
      rospy.loginfo("Enter 15 (16) to equip (unequip) m6 nut tool (c_bot).")
      rospy.loginfo("Enter 2 to move the robots home to starting positions.")
      rospy.loginfo("Enter 30 to pick screw m3 from tray with b_bot (31-36 for numbers 1-6).")
      rospy.loginfo("Enter 41-49 to pick screw m4 from tray with b_bot (number 1-9).")
      rospy.loginfo("Enter 50 to face the sky with b_bot.")
      rospy.loginfo("Enter 51 to test the pulley push motion.")
      rospy.loginfo("Enter 52 to go to screw_plate_ready.")
      rospy.loginfo("Enter 53 to recenter plate 3 (as if after a screw failure).")
      rospy.loginfo("Enter 55 to pick all subtask_e items with a_bot.")
      rospy.loginfo("Enter 56 to pick motor pulley.")
      rospy.loginfo("Enter 91-94 for subtasks (Large plate, motor plate, idler pin, motor).")
      rospy.loginfo("Enter 95-98 for subtasks (motor pulley, bearing+shaft, clamp pulley, belt).")
      rospy.loginfo("Enter 911 to place plate 3 (but don't screw)")
      rospy.loginfo("Enter 912 to place plate 3 from the base plate and put it on the table")
      rospy.loginfo("Enter 913 to screw in plate 3 (but don't place it)")
      rospy.loginfo("Enter 92 to place plate 2 and screw")
      rospy.loginfo("Enter 93 to do idle pulley set")
      rospy.loginfo("Enter 931 to do the idle pulley set directly from the tray")
      rospy.loginfo("Enter 932 to do the idle pulley set until the spacer (completion level 1)")
      rospy.loginfo("Enter 934 to do the idle pulley set until completion level 2")
      rospy.loginfo("Enter 94 to do the motor subtask")
      rospy.loginfo("Enter 95 to do the bearing")
      rospy.loginfo("Enter START to start the task.")
      rospy.loginfo("Enter x to exit.")
      i = raw_input()
      if i == '11':
        assy.go_to_named_pose("back", "c_bot", speed=assy.speed_fastest, acceleration=assy.acc_fastest, force_ur_script=assy.use_real_robot)
        assy.do_change_tool_action("b_bot", equip=True, screw_size=4)
      if i == '12':
        assy.go_to_named_pose("back", "c_bot", speed=assy.speed_fastest, acceleration=assy.acc_fastest, force_ur_script=assy.use_real_robot)
        assy.do_change_tool_action("b_bot", equip=False, screw_size=4)
      if i == '13':
        assy.go_to_named_pose("back", "c_bot", speed=assy.speed_fastest, acceleration=assy.acc_fastest, force_ur_script=assy.use_real_robot)
        assy.do_change_tool_action("b_bot", equip=True, screw_size=3)
      if i == '14':
        assy.go_to_named_pose("back", "c_bot", speed=assy.speed_fastest, acceleration=assy.acc_fastest, force_ur_script=assy.use_real_robot)
        assy.do_change_tool_action("b_bot", equip=False, screw_size=3)
      if i == '15':
        assy.go_to_named_pose("tool_pick_ready", "c_bot", speed=assy.speed_fastest, acceleration=assy.acc_fastest, force_ur_script=assy.use_real_robot)
        assy.do_change_tool_action("c_bot", equip=True, screw_size=66)
      if i == '16':
        assy.go_to_named_pose("tool_pick_ready", "c_bot", speed=assy.speed_fastest, acceleration=assy.acc_fastest, force_ur_script=assy.use_real_robot)
        assy.do_change_tool_action("c_bot", equip=False, screw_size=66)
      if i == '2':
        assy.go_to_named_pose("back", "a_bot", speed=assy.speed_fastest, acceleration=assy.acc_fastest, force_ur_script=assy.use_real_robot)
        assy.go_to_named_pose("home", "c_bot", speed=assy.speed_fastest, acceleration=assy.acc_fastest, force_ur_script=assy.use_real_robot)
        assy.go_to_named_pose("home", "b_bot", speed=assy.speed_fastest, acceleration=assy.acc_fastest, force_ur_script=assy.use_real_robot)
      if i == '30':
        assy.go_to_named_pose("screw_pick_ready", "b_bot")
        assy.pick_screw("b_bot", screw_size=3, screw_number="auto")
        assy.go_to_named_pose("screw_pick_ready", "b_bot")
      elif i in ['31', '32', '33', '34', '35', '36']:
        assy.go_to_named_pose("screw_pick_ready", "b_bot")
        assy.pick_screw("b_bot", screw_size=3, screw_number=int(i)-30)
        assy.go_to_named_pose("screw_pick_ready", "b_bot")
      if i == '40':
        assy.go_to_named_pose("screw_pick_ready", "b_bot")
        assy.pick_screw("b_bot", screw_size=4, screw_number="auto")
        assy.go_to_named_pose("screw_pick_ready", "b_bot")
      elif i in ['41', '42', '43', '44', '45', '46', '47', '48', '49']:
        assy.go_to_named_pose("screw_pick_ready", "b_bot")
        assy.pick_screw("b_bot", screw_size=4, screw_number=int(i)-40)
        assy.go_to_named_pose("screw_pick_ready", "b_bot")
      elif i == '50':
        assy.rotate_hand_facing_the_sky()
      elif i == '51':
        assy.push_idler_pulley_down()
      elif i == '52':
        assy.go_to_named_pose("screw_plate_ready", "b_bot")
      elif i == '53':
        assy.recenter_plate_3_on_base_plate()
      elif i == "55":
        assy.pick_retainer_pin_washer(1)
        assy.pick_retainer_pin_washer(2)
        assy.pick_retainer_pin_spacer()
        assy.pick_retainer_pin_nut()
      elif i == '56':
        assy.pick_motor_pulley()
      elif i == '91':
        assy.subtask_g()  # Large plate
      elif i == '911':
        assy.place_plate_3_and_screw(place_plate_only=True)
      elif i == '912':
        assy.place_plate_3_and_screw(reverse_placement_only=True)
      elif i == '913':
        assy.place_plate_3_and_screw(screw_first_only=True)
      elif i == '92':
        assy.subtask_f()  # Motor plate
      elif i == '93':
        assy.subtask_e(pick_from_holder=True)  # Idler pin
      elif i == '931':
        assy.subtask_e(pick_from_holder=False) # Idler pin without retainer pin holder
      elif i == '932':
        assy.subtask_e(pick_from_holder=False, exit_before_spacer=True) # Idler pin without retainer pin holder
      elif i == '933':
        assy.subtask_e(pick_from_holder=False, exit_before_spacer=True, force_continue_task_to_the_end=False) # Idler pin without retainer pin holder
      elif i == '934':
        assy.subtask_e(pick_from_holder=False, exit_on_level_2=True, force_continue_task_to_the_end=False) # Idler pin without retainer pin holder
      elif i == '94':
        assy.subtask_a() #
      elif i == '95':
        assy.subtask_c() #
      elif i == "2306":
        assy.handover_and_place_idle_pulley_in_a_safe_place()
      elif i == "2307":
        assy.pick_retainer_pin_washer(1)
        assy.place_retainer_pin_washer_on_table()
        assy.pick_retainer_pin_washer_from_table()
      elif i == 'START' or i == 'start' or i == "9999":
        for i in [1,2]:
          rospy.loginfo("Starting set number " + str(i))
          assy.real_assembly_task()
          rospy.loginfo("SET NUMBER " + str(i) + " COMPLETED. PUT THE ROBOT IN PAUSE MODE AND REPLACE THE PARTS")
          raw_input()
          if rospy.is_shutdown():
            rospy.loginfo("ABORTING")
            break
      elif i == "new":
        rospy.loginfo("SET NUMBER " + str(i) + " COMPLETED. PUT THE ROBOT IN PAUSE MODE AND REPLACE THE PARTS")
        raw_input()
        if rospy.is_shutdown():
          rospy.loginfo("ABORTING")
          break
        rospy.loginfo("Starting new set")
        assy.real_assembly_task()
      elif i == 'x':
        break
      elif i == "":
        continue
  except rospy.ROSInterruptException:
    pass

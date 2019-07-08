#!/usr/bin/env python

import csv
import sys
import os
import copy
import rospy
import rospkg
rospack = rospkg.RosPack()

# import tf
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
import moveit_msgs.msg
import geometry_msgs.msg
from math import radians, degrees

from omron_cad_matching.srv import *
from o2as_cad_matching.util import *
from o2as_cad_matching.client import *
from o2as_cad_matching.camera_adapter import CameraAdapter
from o2as_routines.base import O2ASBaseRoutines
from o2as_calibration.calibrator import HandeyeCalibrator

# Poses taken during handeye calibration
# TODO: These poses are specific for `d_bot` camera, need to modify for Phoxi
keyposes = {
  'a_phoxi_m_camera': {
    'a_bot': [
      [0.245, 0.3, 0.30, radians(  0), radians(  0), radians(90)],
      [0.245, 0.3, 0.30, radians(  0), radians( 30), radians(90)],
      [0.245, 0.3, 0.30, radians( 30), radians( 30), radians(90)],
      [0.245, 0.3, 0.30, radians( 30), radians(  0), radians(90)],
      [0.245, 0.3, 0.30, radians(-30), radians(  0), radians(90)],
      [0.245, 0.3, 0.30, radians(-30), radians( 30), radians(90)],
    ],

    'b_bot': [
      [-0.32, 0.40, 0.15, radians( 30), radians( 25), radians(180)],
      [-0.32, 0.50, 0.15, radians( 30), radians( 25), radians(180)],
      [-0.35, 0.60, 0.15, radians(  0), radians( 25), radians(180)],

      [-0.35, 0.60, 0.25, radians(  0), radians( 25), radians(180)],
      [-0.32, 0.50, 0.25, radians( 30), radians( 25), radians(180)],
      [-0.32, 0.40, 0.25, radians( 30), radians( 25), radians(180)],

      [-0.20, 0.30, 0.15, radians( 30), radians( 25), radians(180)],
      [-0.20, 0.40, 0.15, radians( 30), radians( 25), radians(180)],
      [-0.20, 0.50, 0.15, radians(  0), radians( 25), radians(180)],

      [-0.15, 0.60, 0.15, radians(  0), radians( 25), radians(180)],
      [-0.15, 0.45, 0.15, radians( 30), radians( 25), radians(180)],
      [-0.15, 0.30, 0.15, radians( 30), radians( 25), radians(180)],
    ],

    'c_bot': [
      [-0.110,  0.433,  0.386, -0.900,  0.000,  0.000],
      [-0.110,  0.415,  0.351, -0.570,  0.000,  0.000],
      [-0.143,  0.424,  0.365, -0.699,  0.559, -0.428]
    ]
  }
  'a_bot_camera': {
    'a_bot': [
      [0.245, 0.3, 0.30, radians(  0), radians(  0), radians(90)],
      [0.245, 0.3, 0.30, radians(  0), radians( 30), radians(90)],
      [0.245, 0.3, 0.30, radians( 30), radians( 30), radians(90)],
      [0.245, 0.3, 0.30, radians( 30), radians(  0), radians(90)],
      [0.245, 0.3, 0.30, radians(-30), radians(  0), radians(90)],
      [0.245, 0.3, 0.30, radians(-30), radians( 30), radians(90)],
    ],
  }
}

######################################################################
#  class CalibrationCommander                                        #
######################################################################

class CalibrationCommander(object):
  """Wrapper of MoveGroupCommander specific for this script"""
  def __init__(self, camera_name, robot_name):
    rospy.loginfo("start calibration")
    self.samples = []
    self.camera_name = camera_name
    self.eye_on_hand = rospy.get_param('~eye_on_hand', False)
    # rospy.logdebug("eye_on_hand: " + str(self.eye_on_hand))

    ## Initialize `moveit_commander`
    self.baseRoutines = O2ASBaseRoutines()
    self.robot_name   = robot_name
    group = self.baseRoutines.groups[robot_name]

    # Set `_ee_link` as end effector wrt `_base_link` of the robot
    group.set_pose_reference_frame(robot_name + "_base_link")
    # group.set_end_effector_link(robot_name    + "_ee_link")
    group.set_end_effector_link(robot_name    + "_ar_marker")

    # Trajectory publisher
    display_trajectory_publisher = rospy.Publisher(
      '/move_group/display_planned_path',
      moveit_msgs.msg.DisplayTrajectory,
      queue_size=20
    )

    # prepare for cad matching
    self.camera = CameraAdapter(camera_name, "phoxi")
    self.cad_matching = CadMatchingClient()
    self.cad_matching.select_camera(camera_name)
    self.cad_matching.select_object(str(102)) # calibration target object id is 102
    self.cad_matching.lib.set_search_range(300,900)
    self.cad_matching.lib.set_thresh_search(45)
    self.cad_matching.lib.set_max_result_num(1,1) # detect only one target object
    self.save_frame = ros_service_proxy("/data_collection_server/save_frame", SaveFrame)

    # self.listener = tf.TransformListener()
    self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    self.robot_base_frame = rospy.get_param("~robot_base_frame")
    self.robot_effector_frame = rospy.get_param("~robot_effector_frame")

    # Logging
    print("============ Reference frame: %s" % group.get_planning_frame())
    print("============ End effector: %s"    % group.get_end_effector_link())

  def go_home(self):
    self.baseRoutines.go_to_named_pose("home", self.robot_name)

  def move_to_subposes(self, pose_id, keypose, speed, sleep_time):
    pose = keypose
    for i in range(3):
      print("\n--- Subpose [{}/5]: Try! ---".format(i+1))
      self.move(pose, speed)
      print("--- Subpose [{}/5]: Completed. ---".format(i+1))
      if self.take_sample(sleep_time) is False:
        rospy.logwarn("take sample failed at pose_{}_{}".format(pose_id, i))
        raw_input("next pose?")
      pose[3] -= radians(30)
    pose[3]  = radians(0)
    pose[4] += radians(15)

    for i in range(2):
      print("\n--- Subpose [{}/5]: Try! ---".format(i+4))
      self.move(pose, speed)
      print("--- Subpose [{}/5]: Completed. ---".format(i+4))
      if self.take_sample(sleep_time) is False:
        rospy.logwarn("take sample failed at pose_{}_{}".format(pose_id, i+3))
        raw_input("next pose?")
      pose[4] -= radians(30)
    #raw_input("next keypose?")
    
  def move(self, pose, speed):
    """Move the end effector"""
    # TODO: check type of `pose`
    print("move to {}".format(pose))
    poseStamped                 = geometry_msgs.msg.PoseStamped()
    poseStamped.header.frame_id = self.robot_name + "_base_link"
    poseStamped.pose.position.x = pose[0]
    poseStamped.pose.position.y = pose[1]
    poseStamped.pose.position.z = pose[2]
    poseStamped.pose.orientation = geometry_msgs.msg.Quaternion(
      *tf_conversions.transformations.quaternion_from_euler(pose[3], pose[4], pose[5]))
    self.baseRoutines.go_to_pose_goal(self.robot_name, poseStamped, speed, move_lin=False)

  def take_sample(self, sleep_time):
    transforms = self.get_transforms(sleep_time)
    if transforms is not None:
      self.samples.append(transforms)
      return True
    return False

  def get_transforms(self, sleep_time):
    opt = self.get_opt_transform()
    rospy.sleep(sleep_time) # sleep may not necessary because get opt takes time
    rob = self.get_rob_transform()
    if rob is not None and opt is not None:
      # rospy.logdebug("rob: " + str(rob))
      # rospy.logdebug("opt: " + str(opt))
      return {'robot': rob, 'optical': opt}
    return None

  def get_opt_transform(self):
    # capture frame from the camera and save to the files
    cloud, image = self.camera.get_frame()
    filename = self.camera_name + "_" + str(len(self.samples))
    image_dir = os.path.join(rospack.get_path("o2as_calibration"), "data/image")
    cloud_filename = os.path.join(image_dir, filename+".dat")
    image_filename = os.path.join(image_dir, filename+".png")
    self.save_frame(cloud, image, cloud_filename, image_filename)

    # Get marker pose using 3d cad matching
    res = self.cad_matching.search_objects(cloud_filename, image_filename, "")
    if len(res.objects) > 0:
      # Pose to Transform
      pose = res.objects[0].pose
      trans = geometry_msgs.msg.Vector3(pose.position.x, pose.position.y, pose.position.z)
      rot = geometry_msgs.msg.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
      return geometry_msgs.msg.Transform(trans, rot)
    return None

  def get_rob_transform(self):
    if self.eye_on_hand:
      transform = self.tf_buffer.lookup_transform(self.robot_base_frame, self.robot_effector_frame, rospy.Time(0))
    else:
      transform = self.tf_buffer.lookup_transform(self.robot_effector_frame, self.robot_base_frame, rospy.Time(0))
    return transform.transform

######################################################################
#  global functions                                                  #
######################################################################

def collect_data(camera_name, robot_name, speed, sleep_time):
  """Run handeye calibration for the specified robot (e.g., "b_bot")"""
  print("=== Calibration started for {} + {} ===".format(camera_name, robot_name))
  commander = CalibrationCommander(camera_name, robot_name)
  commander.go_home()

  # Collect samples over pre-defined poses
  keypose_list = keyposes[camera_name][robot_name]
  for i, keypose in enumerate(keypose_list):
    print("\n*** Keypose [{}/{}]: Try! ***".format(i+1, len(keypose_list)))
    commander.move_to_subposes(i, keypose, speed, sleep_time)
    print("*** Keypose [{}/{}]: Completed. ***".format(i+1, len(keypose_list)))
  commander.go_home()

  # Save data to csv file for debug
  image_dir = rospy.get_param("~image_dir")
  filename = os.path.join(image_dir, "calibration.csv")
  save_data(filename, commander.samples)
  return commander.samples

def save_data(filename, samples):
  rospy.loginfo("save data to " + str(filename))
  with open(filename, 'wb') as f:
      writer = csv.writer(f, delimiter=',')
      # header
      writer.writerow([
        "rob.t.x","rob.t.y","rob.t.z","rob.r.x","rob.r.y","rob.r.z","rob.r.w",
        "cam.t.x","cam.t.y","cam.t.z","cam.r.x","cam.r.y","cam.r.z","cam.r.w"])
      # samples
      for sample in samples:
        rob = sample["robot"]
        opt = sample["optical"]
        writer.writerow([
          str(rob.translation.x), str(rob.translation.y), str(rob.translation.z),
          str(rob.rotation.x), str(rob.rotation.y), str(rob.rotation.z), str(rob.rotation.w),
          str(opt.translation.x), str(opt.translation.y), str(opt.translation.z),
          str(opt.rotation.x), str(opt.rotation.y), str(opt.rotation.z), str(opt.rotation.w)])

def run(camera_name, robot_name, speed, sleep_time):
  # Collect data
  samples = collect_data(camera_name, robot_name, speed, sleep_time)

  # Calibration
  rospy.loginfo("run calibration")
  calibrator = HandeyeCalibrator()
  calibrator.set_transforms(samples)
  calibrator.compute_calibration()
  print("=== Calibration completed for {} + {} ===".format(camera_name, robot_name))
  
def main():
  try:
    camera_name = sys.argv[1]
    robot_name  = sys.argv[2]
    assert(camera_name in {"a_phoxi_m_camera", "c_bot_camera"})
    assert(robot_name  in {"a_bot", "b_bot", "c_bot"})
    run(camera_name, robot_name, speed=0.1, sleep_time=0.1)
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

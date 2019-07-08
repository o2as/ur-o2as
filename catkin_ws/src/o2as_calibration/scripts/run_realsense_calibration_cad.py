#!/usr/bin/env python

import csv
import sys
import os
import copy
import rospy
import rospkg
rospack = rospkg.RosPack()

# import tf
import tf
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Transform, Quaternion, Vector3
from math import radians, degrees

from omron_cad_matching.srv import *
from o2as_cad_matching.util import *
from o2as_cad_matching.client import *
from o2as_cad_matching.camera_adapter import CameraAdapter
from o2as_routines.base import O2ASBaseRoutines
from o2as_calibration.calibrator import HandeyeCalibrator

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from std_srvs.srv import Empty
from easy_handeye.srv import TakeSample, RemoveSample, ComputeCalibration


# Poses taken during handeye calibration
posess = {
  'a_bot_camera': {
    'a_bot': [
      [-0.127, 0.556, 0.432, -1.591,  0.147, -0.285],
      [-0.242, 0.641, 0.415, -1.593, -0.394, -0.273],
      [-0.264, 0.648, 0.457, -1.592, -0.418, -0.272],
      [-0.092, 0.522, 0.456, -1.592,  0.426, -0.290],
    ]
  }
}


# Initial poses are independent on cameras
init_poses = {
  'a_bot': [-0.108, 0.183, 0.707, -1.571, 0.000, 0.000],
}


class CalibrationCommander(object):
  """Wrapper of MoveGroupCommander specific for this script"""
  def __init__(self, robot_name, camera_name):
    ## Initialize `moveit_commander`
    self.index = 0
    moveit_commander.roscpp_initialize(sys.argv)  # TODO: see if it is required
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander(robot_name)

    # Set `_wrist_3_link` as end effector wrt `_base_link` of the robot
    ref_link = robot_name + "_base_link"
    ee_link = robot_name + "_wrist_3_link"  # the link AR marker is put on
    group.set_pose_reference_frame(ref_link)
    group.set_end_effector_link(ee_link)

    # Trajectory publisher
    _ = rospy.Publisher(
      '/move_group/display_planned_path',
      moveit_msgs.msg.DisplayTrajectory,
      queue_size=20
    )

    # Logging
    planning_frame = group.get_planning_frame()
    print("============ Reference frame: %s" % planning_frame)
    eef_link = group.get_end_effector_link()
    print("============ End effector: %s" % eef_link)
    group_names = robot.get_group_names()
    print("============ Robot Groups:", robot.get_group_names())
    # print("============ Printing robot state")
    # print(robot.get_current_state())
    print()

    self.camera_name = camera_name
    self.eye_on_hand = rospy.get_param('~eye_on_hand', False)
    rospy.logdebug("eye_on_hand: " + str(self.eye_on_hand))
    raw_input("check eye_on_hand")

    # prepare for cad matching
    self.camera = CameraAdapter(camera_name, "realsense")
    # self.marker_detector = MarkerDetection()
    self.cad_matching = CadMatchingClient()
    self.cad_matching.select_camera(camera_name)
    self.cad_matching.select_object(str(102)) # calibration target object id is 102
    self.cad_matching.lib.set_search_range(300,900)
    self.cad_matching.lib.set_thresh_search(45)
    self.cad_matching.lib.set_max_result_num(1,1) # detect only one target object
    self.save_frame = ros_service_proxy("/data_collection_server/save_frame", SaveFrame)
    # TF
    self.listener = tf.TransformListener()
    self.robot_base_frame = rospy.get_param("~robot_base_frame")
    self.robot_effector_frame = rospy.get_param("~robot_effector_frame")

    # Misc variables
    self.robot = robot
    self.group = group
    self.eef_link = eef_link
    self.group_names = group_names

  def move(self, pose, speed=0.1):
    """Move the end effector"""
    # TODO: check type of `pose`
    self.group.set_pose_target(pose)
    self.group.set_max_velocity_scaling_factor(speed)
    self.group.go(wait=True)
    self.group.clear_pose_targets()

  def take_sample(self):
    opt = self.get_opt_transform()
    rob = self.get_rob_transform()
    if rob is not None and opt is not None:
      print("rob: " + str(rob))
      print("opt: " + str(opt))
      raw_input("marker is detected.")
      return {'robot': rob, 'optical': opt}
    return None

  def get_opt_transform(self):
    # capture frame from the camera and save to the files
    cloud, image = self.camera.get_frame()
    filename = self.camera_name + "_" + str(self.index)
    image_dir = os.path.join(rospack.get_path("o2as_calibration"), "data/image")
    cloud_filename = os.path.join(image_dir, filename+".dat")
    image_filename = os.path.join(image_dir, filename+".png")
    self.save_frame(cloud, image, cloud_filename, image_filename)

    # Get marker pose using 3d cad matching
    res = self.cad_matching.search_objects(cloud_filename, image_filename, "")
    if len(res.objects) > 0:
      self.index = self.index + 1
      # Pose to Transform
      pose = res.objects[0].pose
      trans = geometry_msgs.msg.Vector3(pose.position.x, pose.position.y, pose.position.z)
      rot = geometry_msgs.msg.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
      return geometry_msgs.msg.Transform(trans, rot)
    return None

  def get_rob_transform(self):
    print("eye_on_hand: " + str(self.eye_on_hand))
    now = rospy.Time.now()
    print("wait for transform: " + "a_bot_base_link, a_bot_ee_link")
    self.listener.waitForTransform("a_bot_base_link", "a_bot_ee_link", now, rospy.Duration(10))
    transform = self.listener.lookupTransform("a_bot_base_link", "a_bot_ee_link", now)
    print("transform: " + str(transform))
    if self.eye_on_hand:
      print("wait for transform: " + self.robot_base_frame + ", " + self.robot_effector_frame)
      self.listener.waitForTransform(self.robot_base_frame, self.robot_effector_frame, now, rospy.Duration(10))
      transform = self.listener.lookupTransform(self.robot_base_frame, self.robot_effector_frame, now)
    else:
      print("wait for transform: " + self.robot_effector_frame + ", " + self.robot_base_frame)
      self.listener.waitForTransform(self.robot_effector_frame, self.robot_base_frame, now, rospy.Duration(10))
      transform = self.listener.lookupTransform(self.robot_effector_frame, self.robot_base_frame, now)
    trans = Transform(Vector3(*transform[0]), Quaternion(*transform[1]))
    return trans

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

def collect_data(camera_name, robot_name, speed, sleep_time):
  # Save data to csv file for debug

  return commander.samples

def run_calibration(camera_name, robot_name):
  """Run handeye calibration for the specified robot (e.g., "b_bot")"""
  # Initialize move group and service proxies
  mg = CalibrationCommander(robot_name, camera_name)
  print("=== Calibration started for {} ===".format(robot_name))

  # Collect samples over pre-defined poses
  mg.move(init_poses[robot_name])
  samples = []
  for i, pose in enumerate(posess[camera_name][robot_name]):
    mg.move(pose)
    sample = mg.take_sample()
    if sample is not None:
      samples.append(sample)
    rospy.sleep(1)  # Sleep for 1 seconds
  mg.move(init_poses[robot_name])

  # Save samples
  image_dir = rospy.get_param("~image_dir")
  filename = os.path.join(image_dir, "calibration.csv")
  save_data(filename, samples)

  # Calibration
  rospy.loginfo("run calibration")
  calibrator = HandeyeCalibrator()
  calibrator.set_transforms(samples)
  calibrator.compute_calibration()
  print("=== Calibration completed for {} + {} ===".format(camera_name, robot_name))


def main(camera_name, robot_name):
  try:
    rospy.init_node('run_calibration', anonymous=True, log_level=rospy.DEBUG)
    run_calibration(camera_name, robot_name)

  except rospy.ROSInterruptException:
    return

  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  camera_name = 'a_bot_camera'
  robot_name = 'a_bot'
  main(camera_name, robot_name)

#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from std_srvs.srv import Empty
from easy_handeye.srv import TakeSample, RemoveSample, ComputeCalibration


# Poses taken during handeye calibration
posess = {
  'realsense': {
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


def get_service_proxy(service_name, base_name):
  """Return ROS service proxy"""
  ns = "/o2as_easy_handeye_{}_eye_on_hand/".format(base_name)

  if service_name is "take_sample":
    service_type = TakeSample
    service_name_full = ns + "take_sample"
  elif service_name is "get_sample_list":
    service_type = TakeSample
    service_name_full = ns + "get_sample_list"
  elif service_name is "remove_sample":
    service_type = RemoveSample
    service_name_full = ns + "remove_sample"
  elif service_name is "compute_calibration":
    service_type = ComputeCalibration
    service_name_full = ns + "compute_calibration"
  elif service_name is "save_calibration":
    service_type = Empty
    service_name_full = ns + "save_calibration"
  else:
    raise NameError("Service name {} does not exist".format(service_name))

  return rospy.ServiceProxy(service_name_full, service_type)


class MoveGroupCommander(object):
  """Wrapper of MoveGroupCommander specific for this script"""
  def __init__(self, base_name):
    ## Initialize `moveit_commander`
    moveit_commander.roscpp_initialize(sys.argv)  # TODO: see if it is required
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander(base_name)

    # Set `_wrist_3_link` as end effector wrt `_base_link` of the robot
    ref_link = base_name + "_base_link"
    ee_link = base_name + "_wrist_3_link"  # the link AR marker is put on
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

    # Misc variables
    self.robot = robot
    self.group = group
    self.eef_link = eef_link
    self.group_names = group_names

  def move(self, pose):
    """Move the end effector"""
    # TODO: check type of `pose`
    self.group.set_pose_target(pose)
    self.group.go(wait=True)
    self.group.clear_pose_targets()


def run_calibration(camera_name, robot_name):
  """Run handeye calibration for the specified robot (e.g., "b_bot")"""
  # Initialize move group and service proxies
  mg = MoveGroupCommander(robot_name)
  take_sample = get_service_proxy("take_sample", robot_name)
  get_sample_list = get_service_proxy("get_sample_list", robot_name)
  remove_sample = get_service_proxy("remove_sample", robot_name)
  compute_calibration = get_service_proxy("compute_calibration", robot_name)
  save_calibration = get_service_proxy("save_calibration", robot_name)

  print("=== Calibration started for {} ===".format(robot_name))

  # Clear samples in the buffer if exist
  n_samples = len(get_sample_list().samples.hand_world_samples.transforms)
  if 0 < n_samples:
    for _ in range(n_samples):
      remove_sample(0)

  # Reset pose
  mg.move(init_poses[robot_name])

  # Collect samples over pre-defined poses
  for i, pose in enumerate(posess[camera_name][robot_name]):
    mg.move(pose)
    take_sample()
    rospy.sleep(1)  # Sleep for 1 seconds
    sample_list = get_sample_list()
    n1 = len(sample_list.samples.hand_world_samples.transforms)
    n2 = len(sample_list.samples.camera_marker_samples.transforms)
    print(("Loop {}, took {} hand_world samples and {} camera_marker " +
          "samples").format(i, n1, n2))

  # Compute and save calibration
  compute_calibration()
  save_calibration()

  # Reset pose
  mg.move(init_poses[robot_name])

  print("=== Calibration completed for {} ===".format(robot_name))


def main(camera_name, robot_name):
  try:
    rospy.init_node('run_calibration', anonymous=True, log_level=rospy.INFO)
    run_calibration(camera_name, robot_name)

  except rospy.ROSInterruptException:
    return

  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  # camera_name = sys.argv[1]
  # robot_name = sys.argv[2]
  # assert(robot_name in {"b_bot"})
  # assert(camera_name in {"realsense"})
  camera_name = 'realsense'
  robot_name = 'a_bot'
  main(camera_name, robot_name)

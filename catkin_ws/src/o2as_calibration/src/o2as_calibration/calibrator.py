import rospy
import tf
from tf import transformations as tfs
from geometry_msgs.msg import Vector3, Quaternion, Transform
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import compute_effector_camera_quick
from easy_handeye.handeye_calibration import HandeyeCalibration

class HandeyeCalibrator(object):
  MIN_SAMPLES = 2

  def __init__(self):
    self.samples = []
    # get params
    self.eye_on_hand = rospy.get_param('~eye_on_hand', False)
    self.robot_effector_frame = rospy.get_param('robot_effector_frame', 'tool0')
    self.robot_base_frame = rospy.get_param('robot_base_frame', 'base_link')
    self.tracking_base_frame = rospy.get_param('tracking_base_frame', 'optical_origin')
    self.tracking_marker_frame = rospy.get_param('tracking_marker_frame', 'optical_target')
    # service
    rospy.wait_for_service('compute_effector_camera_quick')
    self.calibrate = rospy.ServiceProxy('compute_effector_camera_quick', compute_effector_camera_quick)

  def set_transforms(self, transforms):
    self.samples = transforms

  def get_visp_samples(self):
    hand_world_samples = TransformArray()
    hand_world_samples.header.frame_id = self.tracking_base_frame # bug?
    camera_marker_samples = TransformArray()
    camera_marker_samples.header.frame_id = self.tracking_base_frame
    for s in self.samples:
      camera_marker_samples.transforms.append(s['optical'])
      hand_world_samples.transforms.append(s['robot'])
    return hand_world_samples, camera_marker_samples

  def compute_calibration(self):
    # check sample counts
    if len(self.samples) < HandeyeCalibrator.MIN_SAMPLES:
      rospy.logwarn("{} more samples needed! Not computing the calibration".format(
        HandeyeCalibrator.MIN_SAMPLES - len(self.samples)))
      return
    hand_world_samples, camera_marker_samples = self.get_visp_samples()
    if len(hand_world_samples.transforms) != len(camera_marker_samples.transforms):
      rospy.logerr("Different numbers of hand-world and camera-marker samples!")
      raise AssertionError

    # run calibration
    rospy.loginfo("Computing from %g poses..." % len(self.samples))
    try:
      rospy.loginfo("Eye on hand is: {}".format(self.eye_on_hand))
      result = self.calibrate(camera_marker_samples, hand_world_samples)
      rospy.loginfo("Computed calibration: {}".format(str(result)))
      transl = result.effector_camera.translation
      rot = result.effector_camera.rotation
      result_tuple = ((transl.x, transl.y, transl.z), (rot.x, rot.y, rot.z, rot.w))
      ret = HandeyeCalibration(self.eye_on_hand, self.robot_base_frame, self.robot_effector_frame, self.tracking_base_frame, result_tuple)
      return ret
    except rospy.ServiceException as ex:
      rospy.logerr("Calibration failed: " + str(ex))
      return None

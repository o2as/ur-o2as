#!/usr/bin/env python

import os
import rospy
import rospkg
rospack = rospkg.RosPack()

import numpy as np
from numpy import linalg as LA

import moveit_commander
from o2as_vision.phoxi_camera_adapter import PhoXiCamera
from o2as_vision.realsense_camera_adapter import RealSenseCamera
from o2as_cad_matching.client import *
from o2as_msgs.srv import *
from sensor_msgs.msg import Image

def ros_service_proxy(service_name, service_type):
  proxy = None
  try:
    rospy.logwarn("wait for service " + service_name)
    rospy.wait_for_service(service_name)
    rospy.logdebug("service " + service_name + " is ready.")
    proxy = rospy.ServiceProxy(service_name, service_type)
  except rospy.ServiceException as e:
    rospy.logerr("service error: %s", str(e)) 
  return proxy

class PartsInfo(object):
  def __init__(self, object_id, name, type, cad, description):
    self.object_id = object_id
    self.name = name
    self.type = type
    self.cad = cad
    self.description = description

class VisionServer(object):
  def __init__(self):
    try:
      # The directory where the image data for cad matching is saved into
      self._image_dir = rospy.get_param("~image_dir")

      # Get parts info
      self._parts_dict = dict()
      parts_list = rospy.get_param("/parts_list")
      for parts in parts_list:
        object_id = str(parts['id'])
        parts_info = PartsInfo(object_id=object_id, name=parts['name'], type=parts['type'], cad=parts['cad'], description=parts['description'])
        self._parts_dict[object_id] = parts_info

      # The cad matching service 
      self.cad_matching = CadMatchingClient()

      # Init cameras
      self._cameras = dict()
      self._cameras["phoxi"] = PhoXiCamera(name="phoxi")
      self._cameras["phoxi"].start()
      self._cameras["b_bot_camera"] = RealSenseCamera(name="b_bot_camera")
      self._cameras["b_bot_camera"].start()

      # # Planning scene interface is necessary to add detected object into the scene
      rospy.logdebug("waiting for the planning scene interface")
      self._planning_scene = moveit_commander.PlanningSceneInterface()

      # Start service server
      rospy.logdebug("start service %s","find_object")
      self._find_object_server = rospy.Service("~find_object", FindObject, self.find_object)

    except rospy.ServiceException, e:
      rospy.logerr("Service call failed: " + str(e))

  def get_parts_info(self, object_id):
    parts_info = self._parts_dict[str(object_id)]
    if parts_info is None:
      rospy.logerr("Failed to add mesh to the scene because information was not found: ", str(object_id))
    return parts_info

  def get_camera(self, name):
    return self._cameras[name]

  #########################################################
  # object detection and pose estimation using cad matching
  #########################################################

  def choose_nearest_object(self, objects, expected_position, tolerance):
    n = len(objects) 
    if n == 0:
      return None

    ix_closest = 0
    tolerance = float("inf")
    for i in range(n):
      obj = objects[i]
      expected_pos = np.array([expected_position.pose.position.x, expected_position.pose.position.y, expected_position.pose.position.z])
      seen_pos = np.array([obj.pose.position.x, obj.pose.position.y, obj.pose.position.z])
      dist = LA.norm(expected_pos-seen_pos)
      if dist < tolerance:
        tolerance = dist
        ix_closest = i
      rospy.logdebug("expected=({},{},{}), seen=({},{},{}), d={}".format(expected_pos[0], expected_pos[1], expected_pos[2], seen_pos[0], seen_pos[1], seen_pos[2], dist))

    return objects[ix_closest]

  def find_object(self, req):
    use_file = True
    rospy.logdebug("find object")
    rospy.logdebug("camera = %s", req.camera)
    rospy.logdebug("object_id = %s", req.object_id)
    rospy.logdebug("expected_position = (%f, %f, %f)", req.expected_position.pose.position.x, req.expected_position.pose.position.y, req.expected_position.pose.position.z)
    rospy.logdebug("position_tolerance = %f", req.position_tolerance)

    # get image data from the camera
    camera = self.get_camera(req.camera)
    self.cad_matching.select_camera(req.camera)
    self.cad_matching.select_object(req.object_id)

    if use_file:
      cloud_filename = os.path.join(self._image_dir, req.camera + ".dat")
      image_filename = os.path.join(self._image_dir, req.camera + ".png")
      camera.dump_frame(cloud_filename, image_filename)
      response = self.cad_matching.search_objects(cloud_filename, image_filename, "")
    else:
      cloud, texture = camera.get_frame()
      response = self.cad_matching.search_objects(cloud, texture, Image())

    # choose nearest object from expected position within tolerance
    # expected position should be specified with depth image frame of the camera
    objects = []
    if response.success:
      objects = response.objects
      rospy.logdebug("%d objects found. Selecting nearest.", len(objects))
    else:
      rospy.loginfo("no object found")
    obj = self.choose_nearest_object(objects, req.expected_position, req.position_tolerance)

    # prepare return value
    res = FindObjectResponse()
    pose_stamped = geometry_msgs.msg.PoseStamped()
    if obj:
      # change Pose to PoseStamped 
      # (cad matching always returns pose in depth image frame)
      pose_stamped.header.frame_id = camera.get_depth_image_frame()
      pose_stamped.pose = obj.pose
      res.success = True

      # Optional
      self.add_detected_object_to_planning_scene(obj.object_id, pose_stamped)
    else:
      res.success = False
    res.object_pose = pose_stamped
    return res

  def add_detected_object_to_planning_scene(self, object_id, pose):
    rospy.logdebug("add detected object to planning scene")

    parts_info = self.get_parts_info(object_id)
    if parts_info:
      name = parts_info.type+"_mesh"
      
      mesh_filename = os.path.join(rospack.get_path('o2as_parts_description'), "meshes", parts_info.cad)
      rospy.logdebug("mesh_filename:" + mesh_filename)
      scale = [0.001, 0.001, 0.001] # [mm to meter]
      self._planning_scene.add_mesh(name, pose, mesh_filename, scale)
      #self._planning_scene.add_box(name=name+"_box", pose=pose, size=(0.2, 0.2, 0.2)) # debug

    rospy.loginfo("Get frame request to the phoxi camera finished successfully.")

if __name__ == "__main__":
  rospy.init_node('o2as_vision', anonymous=True, log_level=rospy.DEBUG)
  node = VisionServer()
  rospy.spin()

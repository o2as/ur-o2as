#!/usr/bin/env python

import yaml
import numpy as np

import os
import rospy
import dynamic_reconfigure.server
import dynamic_reconfigure.client
from o2as_cad_matching.cfg import CadMatchingConfig

import tf
import sensor_msgs
import geometry_msgs
from o2as_cad_matching.msg import *
from o2as_cad_matching.srv import *
from o2as_cad_matching.util import *
from omron_cad_matching.search_client import SearchClient

# server class of o2as_cad_matching that provides object search functionality
class CadMatchingServerNode(SearchClient):
    def __init__(self):
        super(CadMatchingServerNode, self).__init__(server="search_server")

        # get params
        self.model_dir = rospy.get_param("~model_dir")
        self.setting_dir = rospy.get_param("~setting_dir")
        self.object_config_dir = rospy.get_param("~object_config_dir")
        self.mask_filename = ""

        # provice cad matching services as a server
        self._select_camera = rospy.Service("~select_camera", SelectCamera, self.select_camera)
        self._select_object = rospy.Service("~select_object", SelectObject, self.select_object)
        self._search_objects = rospy.Service("~search_objects", SearchObjects, self.search_objects)
        self._search_objects2 = rospy.Service("~search_objects2", SearchObjects2, self.search_objects2)

        # optional setter of parameters 
        self._set_search_area = rospy.Service("~set_search_area", SetSearchArea, self.set_search_area)
        self._set_mask_image = rospy.Service("~set_mask_image", SetMaskImage, self.set_mask_image)
        self._set_search_range = rospy.Service("~set_search_range", SetSearchRange, self.set_search_range)

        # start dynamic reconfigure server
        self.select_camera_impl(rospy.get_param("~camera"), force=True)
        self.select_object_impl(rospy.get_param("~object_id"), force=True)
        self.dynamic_reconfigure = dynamic_reconfigure.server.Server(CadMatchingConfig, self.dynamic_reconfigure_callback)
        rospy.loginfo("o2as_cad_matching_server is ready")

    ###############################################
    # dynamic reconfigure
    ###############################################

    def set_param(self, config, level):
        if level & (1 << 0):
            # select object
            self.select_object_impl(config.object_id)

    def dynamic_reconfigure_callback(self, config, level):
        self.set_param(config, level)
        return config

    ###############################################
    # select object
    ###############################################

    def select_object_impl(self, object_id, force=False):
        # note: object_id must be an integer
        rospy.loginfo("select object {}".format(object_id))
        last_object_id = rospy.get_param("~object_id")
        if last_object_id == object_id and not force:
            rospy.logwarn("same object is selected. do nothing")
            return True
        rospy.set_param("~object_id", object_id)

        # load trained model file for the object seen by the camera
        model_filename = ""
        if object_id in self.id_map.keys():
            model_filename = self.id_map[object_id]
            self.load_model_data(model_filename)
            rospy.loginfo("object " + str(object_id) + " is selected. model file is " + model_filename)
        else:
            rospy.logerr("object_id is not in the id table")
            return False
        return True

    def select_object(self, req):
        res = SelectObjectResponse()
        # TODO: needs to be changed to call dynamic reconfigure
        res.success = self.select_object_impl(int(req.object_id))
        return res

    ###############################################
    # select camera
    ###############################################

    def read_id_map(self, id_map_filename):
        # load dictionary from object_id to cad_matching model
        with open(id_map_filename, 'r') as f:
            data = yaml.load(f)
            id_map = data["id_map"]
            return id_map

    def select_camera_impl(self, camera, force=False):
        # Note : camera should be a string
        rospy.loginfo("select camera {}".format(camera))
        last_camera = rospy.get_param("~camera")
        if last_camera == camera and not force: 
            rospy.logwarn("same camera is selected. do nothing")
            return True
        rospy.set_param("~camera", camera)

        # Load dictionary of object_id to model_filename
        id_map_filename = os.path.join(self.model_dir, camera, "id_map.yaml")
        self.id_map = self.read_id_map(id_map_filename)

        # Set global search parameters and object search parameters
        search_param_filename = os.path.join(self.setting_dir, camera+"_search_param.yaml")
        self.read_search_params(search_param_filename)
        return True

    def select_camera(self, req):
        res = SelectCameraResponse()
        res.success = self.select_camera_impl(req.camera)
        return res

    ###############################################
    # search params (to be discontinued)
    ###############################################

    def set_search_area(self, req):
        res = SetSearchAreaResponse()
        self.set_search_area(req.left, req.top, req.right, req.bottom)
        return res

    def set_search_range(self, req):
        res = SetSearchRangeResponse()
        self.set_search_range(req.min_dist, req.max_dist)
        return res

    def set_mask_image(self, req):
        res = SetMaskImageResponse()
        self.mask_filename = req.filename
        return res

    ###############################################
    # search
    ###############################################

    def make_response(self, search_result, response):
        # convert search result of omron cad matching library to ros data
        n = search_result.result_num
        rospy.loginfo("cad matching : %d objects found", n)
        if (n > 0):
            for i in range(n):
                o = search_result.detected_objects[i]
                rospy.logdebug("object " + str(i))
                rospy.logdebug("score=({},{},{})".format(o.score2D, o.score3D, o.score_final))
                rospy.logdebug("pos=({},{},{})".format(o.pos3D.x, o.pos3D.y, o.pos3D.z))
                rospy.logdebug("rot=({},{},{})".format(o.rot3D[0], o.rot3D[1], o.rot3D[2]))
                rospy.logdebug("rot_mat: ")
                rospy.logdebug(o.rot_mat)
                # prepare message
                obj = o2as_cad_matching.msg.DetectedObject()
                obj.object_id    = search_result.detected_objects[i].object_id			
                obj.model_id     = search_result.detected_objects[i].model_id				
                obj.score_2d     = search_result.detected_objects[i].score2D
                obj.score_3d     = search_result.detected_objects[i].score3D
                obj.score_final  = search_result.detected_objects[i].score_final
                # object pose is in depth image frame of the camera (x,y,z) and depth_frame (X,Y,Z) = (z,-x,-y) 
                p = geometry_msgs.msg.Pose()
                # change unit of position from mm (cad matching) to meter (ros)
                p.position.x = search_result.detected_objects[i].pos3D.x * 0.001
                p.position.y = search_result.detected_objects[i].pos3D.y * 0.001
                p.position.z = search_result.detected_objects[i].pos3D.z * 0.001
                # change unit of rotation from deg (cad) to rad (ros)
                ax = np.deg2rad(search_result.detected_objects[i].rot3D[0])
                ay = np.deg2rad(search_result.detected_objects[i].rot3D[1])
                az = np.deg2rad(search_result.detected_objects[i].rot3D[2])
                # rotation matrix must be R = RxRyRz so needs to specify 'rxyz'
                q = tf.transformations.quaternion_from_euler(ax, ay, az, 'rxyz')
                p.orientation.x = q[0]
                p.orientation.y = q[1]
                p.orientation.z = q[2]
                p.orientation.w = q[3]
                obj.pose = p
                response.objects.append(obj)

    def search_objects(self, req):
        res = SearchObjectsResponse()
        try:
            rospy.logdebug("cad matching : call search")
            response = self.search(req.pcloud_filename, req.image_filename, self.mask_filename)
            self.make_response(response.search_result, res)
            res.success = True
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))
            res.success = False
        return res

    def search_objects2(self, req):
        res = SearchObjects2Response()
        try:
            rospy.logdebug("cad matching : call search")
            response = self.search2(req.pcloud, req.image, req.mask)
            self.make_response(response.search_result, res)
            res.success = True
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))
            res.success = False
        return res

if __name__ == "__main__":
    rospy.init_node('o2as_cad_matching_server', anonymous=True, log_level=rospy.DEBUG)
    node = CadMatchingServerNode()
    rospy.spin()

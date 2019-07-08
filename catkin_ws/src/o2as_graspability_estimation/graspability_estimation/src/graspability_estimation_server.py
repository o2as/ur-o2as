#!/usr/bin/env python

import copy
from skimage import io
import os
import numpy as np

from cv_bridge import CvBridge
import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import o2as_msgs.msg
from std_srvs.srv import *
import rospkg
rp = rospkg.RosPack()
import time
import datetime

from graspability_estimation.srv import *
from o2as_phoxi_camera.srv import *
import actionlib

LOG_LEVEL = log_level = rospy.INFO
GRASPABLE_POINT_NOT_FOUND_MSG = "ERROR: Graspable point is not found!"

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

class GraspabilityEstimation(object):
    def __init__(self):
        rospy.init_node("o2as_graspability_estimation", anonymous=True, log_level=LOG_LEVEL)

        # Publish topic
        self.pub_res_img = rospy.Publisher("search_grasp_phoxi/image_grasp_candidates", sensor_msgs.msg.Image, queue_size=1)
        self.bridge = CvBridge()

        # params
        self.bin_id = rospy.get_param("fge_bin_id")
        print("Read in these bin ids:")
        print(self.bin_id)
        self.camera_name = rospy.get_param("~camera_name")
        self.image_dir = rospy.get_param('~image_dir')
        self.cx = 0
        self.cy = 0
        self.fx = 0
        self.fy = 0

        ts = time.time()
        st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H:%M:%S')
        self.filename = self.image_dir + self.camera_name + "_" + st + ".tiff"

        self.init_camera(self.camera_name)
        rospy.loginfo("Starting up camera")
        self._start()

        # client
        # rospy.loginfo("Waiting for service FGE")
        # rospy.wait_for_service("/FGE")
        # self._call_fge = rospy.ServiceProxy("/FGE", CallFGE)

        # server
        try:
            rospy.loginfo("debug1")
            self._action_server = actionlib.SimpleActionServer("search_grasp_phoxi", o2as_msgs.msg.SearchGraspPhoxiAction, execute_cb=self.search_grasp_action_callback, auto_start = False)
            self._action_server.start()
            # self.search_grasp = rospy.Service("search_grasp", SearchGrasp)
            rospy.loginfo("debug1")
        except rospy.ServiceException as e:
            rospy.logerr(e.message)
            return False
        
        rospy.loginfo("Node [" + self.camera_ns + "] is started successfully!")
    
    def init_camera(self, camera_name):
        """initialize phoxi
        
        Params:
            - camera_name serial number to identify phoxi
        """
        self.camera_ns = "/" + camera_name + "/"
        
        rospy.loginfo("Trying to connect to phoxi: " + self.camera_ns)
        rospy.wait_for_service(self.camera_ns + "start_acquisition")
        self._start_acquisition = ros_service_proxy(self.camera_ns + "start_acquisition", Trigger)
        self._trigger_frame = ros_service_proxy(self.camera_ns + "trigger_frame", Trigger)
        self._get_frame_srv_client = ros_service_proxy(self.camera_ns + "get_frame", GetFrame)
        rospy.Subscriber(self.camera_ns + "camera_info", sensor_msgs.msg.CameraInfo, self._camera_info_callback)
        rospy.loginfo("Camera initialized")

    def _graspability_matlab(self, parts_id, bin_id, filename, gripper_type):
        req_to_fge = CallFGERequest()
        resp_from_fge = CallFGEResponse()
        req_to_fge.parts_id = parts_id
        req_to_fge.bin_id = bin_id
        req_to_fge.filename = os.path.join("/home/osx/ur-o2as/catkin_ws/src/o2as_graspability_estimation/graspability_estimation/data/images", filename)
        req_to_fge.gripper_type= gripper_type
        resp = SearchGraspResponse()
        try:
            resp_from_fge = self._call_fge(req_to_fge)
        except rospy.ServiceException:
            rospy.logerr(GRASPABLE_POINT_NOT_FOUND_MSG)
            resp.success = False
            return resp
        resp.result_num = resp_from_fge.result_num
        for point_in_image in resp_from_fge.pos3D:
            resp.pos3D.append(self._convert_pixel_to_meter(point_in_image))
        resp.rot3D = copy.deepcopy(resp_from_fge.rot3D) # May be necessary to convert these values.
        resp.rotipz = copy.deepcopy(resp_from_fge.rotipz)
        resp.success = True
        return resp

    def _graspability_python(self, parts_id, bin_id, filename, gripper_type, viz=True):
        import graspability

        # modify gripper type to avoid vacuum the bottom of bins
        if parts_id == 12:
            gripper_type = "two_finger"

        image_dir = os.path.join(rp.get_path("graspability_estimation"), "data/images")
        mask_filename = os.path.join(image_dir, "imr3.png")
        mask_img = io.imread(os.path.join(image_dir, mask_filename), as_gray=True)
        img = io.imread(os.path.join(image_dir, filename), as_gray=True)
        result_tmp = graspability.graspability(img, parts_id, bin_id, gripper_type, mask_img)
        resp = SearchGraspResponse()
        if result_tmp is not None:
            (posx, posy, posz, rotx, roty, rotz, rotipz, gscore, res_img) = result_tmp
        else:
            resp.success = False
            return resp
        io.imsave(os.path.join(image_dir, filename.rstrip(".tiff") + ".png"), res_img)
        res_img_msg = self.bridge.cv2_to_imgmsg(res_img, "bgr8")
        self.pub_res_img.publish(res_img_msg)

        ids_depth_nonzero = np.nonzero(posz)[0]

        # delete neighbor points
        positions = [self._convert_pixel_to_meter(posx[i], posy[i], posz[i]) for i in ids_depth_nonzero]
        ids_del_elem = []
        for j in range(len(positions)):
            if j in ids_del_elem:
                continue
            for i in range(j+1, len(positions)):
                if i in ids_del_elem:
                    continue
                if np.linalg.norm(np.asarray(positions[i])-np.asarray(positions[j])) <= 0.02:
                    ids_del_elem.append(i)
                    
        resp.result_num = len(ids_depth_nonzero) - len(ids_del_elem)
        for i in ids_depth_nonzero:
            if i not in ids_del_elem:
                x, y, z = self._convert_pixel_to_meter(posx[i], posy[i], posz[i])
                resp.pos3D.append(geometry_msgs.msg.Point(x, y, z))
                resp.rot3D.append(geometry_msgs.msg.Point(rotx[i], roty[i], rotz[i]))
                resp.rotipz.append(rotipz[i])
        resp.success = False if resp.result_num == 0 else True

        return resp

    def search_grasp(self, req):
        if req.is_updated:
            ts = time.time()
            st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H:%M:%S')
            self.filename = self.camera_name + "_" + st + ".tiff"
            self._save_image(self.image_dir + self.filename)

        parts_id = req.parts_id
        bin_id = self.bin_id[req.bin_name]
        filename = self.filename
        gripper_type = None

        if req.gripper_type == "suction":
            gripper_type = "suction"
        elif req.gripper_type == "precision_gripper_from_inside":
            gripper_type = "inner"
        elif req.gripper_type == "precision_gripper_from_outside" or req.gripper_type == "robotiq_gripper":
            gripper_type = "two_finger"
        else:
            raise rospy.ROSException("%s is undefined gripper type")

        # MATLAB Graspability
        # return self._graspability_matlab(parts_id, bin_id, filename, gripper_type)

        # Python Graspability
        return self._graspability_python(parts_id, bin_id, filename, gripper_type)

    def search_grasp_action_callback(self, goal):
        rospy.loginfo("Received SearchGraspPhoxi goal")
        if goal.update_image:
            ts = time.time()
            st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H:%M:%S')
            self.filename = self.camera_name + "_" + st + ".tiff"
            self._save_image(os.path.join(self.image_dir, self.filename))

        parts_id = goal.part_id
        bin_id = self.bin_id[goal.bin_name]
        filename = self.filename
        gripper_type = None
        if goal.gripper_type == "suction":
            gripper_type = "suction"
        elif goal.gripper_type == "precision_gripper_from_inside":
            gripper_type = "inner"
        elif goal.gripper_type == "precision_gripper_from_outside" or goal.gripper_type == "robotiq_gripper":
            gripper_type = "two_finger"
        else:
            raise rospy.ROSException("%s is undefined gripper type")

        srv_res = self._graspability_python(parts_id, bin_id, filename, gripper_type, viz=True)
        # srv_res = self._graspability_matlab(parts_id, bin_id, filename, gripper_type)
        print("received from python:")
        print(srv_res)
        goal_res = o2as_msgs.msg.SearchGraspPhoxiResult()
        goal_res.success = srv_res.success
        goal_res.result_num = srv_res.result_num
        goal_res.pos3D = copy.deepcopy(srv_res.pos3D)
        goal_res.rot3D = copy.deepcopy(srv_res.rot3D) # May be necessary to convert these values.
        goal_res.rotipz = copy.deepcopy(srv_res.rotipz)

        self._action_server.set_succeeded(goal_res)
        return True

    ########
    # Internal method
    ########

    def _save_image(self, filename):
        depth_image = self.bridge.imgmsg_to_cv2(self._get_frame(), '32FC1')
        io.imsave(filename, depth_image)

    def _get_frame(self):
        # res_trigger = self._trigger_frame()
        while not self._trigger_frame().success:
            rospy.logerr("Trigger frame request to the phoxi camera was failed.")
            rospy.sleep(1)

        res_get_frame = self._get_frame_srv_client(0, True)
        return res_get_frame.depth_map

    def _camera_info_callback(self, data):
        self.cx = data.K[2]
        self.cy = data.K[5]
        self.fx = data.K[0]
        self.fy = data.K[4]

    def _start(self):
        return self._start_acquisition()
    
    def _convert_pixel_to_meter(self, u, v, d):
        """ 
        convert graspability result pixel value to distance.
        """

        rospy.logdebug("pixel point and depth: %f, %f, %f" % (u, v, d))
        z = d
        x = (u - self.cx) * d / self.fx
        y = (v - self.cy) * d / self.fy
        rospy.logdebug("spatial position: %f, %f, %f" % (x, y, z))
        return x, y, z

if __name__ == '__main__':
    node = GraspabilityEstimation()
    try:
        rospy.spin()
    except rospy.ROSInitException as e:
        rospy.logerr(e.message)
    except rospy.ROSInitException as e:
        rospy.logerr(e.message)

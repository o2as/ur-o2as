import rospy
from o2as_realsense_camera.srv import *

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
    
# the class send request to the camera server
class RealSenseCameraClient(object):
    def __init__(self, server_node_name=""):
        ns = "/"+server_node_name+"/"
        self._get_frame = ros_service_proxy(ns+"get_frame", GetFrame)
        self._dump_frame = ros_service_proxy(ns+"dump_frame", DumpFrame)

    def get_frame(self, publish=False):
        return self._get_frame(publish)

    def dump_frame(self, color_image_filename, depth_image_filename, point_cloud_filename):
        return self._dump_frame(color_image_filename, depth_image_filename, point_cloud_filename)

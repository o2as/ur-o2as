import rospy
from o2as_phoxi_camera.srv import *
from std_srvs.srv import *

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

class PhoXiCamera(object):
    def __init__(self, name):
        ns = "/"+name+"/"
        self._get_device_list = ros_service_proxy(ns+'get_device_list', GetStringList)
        self._start_acquisition = ros_service_proxy(ns+'start_acquisition', Trigger)
        self._trigger_frame = ros_service_proxy(ns+'trigger_frame', Trigger)
        self._dump_frame = ros_service_proxy(ns+'dump_frame', DumpFrame)
        self._get_frame = ros_service_proxy(ns+'get_frame', GetFrame)
    
    def start(self):
        return self._start_acquisition()

    def dump_frame(self, cloud_filename, image_filename):
        res_trigger = self._trigger_frame()
        if (res_trigger.success == False):
            rospy.logerr("Trigger frame request to the phoxi camera was failed.")

        return self._dump_frame(cloud_filename, image_filename)

    def get_frame(self):
        res_trigger = self._trigger_frame()
        if (res_trigger.success == False):
            rospy.logerr("Trigger frame request to the phoxi camera was failed.")

        res_get_frame = self._get_frame()
        return res_get_frame.cloud, res_get_frame.texture

    def get_depth_image_frame(self):
        return "phoxi"        

import rospy
from o2as_cad_matching.srv import *
from omron_cad_matching.search_client import SearchClient

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

class CadMatchingClient(object):
    def __init__(self):
        self.lib = SearchClient(server="search_server")
        self.select_camera = ros_service_proxy("/cad_matching/select_camera", SelectCamera)
        self.select_object = ros_service_proxy("/cad_matching/select_object", SelectObject)
        self.search_objects = ros_service_proxy("/cad_matching/search_objects", SearchObjects)
        self.search_objects2 = ros_service_proxy("/cad_matching/search_objects2", SearchObjects2)

        # optional setter of parameters 
        self.set_search_area = ros_service_proxy("/cad_matching/set_search_area", SetSearchArea)
        self.set_mask_image = ros_service_proxy("/cad_matching/set_mask_image", SetMaskImage)
        self.set_search_range = ros_service_proxy("/cad_matching/set_search_range", SetSearchRange)

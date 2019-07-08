#!/usr/bin/env python

import rospy
from o2as_graspability_estimation.srv import *
import actionlib

LOG_LEVEL = log_level = rospy.DEBUG

def ros_service_proxy(service_name, service_type, timeout=None):
    proxy = None
    try:
        rospy.loginfo("wait for service: %s" % service_name)
        rospy.wait_for_service(service_name, service_type, timeout=timeout)
        rospy.loginfo("connect to service %s" % service_name)
        proxy = rospy.ServiceProxy(service_name, service_type)
    except rospy.ServiceException as e:
        rospy.logerr("connect to service %s if failed." % service_name)

    return proxy
    
class GraspTest(object):
    def __init__(self):
        self.search_grasp_client = actionlib.SimpleActionClient("search_grasp_phoxi", o2as_msgs.msg.SearchGraspPhoxiAction)

    def search_grasp_test(self):
        goal = o2as_msgs.msg.SearchGraspPhoxiGoal()
        goal.part_id = item.part_id
        goal.bin_name = item.bin_name
        goal.gripper_type = item.ee_to_use
        goal.update_image = True

        try:
            self.search_grasp_client.send_goal(goal)
            self.search_grasp_client.wait_for_result(rospy.Duration(5.0))
            resp_search_grasp = self.search_grasp_client.get_result()
        except:
            rospy.logerr("Could not get grasp from Phoxi")
        
        return resp_search_grasp

if __name__ == '__main__':
    g = GraspTest()

    rospy.loginfo("================ Search Grasp PhoXi Test ================")
    rospy.loginfo("Enter 1: to search grasp candidates.")
    rospy.loginfo()

    i = raw_input()
    
    if i == '1':
        res = g.search_grasp_test()
        if res.success:
            rospy.loginfo("Grasp candidates represent Phoxi sensor coordinate system.")

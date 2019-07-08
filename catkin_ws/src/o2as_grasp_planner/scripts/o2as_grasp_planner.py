#!/usr/bin/env python

from o2as_grasp_planner.srv import *
import moveit_msgs 
import rospy
import sys

class GraspPlanner():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.find_grasp_service = rospy.Service('find_grasp', FindGrasp, self.find_grasp_callback)

        rospy.loginfo('Grasp planner started up')
        rospy.spin()

    def find_grasp_callback(self, req):
        print("findGrasp service was called for item ID: " + req.item_id)
        # Insert the methods to generate a grasp for the item
        return FindGraspResponse()

    def add_two_ints_server(self, ):
        
        print "Ready to add two ints."
        rospy.spin()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('o2as_grasp_planner')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = GraspPlanner()
    except rospy.ROSInterruptException: pass
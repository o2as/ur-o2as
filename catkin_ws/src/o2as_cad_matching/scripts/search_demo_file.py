#!/usr/bin/env python

import os
import rospy
import rospkg
from o2as_cad_matching.util import *
from o2as_cad_matching.client import *
import dynamic_reconfigure.client

class SearchDemo(object):
	def __init__(self):
		# get params
		object_id = rospy.get_param("~object_id")
		camera_name = rospy.get_param("~camera_name")
		cloud_filename = rospy.get_param("~cloud_filename")
		image_filename = rospy.get_param("~image_filename")

		# prepare for cad matching
		cad_matching = CadMatchingClient()
		cad_matching.select_camera(camera_name)
		cad_matching.select_object(object_id)
		#cad_matching.set_search_area(0, 0, 640, 360)
		#cad_matching.set_search_range(300, 500)

		try:
			while not rospy.core.is_shutdown():
				# search objects
				cad_matching.search_objects(cloud_filename, image_filename, "")
				rospy.rostime.wallsleep(0.001)
		except rospy.ServiceException as e:
			rospy.logerr("Service call failed: %s", str(e)) 

if __name__ == "__main__":
	rospy.init_node('search_demo', anonymous=True, log_level=rospy.DEBUG)
	node = SearchDemo()
	rospy.spin()

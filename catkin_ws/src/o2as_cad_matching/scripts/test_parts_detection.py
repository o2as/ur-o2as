#!/usr/bin/env python

import os
import rospy
import rospkg
from o2as_cad_matching.util import *
from o2as_cad_matching.client import *

from os import listdir
from os.path import isfile, join

class SearchDemo(object):
	def __init__(self):
		# test cases
		# directories = {
		# 	4: "motor",
		# 	7: "housing",
		# }

		# get params
		object_id = 7
		camera_name = "phoxi"

		# prepare for cad matching
		cad_matching = CadMatchingClient()
		cad_matching.select_camera(camera_name)
		cad_matching.select_object(str(object_id))
		cad_matching.select_object(str(object_id))
		# cad_matching.set_search_area(0, 0, 2064, 1544)
		#cad_matching.set_search_area(409, 585, 1508, 1296)
		# cad_matching.lib.set_search_area(0, 756, 2064, 1269) # 02/middle_bin
		cad_matching.lib.set_search_area(0, 432, 2064, 1269) # 02/all_bin
		cad_matching.lib.set_search_range(300, 1100)
		cad_matching.lib.set_thresh_search(50)

		# list files in image directory
		rospack = rospkg.RosPack()
		directory = "02" #directories[object_id]
		image_dir = os.path.join(rospack.get_path("o2as_cad_matching"), "data/image/"+directory)
		files = [f for f in listdir(image_dir) if isfile(join(image_dir, f))]
		names = dict()
		i = 0
		for file in files:
			name, ext = os.path.splitext(file)
			if (ext == ".dat" or ext == ".png"):
				names[name] = i
				i = i+1

		while not rospy.core.is_shutdown():
			for name in names:
				try:
					# search objects
					rospy.loginfo("search motor in the file :" + name)
					cloud_filename = os.path.join(image_dir, name+".dat")
					image_filename = os.path.join(image_dir, name+".png")
					cad_matching.search_objects(cloud_filename, image_filename, "")
					rospy.rostime.wallsleep(0.001)
				except rospy.ServiceException as e:
					rospy.logerr("Service call failed: %s", str(e)) 
			break

if __name__ == "__main__":
	rospy.init_node('test_parts_detection', anonymous=True, log_level=rospy.DEBUG)
	node = SearchDemo()
	rospy.spin()

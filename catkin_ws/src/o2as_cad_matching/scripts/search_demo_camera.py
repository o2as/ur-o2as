#!/usr/bin/env python

import os
import rospy
import rospkg
import dynamic_reconfigure.client

import time
import datetime

from sensor_msgs.msg import Image
from omron_cad_matching.srv import *
from o2as_cad_matching.util import *
from o2as_cad_matching.client import *
from o2as_cad_matching.camera_adapter import CameraAdapter

class SearchDemo(CadMatchingClient):
	def __init__(self):
		super(SearchDemo, self).__init__()
		
		# get camera
		object_id = rospy.get_param("~object_id")
		camera_name = rospy.get_param("~camera_name")
		camera_type = rospy.get_param("~camera_type")
		camera_serial_number = rospy.get_param("~camera_serial_number")
		image_dir = rospy.get_param("~image_dir")

		# save image service
		self.save_frame = ros_service_proxy("/data_collection_server/save_frame", SaveFrame)

		# init camera
		camera = CameraAdapter(camera_name, camera_type)
		if camera is None:
			rospy.logerr("failed to connect to the camera server.")
			return

		# prepare for search
		self.select_camera(camera_name)
		self.select_object(object_id)
		#self.set_search_area(0, 0, 640, 360)
		#self.set_search_range(300, 500)

		try:
			while not rospy.core.is_shutdown():
				# capture image and search object
				cloud, image = camera.get_frame()

				# # search object
				# mask_image = Image() # mask image is empty for the moment
				# self.search_objects2(cloud, image, mask_image)

				# save frame to the file (named with time stamp)
				# ts = time.time()
				# st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H:%M:%S')
				# filename = camera_type + "_" + camera_serial_number + "_" + st
				filename = camera_type+"_"+camera_serial_number
				cloud_filename = os.path.join(image_dir, filename+".dat")
				image_filename = os.path.join(image_dir, filename+".png")
				self.save_frame(cloud, image, cloud_filename, image_filename)
				self.search_objects(cloud_filename, image_filename, "")

				rospy.rostime.wallsleep(0.001)
		except rospy.ServiceException as e:
			rospy.logerr("Service call failed: %s", str(e)) 

if __name__ == "__main__":
	rospy.init_node('search_demo', anonymous=True, log_level=rospy.DEBUG)
	node = SearchDemo()
	rospy.spin()

#!/usr/bin/env python

import os
import rospy
import time
import datetime

import cv2
from cv_bridge import CvBridge

from omron_cad_matching.srv import *
from o2as_cad_matching.util import *
from o2as_cad_matching.camera_adapter import CameraAdapter

class DataCollection():
	def __init__(self):
		# get params
		image_dir = rospy.get_param("~image_dir")
		camera_name = rospy.get_param("~camera_name")
		camera_type = rospy.get_param("~camera_type")
		camera_serial_number = rospy.get_param("~camera_serial_number")

		# client
		self.save_frame = ros_service_proxy("/data_collection_server/save_frame", SaveFrame)

		# camera
		camera = CameraAdapter(camera_name, camera_type)

		# display image for debug
		bridge = CvBridge()
		cv2.namedWindow('image', cv2.WINDOW_NORMAL)

		i = 0
		while not rospy.core.is_shutdown():
			# get frame from the camera
			cloud, image = camera.get_frame()

			# save image and point cloud to the files named with time stamp
			ts = time.time()
			st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H:%M:%S')
			filename = camera_type + "_" + camera_serial_number + "_" + st
			# filename = camera_type+"_"+camera_serial_number #+"_"+str(i)
			cloud_filename = os.path.join(image_dir, filename+".dat")
			image_filename = os.path.join(image_dir, filename+".png")
			self.save_frame(cloud, image, cloud_filename, image_filename)

			# display image for debug
			cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
			cv2.imshow('image', cv_image)
			cv2.waitKey(1)
			i = i+1

		cv2.destroyAllWindows()

if __name__ == "__main__":
	rospy.init_node('data_collection', anonymous=True, log_level=rospy.DEBUG)
	node = DataCollection()
	rospy.spin()

#!/usr/bin/env python

import rospy
import cv2
import os
import numpy as np
from cv_bridge import CvBridge
from o2as_vision.phoxi_camera_adapter import *
import csv

class CameraCalibration(object):
    def __init__(self, camera):
        image_dir = rospy.get_param("~image_dir")
        filename = os.path.join(image_dir, "parameter.csv")
        camera.start()
        with open(filename, 'wb') as f:
            writer = csv.writer(f, delimiter=',')
            for i in range(10):
                self.calibrate(camera, writer)

    def calibrate(self, camera, writer):
        bridge = CvBridge()
        cv2.namedWindow('img', cv2.WINDOW_NORMAL)
        
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        pattern_size = (10,7)
        n_points = np.prod(pattern_size)
        pattern_points = np.zeros((n_points, 3), np.float32)
        pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)

        # Arrays to store object points and image points from all the images.
        obj_points = [] # 3d point in real world space
        img_points = [] # 2d points in image plane.
        h, w = 0, 0

        try:
            # capturing frames
            cnt = 0
            n_images = 20
            while not rospy.is_shutdown():
                cloud, texture = camera.get_frame()
                if texture.width == 0 or texture.height == 0:
                    rospy.logerr("get frame failed")
                    continue

                img = bridge.imgmsg_to_cv2(texture, desired_encoding="passthrough")
                img_size = (img.shape[1], img.shape[0])
                h, w = img.shape[:2]

                gray = img
                cv2.imshow('img', gray)

                # Find the chess board corners
                found, corners = cv2.findChessboardCorners(gray, pattern_size, cv2.CALIB_CB_FILTER_QUADS)
                rospy.logdebug("findChessboardCorners: " + str(found))
                rospy.logdebug("corners: " + str(corners))

                # If found, add object points, image points (after refining them)
                if found == True:
                    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
                    corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), term)
                    #img_points.append(corners2)
                    img_points.append(corners.reshape(1, -1, 2))
                    obj_points.append(pattern_points.reshape(1, -1, 3))

                    rospy.logdebug("cornerSubPix: ")
                    rospy.logdebug("corners2: " + str(corners2))

                    # Draw and display the corners
                    img = cv2.drawChessboardCorners(img, pattern_size, corners, found)
                    img = cv2.drawChessboardCorners(img, pattern_size, corners2, found)
                    cv2.imshow('img', img)
                    cv2.waitKey(10) # 500
                    cnt = cnt+1

                cv2.waitKey(1)
                if cnt > n_images:
                    break

            camera_matrix = np.array((3,3), float)
            dist_coeffs = np.array((5), float)
            rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None)
            rospy.logdebug("camera_matrix: " + str(camera_matrix))
            rospy.logdebug("dist_coeffs: " + str(dist_coeffs))

            writer.writerow([str(camera_matrix[0,0]), str(camera_matrix[0,2]), str(camera_matrix[1,1]), str(camera_matrix[1,2])])

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: " + str(e))

        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node('o2as_realsense_camera_calibration', anonymous=True, log_level=rospy.DEBUG)
    camera = PhoXiCamera("o2as_phoxi_camera")
    calibration = CameraCalibration(camera)
    rospy.spin()

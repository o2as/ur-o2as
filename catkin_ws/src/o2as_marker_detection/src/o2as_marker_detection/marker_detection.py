import cv2
import cv2.aruco as aruco
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import rospy

#dir(aruco)

class MarkerDetection(object):
    def __init__(self):
        # self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        # self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_7X7_50)

    def generate_marker(self, filename, id, size=100):
        generator = aruco.drawMarker(self.dictionary, id, size)
        cv2.imwrite(filename, generator)
        img = cv2.imread(filename)
        cv2.imshow('marker', img)
        cv2.waitKey(1)

    def detect_marker(self, cloud, image, target_marker_id=0):
        # read image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # detect marker
        corners, ids, rejectedImgPoints = aruco.detectMarkers(image, self.dictionary)
        rospy.loginfo("corners: " + str(corners))
        rospy.loginfo("ids: " + str(ids))

        x = np.zeros(4)
        y = np.zeros(4)
        z = np.zeros(4)
        points = []
        height, width, channels = image.shape
        if len(corners) > 0:
            for number, corner in zip(ids, corners):
                if number == target_marker_id:
                    for i in range(4):
                        # get pixel coordinates of the detected marker's corners 
                        cx = corner[0][i][0]
                        cy = corner[0][i][1]
                        rospy.loginfo("corner_{}_pixel = ({}, {})".format(i, cx, cy))

                        # get 3d point from the camera using point cloud. unit is meter.
                        index = int(cy)*width + int(cx)
                        p = cloud[index]
                        points.append(p)
                        x[i] = p[0]
                        y[i] = p[1]
                        z[i] = p[2]
                        rospy.loginfo("corner_{}_pos3d = {}".format(i, p))
                    break

        marker_center = (np.mean(x),np.mean(y),np.mean(z))
        rospy.loginfo("marker_center = {}".format(marker_center))

        # draw result
        result = image.copy()
        aruco.drawDetectedMarkers(result, corners, ids, (0,255,0))
        cv2.imshow('drawDetectedMarkers', result)
        cv2.waitKey(1)

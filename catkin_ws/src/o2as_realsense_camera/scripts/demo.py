#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from o2as_realsense_camera.client import *

class CameraDemoNode(RealSenseCameraClient):
    def __init__(self):
        super(CameraDemoNode, self).__init__(rospy.get_param("~camera_name"))

        bridge = CvBridge()

        cloud_filename = rospy.get_param("~cloud_filename")
        image_filename = rospy.get_param("~image_filename")

        cv2.namedWindow('color_image', cv2.WINDOW_NORMAL)
        cv2.namedWindow('depth_image', cv2.WINDOW_NORMAL)
        
        try:
            while not rospy.is_shutdown():
                resp = self.get_frame(publish=False)
                self.dump_frame(cloud_filename, "", image_filename)

                # display captured frame
                if resp.color_image.width > 0 and resp.color_image.height > 0:
                    cv_color_image = bridge.imgmsg_to_cv2(resp.color_image, desired_encoding="passthrough")
                    rospy.logdebug("color image size: " + str(cv_color_image.shape))
                    cv2.imshow('color_image', cv_color_image)
                else:
                    rospy.logwarn("color image is empty")

                if resp.depth_image.width > 0 and resp.depth_image.height > 0:
                    cv_depth_image = bridge.imgmsg_to_cv2(resp.depth_image, desired_encoding="passthrough")
                    rospy.logdebug("depth image size: " + str(cv_depth_image.shape))
                    cv2.imshow('depth_image', cv_depth_image)
                else:
                    rospy.logwarn("depth image is empty")

                cv2.waitKey(1)

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)

        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node('dump_frame_demo', anonymous=True, log_level=rospy.DEBUG)
    node = CameraDemoNode()

#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from o2as_phoxi_camera.srv import *
from std_srvs.srv import *

if __name__ == "__main__":
    rospy.init_node('o2as_phoxi_camera_example', anonymous=True)

    rospy.wait_for_service('o2as_phoxi_camera/get_device_list')

    bridge = CvBridge()
    cv2.namedWindow('texture', cv2.WINDOW_NORMAL)
    cv2.namedWindow('depth', cv2.WINDOW_NORMAL)

    try:
        # get device list
        res_get_device_list = rospy.ServiceProxy('o2as_phoxi_camera/get_device_list', GetStringList)()
        rospy.loginfo("devices" + str(res_get_device_list.out))

        # start data acquisition
        res_star_acq = rospy.ServiceProxy('o2as_phoxi_camera/start_acquisition', Trigger)()

        # capturing frames
        while not rospy.is_shutdown():
            res_trigger = rospy.ServiceProxy('o2as_phoxi_camera/trigger_frame', Trigger)()
            if (res_trigger.success == False):
                rospy.logerr("Trigger frame request to the phoxi camera was failed.")

            res_get_frame = rospy.ServiceProxy('o2as_phoxi_camera/get_frame', GetFrame)(0, False)
            if (res_get_frame.success == False):
                rospy.logerr("Get frame request to the phoxi camera was failed.")
            rospy.loginfo("get_frame: success = " + str(res_get_frame.success))

            # display captured frame
            if res_get_frame.depth_map.width > 0 and res_get_frame.depth_map.height > 0:
                cv_depth_image = bridge.imgmsg_to_cv2(res_get_frame.depth_map, desired_encoding="passthrough")
                rospy.logdebug("depth image size: " + str(cv_depth_image.shape))
                cv2.imshow('depth', cv_depth_image)
            else:
                rospy.logwarn("depth image is empty")

            # display captured frame
            if res_get_frame.texture.width > 0 and res_get_frame.texture.height > 0:
                cv_texture = bridge.imgmsg_to_cv2(res_get_frame.texture, desired_encoding="passthrough")
                rospy.logdebug("texture size: " + str(cv_texture.shape))
                cv2.imshow('texture', cv_texture)
            else:
                rospy.logwarn("texture is empty")

            cv2.waitKey(1)

    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: " + str(e))

    cv2.destroyAllWindows()

#!/usr/bin/env python

import numpy as np
from numpy.linalg import norm
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from cv_bridge import CvBridge

import actionlib
import o2as_msgs.msg
from sensor_msgs.msg import Image

from PIL import Image as Image_
from PIL import ImageDraw


def _draw_rect(img, x, y, w, h, color, thick=False):
    """ Draw rectangle on numpy array.

    Parameter color is given like (255, 0, 0). """

    img_ = Image_.fromarray(img)
    draw = ImageDraw.Draw(img_)
    draw.rectangle((x, y, x + w, y + h), outline=color)

    if thick:
        draw.rectangle((x - 1, y - 1, x + w + 1, y + h + 1), outline=color)

    return np.asarray(img_)


import copy

class InnerPickDetection(object):

    def __init__(self):
         #Variable
        self._current_image = Image()

        self.bridge = CvBridge()
        # Config parameters
        # From before assembly with surprise parts trial
        # origin is top left, x is right, y is down
        self._x = 311
        self._y = 285
        self._w = 23
        self._h = 22

        # Publish input image (and output value)
        # The image is published when bg ratio is computed
        self.bridge = CvBridge()
        self.pub_input_image = rospy.Publisher("/o2as_debug_monitor/bg_ratio_input_image", Image, queue_size=1)
        #self.pub_output_value = rospy.Publisher("/o2as_debug_monitor/bg_ratio_output_value", Float64, queue_size=1)

        self._image_topic = "/a_bot_camera/color/image_raw"

        # Subscriber
        rospy.Subscriber(self._image_topic, Image, self.image_callback)

        #define the action
        self._action_name = "inner_pick_detection_action"
        self._action_server = actionlib.SimpleActionServer(self._action_name, o2as_msgs.msg.innerPickDetectionAction, execute_cb=self.action_callback, auto_start = False)
        self._action_server.start()
        rospy.loginfo('Action server '+ str(self._action_name)+" started.")
        self.action_result = o2as_msgs.msg.innerPickDetectionResult()

        self.img_empty = cv2.imread('/root/catkin_ws/src/o2as_bg_ratio/images/empty_close_gripper.png')
        self.img_empty = np.asarray(self.img_empty)[:, :, ::-1]
        self._empty_bg_ratio = self.compute_red_ratio(self.img_empty, self._x, self._y, self._w, self._h)
        #rospy.get_param("/empty_bg_ratio", '0.7')

        self.img_counter = 0
    
    # Action Callback
    def action_callback(self, goal):
        rospy.loginfo('Executing'+ str(self._action_name)+"."+"request sent:")
        rospy.loginfo(goal)
        rospy.loginfo('Executing'+ str(self._action_name)+"."+"request sent:")

        #TODO consider the part to compare the red ratio
        #Save image of with the ROI
        try:
            if(goal.saveROIImage):
                self.saveROIImage()
        except rospy.ROSInterruptException:
            print "ROI image of calibration not saved"
            print "goal.saveROIImage is not defined in the call of inner_pick_detection-action"

        res = self.compute_red_ratio(self._current_image, self._x, self._y, self._w, self._h) #< self._empty_bg_ratio

        self.img_counter += 1
        if res:
            res_string = "success"
        else:
            res_string = "failed"
        cv2.imwrite("/root/catkin_ws/src/o2as_bg_ratio/images/" + str(self.img_counter) + "_" + res_string + ".png'", cv2.cvtColor(self._current_image, cv2.COLOR_BGR2RGB))

        self.action_result.success = res
        self.action_result.picked = res
        self._action_server.set_succeeded(self.action_result)

    #Save Image with ROI
    def saveROIImage(self):
        imageROI = copy.deepcopy(self._current_image)

        cv2.rectangle(imageROI, (self._x,self._y), (self._x+self._w,self._y + self._h), (255,0,0), 3, ) 
        cv2.imwrite("/root/catkin_ws/src/o2as_bg_ratio/images/ROIempty_close_gripper.png'", cv2.cvtColor(imageROI, cv2.COLOR_BGR2RGB))


    # Image Callback
    def image_callback(self, msg_in):
        self._current_image = self.bridge.imgmsg_to_cv2(msg_in, desired_encoding="passthrough")
        self._current_image = np.asarray(self._current_image)[:, :, ::-1]

    #Compute ratio
    def compute_red_ratio(self, img, x, y, w, h, br_threshold=0.2, red_threshold=0.9, vis=False):
        """Compute the ratio of red area in the image.

        The returned value should be used to check if the precision gripper pick a
        part or not. Assumption here is that the picked part will decrease the
        ratio of red area; low value implies the gripper picks a part.

        Return True if bg_ratio < bg_threshold, False otherwise.

        :param img: RGB image, 8bit (0-255), numpy.ndarray, shape=(w, h, 3)
        :param int x: x-axis of Upper left of ROI.
        :param int y: y-axis of Upper left of ROI.
        :param int w: Width of ROI.
        :param int h: Height of ROI.
        """
        # Input
        img0 = img

        # bright pixels
        img = img / 255.0
        img = img[y:(y + h), x:(x + w), :]

        if vis:
            import matplotlib.pyplot as plt
            plt.imshow(img, interpolation="none")

        # Ignore dark area
        brightness = norm(img, axis=2)
        ixs = np.where(brightness > br_threshold)
        pixels = img[ixs[0], ixs[1], :]
        brightness = brightness[ixs[0], ixs[1]]

        # extract background
        red_ratio = pixels[:, 0] / brightness
        # plt.hist(red_ratio)
        ixs_bg = np.where(red_ratio > red_threshold)[0]

        # compute background ratio
        bg_ratio = float(ixs_bg.shape[0]) / pixels.shape[0]
        bg_ratio_message = Float64()
        bg_ratio_message.data = bg_ratio
        #self.pub_output_value.publish(bg_ratio_message)

        threshold = 0.9
        if bg_ratio < threshold:
            img0 = _draw_rect(img0, x, y, w, h, (0, 255, 0), True)
            item_picked = True
        else:
            img0 = _draw_rect(img0, x, y, w, h, (255, 0, 0))
            item_picked = False

        # publish input image with detection rectangle
        img_message = self.bridge.cv2_to_imgmsg(img0, "rgb8")
        self.pub_input_image.publish(img_message)
        rospy.loginfo("bg_ratio in check_pick: " + str(bg_ratio))

        return item_picked

if __name__ == "__main__":

  # Initialization
  rospy.init_node("o2as_inner_pick_detection")
  node = InnerPickDetection()

  rospy.spin()

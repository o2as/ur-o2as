#!/usr/bin/env python

import numpy as np
from numpy.linalg import norm
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def compute_red_ratio(img, x, y, w, h, br_threshold=0.2, red_threshold=0.7,
                      vis=False):
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
  return float(ixs_bg.shape[0]) / pixels.shape[0]

if __name__ == "__main__":
  # Config parameters
  # TODO: read values from config file
  node_name = "o2as_bg_ratio"
  image_topic = "/a_bot_camera/color/image_raw"
  bg_ratio_topic = "/a_bot_camera/bg_ratio"
  x = 270
  y = 275
  w = 64
  h = 32

  # Initialization
  rospy.init_node(node_name)
  pub = rospy.Publisher(bg_ratio_topic, Float64, queue_size=10)
  bridge = CvBridge()

  # Callback
  def callback(msg_in):
    img = bridge.imgmsg_to_cv2(msg_in, desired_encoding="passthrough")
    img = np.asarray(img)[:, :, ::-1]
    bg_ratio = compute_red_ratio(img, x, y, w, h)
    msg_out = Float64()
    msg_out.data = bg_ratio
    pub.publish(msg_out)

    # Slow down this node and reduce data transfer for image
    rospy.sleep(0.01)

  # Subscriber
  rospy.Subscriber(image_topic, Image, callback)

  rospy.spin()

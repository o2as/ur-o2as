#!/usr/bin/env python

import numpy as np
from numpy.linalg import norm
from skimage import io
import matplotlib.pyplot as plt
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

# Compute ratio
def compute_red_ratio(img, x, y, w, h, br_threshold=0.2,
                      red_threshold=0.7, vis=False):
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
  # bg_ratio_message = Float64()
  # bg_ratio_message.data = bg_ratio
  # self.pub_output_value.publish(bg_ratio_message)

  threshold = 0.9
  if bg_ratio > threshold:
    img0 = _draw_rect(img0, x, y, w, h, (0, 255, 0), True)
  else:
    img0 = _draw_rect(img0, x, y, w, h, (255, 0, 0))

  # # publish input image with detection rectangle
  # img_message = self.bridge.cv2_to_imgmsg(img0, "rgb8")
  # self.pub_input_image.publish(img_message)

  return bg_ratio

if __name__ == "__main__":
  success_files = [
    # "/root/catkin_ws/src/o2as_bg_ratio/images/20_success.png'",
    # "/root/catkin_ws/src/o2as_bg_ratio/images/27_success.png'",
    # "/root/catkin_ws/src/o2as_bg_ratio/images/37_success.png'",
    # "/root/catkin_ws/src/o2as_bg_ratio/images/3_success.png'"
  ]

  failure_files = [
    "/root/catkin_ws/src/o2as_bg_ratio/images/8_failed.png'",
    # "/root/catkin_ws/src/o2as_bg_ratio/images/11_failed.png'",
    # "/root/catkin_ws/src/o2as_bg_ratio/images/12_failed.png'",
    # "/root/catkin_ws/src/o2as_bg_ratio/images/13_failed.png'",
  ]

  # ROI
  x = 311
  y = 280
  w = 30
  h = 27

  bg_ratio_s = []
  bg_ratio_f = []

  # for i, f in enumerate(success_files):
  #   img = io.imread(f)
  #   if i == 0:
  #     plt.figure()
  #     tmp = compute_red_ratio(img, x, y, w, h, vis=True)
  #   else:
  #     tmp = compute_red_ratio(img, x, y, w, h)
  #   print("{}".format(tmp))
  #   bg_ratio_s.append(tmp)

  print("==========")

  for f in failure_files:
    img = io.imread(f)
    tmp = compute_red_ratio(img, x, y, w, h)
    print("{}".format(tmp))
    bg_ratio_f.append(tmp)

  data = np.vstack([np.array(bg_ratio_s), np.array(bg_ratio_f)]).T
  plt.figure()
  plt.hist(data)
  plt.show()
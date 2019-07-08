#!/usr/bin/env python

import numpy as np
from numpy.linalg import norm
import rospy
from std_msgs.msg import Float64
import geometry_msgs.msg
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sensor_msgs.point_cloud2 as pc2

from PIL import Image as PIL_Image
from PIL import ImageDraw as PIL_ImageDraw
from geometry_msgs.msg import Polygon, Point32 

import copy

import actionlib
import o2as_msgs.msg
from geometry_msgs.msg import PoseArray, Pose 

from dynamic_reconfigure.server import Server as ServerDynamicReconfigure
from o2as_blob_detection.cfg import o2asBlobDetectionConfig


class BlobDetectionTestBag(object):
    def __init__(self):
#        self.rospack = rospkg.RosPack()
#        print(self.rospack.get_path(rospy.get_name()))

        #Variable
        self.bridge = CvBridge()
        self.current_image = Image()
        self.current_image_blob = Image()
        self.current_cloud = PointCloud2()
        self.current_detected_poses = PoseArray() 
        self.current_param_part_id = "none"

        # Config parameters
        # TODO: read values from config file
        self.image_topic = "/a_bot_camera/color/image_raw"
        self.cloud_topic = rospy.get_name()+ "/camera/cloud"
        self.blob_pos_img_topic = rospy.get_name()+"/blob_pos_img"
        self.blob_pos_cloud_topic = rospy.get_name()+"/blob_pos_cloud"
        self.img_w_blob_topic = rospy.get_name()+"/img_w_blob"

        # Publisher
        self.pub_img_pos = rospy.Publisher(self.blob_pos_img_topic, geometry_msgs.msg.Point, queue_size=10)
        self.pub_cloud_pos = rospy.Publisher(self.blob_pos_cloud_topic, geometry_msgs.msg.PoseArray, queue_size=10)
        self.pub_img_w_blob = rospy.Publisher(self.img_w_blob_topic, Image, latch=True,queue_size=10)

        # Subscriber
        rospy.Subscriber(self.image_topic, Image, self.image_callback)

        # Dynamic reconfigure
        srv = ServerDynamicReconfigure(o2asBlobDetectionConfig, callbackDynamicReconfigure)
   
        self.minThreshold = 100
        self.maxThreshold = 400
    
        # Filter by color
        self.filterByColor = True
        self.blobColor = 0
    
        # Filter by size of the blob.
        self.filterByArea = True
        self.minArea = 20
        self.maxArea = 150
    
        # Filter by Circularity
        self.filterByCircularity = True
        self.minCircularity = 0.7
    
        # Filter by Convexity
        self.filterByConvexity = False
        self.minConvexity = 0.87
    
        # Filter by Inertia
        self.filterByInertia = False
        self.minInertiaRatio = 0.01


    # Callback

    def callbackDynamicReconfigure(config, level):
        rospy.loginfo("""Reconfigure Request: {min_threshold_param}""".format(**config))
        return config


    def image_callback(self, msg_in):
      # Convert the image to be used wit OpenCV library
      self.current_image = copy.deepcopy(msg_in)

      #mask_u = 200
      #mask_v = 100



      self.detect_blob(self.current_image)
    
    def mask_image(self, in_img_cv, in_polygon):


        polygon = [ (in_polygon.points[0].x,in_polygon.points[0].y),
                    (in_polygon.points[1].x,in_polygon.points[1].y),
                    (in_polygon.points[2].x,in_polygon.points[2].y),
                    (in_polygon.points[3].x,in_polygon.points[3].y)] 
 
        #print(in_img_cv.shape)
        # Create the mask from the polygon
        mask_img = PIL_Image.new('L', (in_img_cv.shape[1],in_img_cv.shape[0]), 0)
        # Draw the mask with the polygon coordinate
        PIL_ImageDraw.Draw(mask_img).polygon(polygon, outline = 1, fill = 255)
        mask_image_np = np.array(mask_img)

        #DEBUG Ssave the mask in a file
        #namefile = "mask.png"
        #mask_img.save(namefile,'PNG')

        # Apply the mask to the image
        #img_cv = cv2.imread(img_to_mask)
        #mask_cv = cv2.imread(mask_img,0)
        #cv2.imwrite("/root/catkin_ws/src/o2as_blob_detection/img_res/blob_detection_"+self.current_param_part_id+"_mask.png", mask_image_np)
        img_cv = in_img_cv
        out_img = cv2.bitwise_and(img_cv,img_cv, mask = mask_image_np)
        #cv2.imwrite('/root/catkin_ws/masked_image.png',out_img)
        #cv2.imwrite("/root/catkin_ws/src/o2as_blob_detection/img_res/blob_detection_"+self.current_param_part_id+"_masked_image.png", cv2.cvtColor(out_img, cv2.COLOR_BGR2RGB))
        return out_img

    def detect_blob(self, in_img):
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
        img_cv = self.bridge.imgmsg_to_cv2(in_img , desired_encoding="passthrough")
        img = np.asarray(img_cv)[:, :, ::-1]

        im_rgb = img
        im_gray = cv2.cvtColor(im_rgb,cv2.COLOR_BGR2GRAY)
    
        params = cv2.SimpleBlobDetector_Params()
    

        # Segmentation Thresholds
        params.minThreshold = 100
        params.maxThreshold = 400
    
        # Filter by color
        params.filterByColor = True
        params.blobColor = 0
    
        # Filter by size of the blob.
        params.filterByArea = True
        params.minArea = 20
        params.maxArea = 150
    
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.7
    
        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.87
    
        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01


    
       # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3:
            detector = cv2.SimpleBlobDetector(params)
        else:
            detector = cv2.SimpleBlobDetector_create(params)
    
        # Detect blobs.
        keypoints = detector.detect(im_gray)
    
        # Draw the key points 
        im_with_keypoints = cv2.drawKeypoints(im_rgb, keypoints, np.array([]), (255, 0, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
        # Show blobs

        #cv2.imwrite("/root/catkin_ws/src/o2as_blob_detection/img_res/blob_detection_"+param_part_id+"_results.png", cv2.cvtColor(im_with_keypoints, cv2.COLOR_BGR2RGB))
        #print("before publish")

        try:
          self.pub_img_w_blob.publish(self.bridge.cv2_to_imgmsg(im_with_keypoints, "rgb8"))
        except CvBridgeError as e:
          print(e)

        self.current_image_blob = self.bridge.cv2_to_imgmsg(im_with_keypoints, "rgb8")

        blob_array = []
     

if __name__ == "__main__":


  # Initialization
  rospy.init_node("o2as_blob_detection_test_bag")
  node = BlobDetectionTestBag()

  rospy.spin()

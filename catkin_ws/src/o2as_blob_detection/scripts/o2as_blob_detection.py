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

import rospkg

class BlobDetection(object):
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
        self.image_topic = rospy.get_name()+ "/camera/color/image_raw"
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
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_callback)

        # Define the action
        self._action_name = "blob_detection_action"
        self._action_server = actionlib.SimpleActionServer(self._action_name, o2as_msgs.msg.blobDetectionAction, execute_cb=self.action_callback, auto_start = False)
        self._action_server.start()
        rospy.loginfo('Action server '+ str(self._action_name)+" started.")
        self.action_result = o2as_msgs.msg.blobDetectionResult()

    # Action Callback
    def action_callback(self, goal):
      rospy.loginfo('Executing'+" "+str(self._action_name)+"."+"request sent:")
      rospy.loginfo(goal)

      self.current_param_part_id = goal.param_part_id
      res = self.detect_poses(goal.maskCorner, goal.param_part_id)

      self.action_result.success = res
      if(res):
          self.action_result.posesDetected = self.current_detected_poses
          self.action_result.blobImage =  self.current_image_blob
          self._action_server.set_succeeded(self.action_result)
      else:
          self.action_result.posesDetected = PoseArray()
          self._action_server.set_succeeded(self.action_result)


    # Callback
    def image_callback(self, msg_in):
      # Convert the image to be used wit OpenCV library
      self.current_image = copy.deepcopy(msg_in)

      #mask_u = 200
      #mask_v = 100

      #test_polygon = Polygon()
      #test_polygon.points = [Point32(mask_u,mask_v,0),
      #                       Point32(640-mask_u,mask_v,0),
      #                       Point32(640-mask_u,360,0),
      #                       Point32(mask_u,360,0)]


      #self.detect_poses(test_polygon)
    
    def detect_poses(self, in_polygon, param_part_id = "default"):


      #img_cv=cv2.imread('/root/catkin_ws/mask_extract/set2_bin1_4_img2.png')
      img_cv = self.bridge.imgmsg_to_cv2(self.current_image , desired_encoding="passthrough")
      #cv2.imwrite('/root/catkin_ws/original_image.png',img_cv)
      cv2.imwrite("/root/catkin_ws/src/o2as_blob_detection/img_res/blob_detection_"+param_part_id+"_original.png", img_cv) 
      img = np.asarray(img_cv)[:, :, ::-1]
      # Apply the mask
      mask_u = 200
      mask_v = 100
      #test_polygon = [ (mask_u,mask_v),
      #                 (msg_in.width-mask_u,mask_v),
      #                 (msg_in.width-mask_u,msg_in.height-mask_v),
      #                 (mask_u,msg_in.height-mask_v)]

      #test_polygon = geometry_msgs.msg.Point()
      #test_polygon [0] = geometry_msgs_msg.Point(mask_u,mask_v,0) 
      #test_polygon [1] = geometry_msgs_msg.Point(msg_in.width-mask_u,mask_v)
      #test_polygon [2] = geometry_msgs_msg.Point(msg_in.width-mask_u,msg_in.height-mask_v)
      #test_polygon [3] = geometry_msgs_msg.Point(mask_u,msg_in.height-mask_v)

      masked_img= self.mask_image(img, in_polygon)

      # Detect the blob in the image
      res_b, blob_array = self.detect_blob(masked_img, param_part_id)
   
      if(res_b):
          msg_out = geometry_msgs.msg.Point()
          msg_out = blob_array[0]
          self.pub_img_pos.publish(msg_out)

          # Convert the blob position image in the 3D camera system
          #TODO synchronize the time stamp between image and cloud
          rospy.loginfo( rospy.get_name() + " number of blob detected " + str(len(blob_array)))

          self.current_detected_poses.header = self.current_cloud.header
          tmp_pose_array = []
          for i in range(0,len(blob_array)):
            tmp_pose = geometry_msgs.msg.Pose() 
            res_point3D = self.compute_3D_pos1(self.current_cloud, int(blob_array[i].x), int(blob_array[i].y))
            if(res_point3D.point.x == -0.0 and res_point3D.point.y ==0.0 and res_point3D.point.z ==0.0):
                #TODO find a approximation with nearby point
                rospy.loginfo( "Bad point cloud reading  (probably non dense) ")
            else:
                tmp_pose.position = res_point3D.point
                tmp_pose_array.append(tmp_pose)
          if(len(tmp_pose_array)>0):
               rospy.loginfo( rospy.get_name() + " number of poses detected " + str(len(tmp_pose_array)))
               self.current_detected_poses.poses = tmp_pose_array
               # Publish the results   
               self.pub_cloud_pos.publish(self.current_detected_poses)
               return True
          else:
               rospy.loginfo( rospy.get_name() + "no poses detected" )
               return False                    


          # Slow down this node and reduce data transfer for image
          #rospy.sleep(0.01)
      else:
          return False  

          # Convert the blob position image in the 3D camera system
          # Publish the results   

          # Slow down this node and reduce data transfer for image
          #rospy.sleep(0.01)

    def cloud_callback(self, msg_in):

      self.current_cloud = copy.deepcopy(msg_in)

    def compute_3D_pos1(self, in_cloud, u,v):
      # TODO the projection require int but the precision of the blob detected are in float so there is a loss of accuracy 
    
      data_out = pc2.read_points(in_cloud, field_names = None, skip_nans=False, uvs=[[u, v]]) 
      int_data = next(data_out)

      res_out = geometry_msgs.msg.PointStamped()
      res_out.header = in_cloud.header
      res_out.point.x = int_data[0]  
      res_out.point.y = int_data[1]
      res_out.point.z = int_data[2] 

      return res_out

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
        cv2.imwrite("/root/catkin_ws/src/o2as_blob_detection/img_res/blob_detection_"+self.current_param_part_id+"_mask.png", mask_image_np)
        img_cv = in_img_cv
        out_img = cv2.bitwise_and(img_cv,img_cv, mask = mask_image_np)
        #cv2.imwrite('/root/catkin_ws/masked_image.png',out_img)
        cv2.imwrite("/root/catkin_ws/src/o2as_blob_detection/img_res/blob_detection_"+self.current_param_part_id+"_masked_image.png", cv2.cvtColor(out_img, cv2.COLOR_BGR2RGB))
        return out_img

    def detect_blob(self, img, param_part_id = "default"):
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
        im_rgb = img
        im_gray = cv2.cvtColor(im_rgb,cv2.COLOR_BGR2GRAY)
    
        params = cv2.SimpleBlobDetector_Params()
    

        rospy.loginfo( rospy.get_name() + " parameter chosen: " + param_part_id)
        if(param_part_id == "none"):
            rospy.loginfo( rospy.get_name() +"in: " + param_part_id)
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

        elif(param_part_id == "part_9"):
            rospy.loginfo( rospy.get_name() +"in: " + param_part_id)
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

        elif(param_part_id == "part_15"):
            rospy.loginfo( rospy.get_name()+ "in: " + param_part_id)

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
        else:
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

        cv2.imwrite("/root/catkin_ws/src/o2as_blob_detection/img_res/blob_detection_"+param_part_id+"_results.png", cv2.cvtColor(im_with_keypoints, cv2.COLOR_BGR2RGB))
        #print("before publish")

        try:
          print("publish")
          self.pub_img_w_blob.publish(self.bridge.cv2_to_imgmsg(im_with_keypoints, "rgb8"))
        except CvBridgeError as e:
          print(e)

        self.current_image_blob = self.bridge.cv2_to_imgmsg(im_with_keypoints, "rgb8")

        blob_array = []
     
        if(len(keypoints)):
            for i in range(len(keypoints)):
                blob= geometry_msgs.msg.Point()
                blob.x = keypoints[i].pt[0]
                blob.y = keypoints[i].pt[1]
                blob_array.append(blob)
            return True, blob_array

        else:  

            return False, blob_array
 

if __name__ == "__main__":


  # Initialization
  rospy.init_node("o2as_blob_detection")
  node = BlobDetection()

  rospy.spin()

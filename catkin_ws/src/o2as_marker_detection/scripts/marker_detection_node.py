#!/usr/bin/env python

import os
import rospy
import rospkg
rospack = rospkg.RosPack()

from dynamic_reconfigure.server import Server
from o2as_marker_detection.cfg import MarkerPoseEstimationConfig
from o2as_marker_detection.marker_detection import *
from o2as_marker_detection.camera_client import *

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

Params= (
    dummy_param,
) = range(0, 1)

class MarkerDetectionNode(MarkerDetection):
    def __init__(self):
        super(MarkerDetectionNode, self).__init__()

        # configure
        self.dynamic_reconfigure = Server(MarkerPoseEstimationConfig, self.dynamic_reconfigure_callback)

        # # generate marker
        # image_dir = rospy.get_param("~image_dir")
        # for i in range(50):
        #     marker_filename = os.path.join(image_dir, "marker_" + str(i) + ".png")
        #     self.generate_marker(marker_filename, i, 600)

        # connect to the camera
        self.bridge = CvBridge()
        camera_name = rospy.get_param("~camera_name")
        camera_type = rospy.get_param("~camera_type")
        self.camera = CameraClient(camera_name, camera_type)
        while not rospy.is_shutdown():
            # get frame
            cloud, texture = self.camera.get_frame(publish=True)
            cv_image = self.bridge.imgmsg_to_cv2(texture, "bgr8")

            # convert point cloud2 message to points
            i = 0
            points=np.zeros((cv_image.shape[1]*cv_image.shape[0], 3))
            for p in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=False):
                points[i,0]=p[0]
                points[i,1]=p[1]
                points[i,2]=p[2]
                i = i+1
            
            self.detect_marker(points, cv_image, target_marker_id=0)
        rospy.spin()

        cv2.destroyAllWindows()

    def set_param(self, config, level):
        if level & (1 << Params[dummy_param]):
            rospy.set_param("~dummy_param", config.dummy_param)

    def dynamic_reconfigure_callback(self, config, level):
        self.set_param(config, level)
        return config

if __name__ == "__main__":
    rospy.init_node('marker_detection', anonymous=True, log_level=rospy.DEBUG)
    node = MarkerDetectionNode()
    rospy.spin()

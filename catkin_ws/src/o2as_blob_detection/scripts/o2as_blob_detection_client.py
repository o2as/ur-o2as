#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used
import o2as_msgs.msg

from geometry_msgs.msg import Polygon, Point32


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS.
        rospy.init_node('o2as_blob_detection_client')
        client = actionlib.SimpleActionClient('blob_detection_action', o2as_msgs.msg.blobDetectionAction)
        client.wait_for_server()
        goal = o2as_msgs.msg.blobDetectionGoal()

        # Apply the mask
        mask_u = 100
        mask_v = 50
        #test_polygon = [ (mask_u,mask_v),
        #                 (msg_in.width-mask_u,mask_v),
        #                 (msg_in.width-mask_u,msg_in.height-mask_v),
        #                 (mask_u,msg_in.height-mask_v)]
        mask_u = 200
        mask_v = 100
  
        test_polygon = Polygon()
        test_polygon.points = [Point32(mask_u,mask_v,0),
                               Point32(640-mask_u,mask_v,0),
                               Point32(640-mask_u,360,0),
                               Point32(mask_u,360,0)]  
          
        goal.maskCorner = test_polygon
        goal.param_part_id = "yep"
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_result()
        rospy.loginfo(result.posesDetected)
        rospy.loginfo(result.success)
    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion", file=sys.stderr)

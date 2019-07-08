#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used
import o2as_msgs.msg


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS.
        rospy.init_node('o2as_inner_pick_detection_client')
        client = actionlib.SimpleActionClient('inner_pick_detection_action', o2as_msgs.msg.innerPickDetectionAction)
        client.wait_for_server()
        goal = o2as_msgs.msg.innerPickDetectionGoal()
              
        #goal.saveROIImage = True
        goal.part_id = 12
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_result()
        rospy.loginfo(result)
    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion", file=sys.stderr)

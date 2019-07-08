#!/usr/bin/env python

"""
$ sh RUN-DOCKER-CONTAINER.sh

# Run below commands in the container
$ roslaunch o2as_gazebo o2as_gazebo.launch

# In another terminal in the container
$ rosrun o2as_pc2depth o2as_pc2depth.py
"""

import rospy
# from o2as_pc2depth.srv import pc2depthResponse
# from o2as_pc2depth.srv import pc2depthRequest
from o2as_pc2depth.srv import pc2depth

def get_depth_image():
    rospy.init_node("test_pc2depth")
    proxy = rospy.ServiceProxy("/pc2depth_server/pc2depth_service", pc2depth)
    res = proxy("/b_bot_camera/depth/points")

    return res

if __name__ == "__main__":
    res = get_depth_image()
    print(res)

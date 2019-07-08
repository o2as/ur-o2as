#!/usr/bin/env python

import sys
import os
import rospy
from std_srvs.srv import Trigger
from o2as_phoxi_camera.srv import GetFrame, SetString

camera_name = "a_phoxi_m_camera"
cs          = "/{}/".format(camera_name)

trigger_frame_srv = rospy.ServiceProxy(cs + "trigger_frame", Trigger)
get_frame_srv     = rospy.ServiceProxy(cs + "get_frame",     GetFrame)
save_depth_srv    = rospy.ServiceProxy(cs + "save_depth",    SetString)

def trigger_frame():
    for i in range(10):
        if (trigger_frame_srv().success):
            return True
    return False
          
def get_frame():
    for i in range(10):
        if (get_frame_srv(0, True).success):
            return True
    return False

def save_detph(file_name):
    return save_depth_srv(file_name).success

if __name__ == '__main__':
    file_name   = "/root/share/initial_bins.tiff"
    
    
    while True:
        try:
            key = raw_input("Hit \"return\"(save depth image) or \"q\"(quit) > ")
            if key == 'q':
                break

            if not trigger_frame():
                raise RuntimeError("trigger_frame failed.")
            if not get_frame():
                raise RuntimeError("get_frame failed.")
            if not save_depth_srv(file_name):
                raise RuntimeError("save_depth failed.")
            print "Depth image successfully saved."
      
        except Exception as ex:
            print ex.message

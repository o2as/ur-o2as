#!/usr/bin/env python
import rospy
from phoxi_camera.srv import *
from std_srvs.srv import *

if __name__ == "__main__":
    rospy.init_node('phoxi_camera_actual_example', anonymous=True)
    while not rospy.is_shutdown():
        rospy.wait_for_service('phoxi_camera/get_device_list')
        try:
            get_device_list = rospy.ServiceProxy('phoxi_camera/get_device_list', GetDeviceList)
            resp1 = get_device_list()
            print "devices", resp1.out
            name = "1711015"
            res_connect = rospy.ServiceProxy('phoxi_camera/connect_camera', ConnectCamera)(name)
            print "connect to", name, res_connect
            if res_connect.success:
                res_star_acq = rospy.ServiceProxy('phoxi_camera/start_acquisition', Empty)()
                res_trig = rospy.ServiceProxy('phoxi_camera/trigger_image', TriggerImage)()
                res_get_fram = rospy.ServiceProxy('phoxi_camera/get_frame', GetFrame)(-1)
                print "get_frame", res_get_fram
                rospy.sleep(10)
            else:
                print "can't connect"
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        res_dis = rospy.ServiceProxy('phoxi_camera/disconnect_camera', Empty)()
        print "disconnected"

#!/usr/bin/env python

import os
import rospy
import yaml
import std_srvs.srv 
from omron_cad_matching.train_client import *

class TrainSingleNode(TrainClient):
    def __init__(self):
        super(TrainSingleNode, self).__init__()

        # file settings
        cad_filename        = rospy.get_param("~cad_filename")
        conf_filename       = rospy.get_param("~conf_filename")
        setting_filename    = rospy.get_param("~camera_setting_filename")
        model_filename      = rospy.get_param("~model_filename")

        # object search param
        min_dist            = rospy.get_param("~train_setting/min_dist")
        max_dist            = rospy.get_param("~train_setting/max_dist")
        thread_num          = rospy.get_param("~train_setting/thread_num")

        # camera setting
        camera_setting = read_camera_setting_yaml(setting_filename)

        # read object config
        obj_conf = read_object_config_yaml(conf_filename)
        search_setting = get_search_setting(obj_conf, min_dist, max_dist, thread_num)

        # train
        res = self.train_model(cad_filename, camera_setting, search_setting)

        # save trained model that include template of all parts into a file
        text_filename = os.path.splitext(model_filename)[0] + "_train.txt"
        self.save_model(model_filename, text_filename)

if __name__ == "__main__":
    rospy.init_node('train_single', anonymous=True, log_level=rospy.INFO)
    node = TrainSingleNode()

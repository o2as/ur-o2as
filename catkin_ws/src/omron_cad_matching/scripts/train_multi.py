#!/usr/bin/env python

import os
import rospy
import yaml
from omron_cad_matching.util import *
from omron_cad_matching.train_client import *

class TrainMultiNode(TrainClient):
    def __init__(self):
        super(TrainMultiNode, self).__init__()

        # file settings
        cad_dir             = rospy.get_param("~cad_dir")
        conf_dir            = rospy.get_param("~conf_dir")
        parts_list          = rospy.get_param("~parts_list")
        setting_filename    = rospy.get_param("~camera_setting_filename")
        model_dir           = rospy.get_param("~model_dir")
        model_name          = rospy.get_param("~model_name")

        # object search param
        min_dist            = rospy.get_param("~train_setting/min_dist")
        max_dist            = rospy.get_param("~train_setting/max_dist")
        thread_num          = rospy.get_param("~train_setting/thread_num")

        # create client of omron cad matching training service server
        self.init_model()

        # camera setting
        camera_setting = read_camera_setting_yaml(setting_filename)

        # for each parts
        id_map = dict()
        object_id = 0
        for parts in parts_list:
            # read object config
            conf_filename = os.path.join(conf_dir, parts + ".yaml")
            rospy.loginfo("read object config. file = %s", conf_filename)
            obj_conf = read_object_config_yaml(conf_filename)
            search_setting = get_search_setting(obj_conf, min_dist, max_dist, thread_num)

            # train
            cad_filename = os.path.join(cad_dir, parts + ".stl")
            rospy.loginfo("train. cad file = %s", cad_filename)
            res = self.train_model(cad_filename, camera_setting, search_setting)
            if res.model_id >= 0:
                id_map[obj_conf.object_id] = res.model_id

        # save trained model that include template of all parts into a file
        data_filename = os.path.join(model_dir, model_name + ".dat")
        text_filename = os.path.join(model_dir, model_name + "_train.txt")
        rospy.loginfo("save model. file = %s", data_filename)
        self.save_model(data_filename, text_filename)

        # save map from object_id to model_id to yaml file 
        id_filename = os.path.join(model_dir, model_name + ".yaml")
        data = dict(id_map = id_map)
        with open(id_filename, 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False)
        rospy.loginfo("id_map from object_id to model_id = " + str(id_map))

if __name__ == "__main__":
    rospy.init_node('train_multi', anonymous=True, log_level=rospy.INFO)
    node = TrainMultiNode()

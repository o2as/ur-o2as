#!/usr/bin/env python

import os
import rospy
import yaml
from o2as_cad_matching.util import *
from o2as_cad_matching.train_client import *

if __name__ == "__main__":
	rospy.init_node('train', anonymous=True, log_level=rospy.INFO)

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

	# create output directory
	output_dir = os.path.join(model_dir, model_name)
	if not os.path.exists(output_dir):
		os.makedirs(output_dir)
		
	# camera setting
	camera_setting = read_camera_setting_yaml(setting_filename)

	# create client of omron cad matching training service server
	trainer = TrainServiceClient()
		
	# for each parts
	map = dict()
	object_id = 0
	for parts in parts_list:
		# read object config
		conf_filename = os.path.join(conf_dir, parts + ".yaml")
		rospy.loginfo("read object config. file = %s", conf_filename)
		obj_conf = read_object_config_yaml(conf_filename)

		# train
		diff = 10
		max_dist_tmp = max_dist
		cad_filename = os.path.join(cad_dir, parts + ".stl")
		while max_dist_tmp > min_dist:
			rospy.loginfo("train. cad file = %s", cad_filename)
			search_setting = get_search_setting(obj_conf, min_dist, max_dist_tmp, thread_num)
			res = trainer.train(cad_filename, camera_setting, search_setting, new_model=False)
			if res.success:
				break
			else:
				max_dist_tmp = max_dist_tmp - diff
		map[obj_conf.object_id] = { "min_dist" : min_dist , "max_dist" : max_dist_tmp }

	# save map from object_id to model_id to yaml file 
	range_filename = os.path.join(output_dir, model_name + "_range.yaml")
	data = dict(training_range = map)
	with open(range_filename, 'w') as outfile:
		yaml.dump(data, outfile, default_flow_style=False)
	rospy.loginfo("map from object_id to model_id = " + str(map))

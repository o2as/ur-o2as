######################
### Service Helper ###
######################

import rospy

def ros_service_proxy(service_name, service_type):
    proxy = None
    try:
        rospy.logdebug("wait for service %s", service_name)
        rospy.wait_for_service(service_name)
        proxy = rospy.ServiceProxy(service_name, service_type)
    except rospy.ServiceException as e:
        rospy.logerr("service error: %s", str(e)) 
    return proxy

###################
### File Helper ###
###################

from omron_cad_matching.msg import *

def read_object_config(conf_filename, comment_char=';', name_char='='):
    obj_conf = omron_cad_matching.msg.ObjectConfig()
    f = open(conf_filename, "r") 
    for line in f:
        pos_comment = line.find(comment_char)
        if pos_comment > 0: line = line[0:pos_comment]
        pos_name = line.find(name_char)
        name = line[0:pos_name].strip()
        value = line[pos_name+1:len(line)].strip()
        if (name == "initial_pose_x" ):   obj_conf.ini_pose_x   = float(value)
        if (name == "initial_pose_y" ):   obj_conf.ini_pose_y   = float(value)
        if (name == "initial_pose_z" ):   obj_conf.ini_pose_z   = float(value)
        if (name == "both_side"      ):   obj_conf.both_sided   = int(value)
        if (name == "min_latitude"   ):   obj_conf.min_lat      = int(value)
        if (name == "max_latitude"   ):   obj_conf.max_lat      = int(value)
        if (name == "min_longitude"  ):   obj_conf.min_lon      = int(value)
        if (name == "max_longitude"  ):   obj_conf.max_lon      = int(value)
        if (name == "min_camera_roll"):   obj_conf.min_roll     = int(value)
        if (name == "max_camera_roll"):   obj_conf.max_roll     = int(value)
        if (name == "min_distance"   ):   obj_conf.min_dist     = int(value)
        if (name == "max_distance"   ):   obj_conf.max_dist     = int(value)
        if (name == "object_id"      ):   obj_conf.object_id    = int(value)
    return obj_conf

def read_object_config_yaml(conf_filename):
    return read_object_config(conf_filename, comment_char='#', name_char=':')

def read_camera_setting(setting_filename, comment_char=";", name_char="="):
    setting = omron_cad_matching.msg.CameraSetting()
    f = open(setting_filename, "r") 
    for line in f:
        pos_comment = line.find(comment_char)
        if pos_comment > 0: line = line[0:pos_comment]
        pos_name = line.find(name_char)
        name = line[0:pos_name].strip()
        value = line[pos_name+1:len(line)].strip()
        if (name == "width"            ):   setting.width               = int(value)
        if (name == "height"           ):   setting.height              = int(value)
        if (name == "focal_length_x"   ):   setting.focal_length_x      = float(value)
        if (name == "focal_length_y"   ):   setting.focal_length_y      = float(value)
        if (name == "principal_point_x"):   setting.principal_point_x   = float(value)
        if (name == "principal_point_y"):   setting.principal_point_y   = float(value)
        if (name == "dist_param_k1"    ):   setting.dist_param_k1       = float(value)
        if (name == "dist_param_k2"    ):   setting.dist_param_k2       = float(value)
        if (name == "dist_param_p1"    ):   setting.dist_param_p1       = float(value)
        if (name == "dist_param_p2"    ):   setting.dist_param_p2       = float(value)
        if (name == "dist_param_k3"    ):   setting.dist_param_k3       = float(value)
    return setting

def read_camera_setting_yaml(setting_filename):
    return read_camera_setting(setting_filename, "#", ":")

def get_search_setting(obj_conf, min_dist, max_dist, thread_num):
    setting = omron_cad_matching.msg.TrainSetting()
    setting.object_id = obj_conf.object_id
    setting.both_sided = obj_conf.both_sided
    setting.min_lat = obj_conf.min_lat
    setting.max_lat = obj_conf.max_lat
    setting.min_lon = obj_conf.min_lon
    setting.max_lon = obj_conf.max_lon
    setting.min_dist = min_dist
    setting.max_dist = max_dist
    setting.ini_pose_x = obj_conf.ini_pose_x
    setting.ini_pose_y = obj_conf.ini_pose_y
    setting.ini_pose_z = obj_conf.ini_pose_z
    setting.thread_num = thread_num
    return setting

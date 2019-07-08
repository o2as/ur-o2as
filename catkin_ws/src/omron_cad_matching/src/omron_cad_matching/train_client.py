import rospy
import std_srvs.srv 
from omron_cad_matching.util import *
from omron_cad_matching.srv import *

class TrainClient(object):
    def __init__(self):
        self.init_model = ros_service_proxy("init_model", std_srvs.srv.Empty)
        self.train_model = ros_service_proxy("train_model", TrainModel)
        self.save_model = ros_service_proxy("save_model", SaveModel)

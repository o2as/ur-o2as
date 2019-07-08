import yaml
import rospy
from omron_cad_matching.util import *
from omron_cad_matching.srv import *
import dynamic_reconfigure.client

# the client class to the search_server of the omron_cad_matching
class SearchClient(object):
    def __init__(self, server=""):
        self.dynamic_reconfigure_client = dynamic_reconfigure.client.Client("search_server")		

        # servers
        ns = "/" + server + "/"
        self.load_model_data = ros_service_proxy(ns+"load_model_data", LoadModelData)
        self.validate_model_data = ros_service_proxy(ns+"validate_model_data", ValidateModelData)
        self.invalidate_model_data = ros_service_proxy(ns+"invalidate_model_data", InvalidateModelData)
        self.search = ros_service_proxy(ns+"search", Search)
        self.search2 = ros_service_proxy(ns+"search2", Search2)

    # set all search params
    def read_search_params(self, filename):
        with open(filename, 'r') as f:
            params = yaml.load(f)
        self.dynamic_reconfigure_client.update_configuration(params)

    # set input image size
    def set_image_size(self, width, height):
        params = { 
            'width' : width,
            'height' : height
            }
        self.dynamic_reconfigure_client.update_configuration(params)

    # set usage of the color image
    def set_color_use(self, search_color_num, icp_color_num):
        params = { 
            'search_color_num' : search_color_num,
            'icp_color_num' : icp_color_num
            }
        self.dynamic_reconfigure_client.update_configuration(params)

    # set threshold of depth
    def set_thresh_depth(self, thresh_depth):
        params = { 
            'thresh_depth' : thresh_depth
            }
        self.dynamic_reconfigure_client.update_configuration(params)

    # set threshold of grad
    def set_thresh_grad(self, thresh_grad):
        params = { 
            'thresh_grad' : thresh_grad
            }
        self.dynamic_reconfigure_client.update_configuration(params)

    # set threshold of inlier pc
    def set_thresh_inlier_pc(self, thresh_inlier_pc):
        params = { 
            'thresh_inlier_pc' : thresh_inlier_pc
            }
        self.dynamic_reconfigure_client.update_configuration(params)

    # set thread number of search
    def set_thread_num(self, thread_num):
        params = { 
            'thread_num' : thread_num
            }
        self.dynamic_reconfigure_client.update_configuration(params)

    # set max result number
    def set_max_result_num(self, max_result_num, max_result_num_all):
        params = { 
            'max_result_num' : max_result_num,
            'max_result_num_all' : max_result_num_all
            }
        self.dynamic_reconfigure_client.update_configuration(params)

    # set search are
    def set_search_area(self, left, top, right, bottom):
        params = { 
            'search_area_left'   : left,
            'search_area_top'    : top, 
            'search_area_right'  : right, 
            'search_area_bottom' : bottom 
            }
        self.dynamic_reconfigure_client.update_configuration(params)

    # set search range
    def set_search_range(self, min_dist, max_dist):
        params = { 
            'min_dist' : min_dist,
            'max_dist' : max_dist
            }
        self.dynamic_reconfigure_client.update_configuration(params)
    
    # set threshold of search
    def set_thresh_search(self, thresh_search):
        params = { 
            'thresh_search' : thresh_search
            }
        self.dynamic_reconfigure_client.update_configuration(params)

    # set coeff of search
    def set_search_coef(self, search_coef):
        params = { 
            'search_coef' : search_coef
            }
        self.dynamic_reconfigure_client.update_configuration(params)

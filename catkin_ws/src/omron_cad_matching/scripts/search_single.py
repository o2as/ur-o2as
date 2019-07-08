#!/usr/bin/env python

import rospy
from omron_cad_matching.search_client import SearchClient

class SearchSingleNode(SearchClient):
    def __init__(self):
        super(SearchSingleNode, self).__init__(server="search_server")

        model_filename = rospy.get_param("~model_filename")
        pcloud_filename = rospy.get_param("~pcloud_filename")
        image_filename  = rospy.get_param("~image_filename")
        mask_filename  = rospy.get_param("~mask_filename")

        self.load_model_data(model_filename)
        self.search(pcloud_filename, image_filename, mask_filename)

if __name__ == "__main__":
    rospy.init_node('search_single', anonymous=True, log_level=rospy.INFO)
    node = SearchSingleNode()

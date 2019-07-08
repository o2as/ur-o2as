import rospy
from o2as_realsense_camera.client import RealSenseCameraClient

class RealSenseCamera(object):
    def __init__(self, name=""):
        self._name = name
        self._client = RealSenseCameraClient(name)
        self._namespace = "/"+name+"/"

    def start(self):
        return True

    def dump_frame(self, cloud_filename, image_filename):
        return self._client.dump_frame(cloud_filename, "", image_filename)

    def get_frame(self):
        return self._client.get_frame()

    def get_depth_image_frame(self):
        return self._name + "_depth_image_frame"

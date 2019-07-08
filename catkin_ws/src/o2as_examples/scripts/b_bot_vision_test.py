#!/usr/bin/env python

import rospy
from o2as_vision.client import VisionClient
from geometry_msgs.msg import PoseStamped


class PartsDatabase(object):
    def __init__(self):
        self._parts_database = dict()
        pass

    # object name : assebmly_part_01, assembly_part_02, ...
    # object_type : 1, 2, 3, 4, 5

    def get_object_type(self, object_name):
        rospy.logerr("TODO: load mesh")
        return None

    def get_mesh(self, object_name):
        rospy.logerr("TODO: load mesh")
        return None

class BBotVisionTestHelper(object):
    def __init__(self):
        pass

    def get_current_pose_in_the_planning_scene(self, object_name):
        # get current pose of specified object from the scene
        pose = PoseStamped()
        return pose

    def get_camera_pose(self, camera_name):


# this class provides missing functions to accomplish BBotVisionTest
class BBotVisionTestHelper(object):
    def __init__(self):
        pass

    def get_current_pose_in_the_planning_scene(self, object_name):
        # get current pose of specified object from the scene
        pose = PoseStamped()
        return pose

    def get_camera_pose(self, camera_name):

    def get_mesh(self, object_name):


class BBotVisionTest(object):
    def __init__(self):
        self.init_parts_dict()
        self._vision = VisionClient()
        self._helper = BBotVisionTestHelper()

    def pick(self, object_id, bin_id):
        # pick specified object 
        rospy.loginfo("Will look for item 11 (output-shaft-pulley) and try to pick it up. Press enter to proceed.")
        raw_input()

        # find parts
        expected_position   = self._helper.get_current_pose_in_the_planning_scene()
        position_tolerance  = 0.2               # within 20 cm
        object_id           = "11"              # output-shaft-pulley
        camera              = "b_bot_camera"    # should be changed to phoxi

        while not rospy.core.is_shutdown():
            item_pose = self._vision.find_object(expected_position, position_tolerance, object_id, camera)
            if item_pose is not None:
                rospy.loginfo("Found the object at pose:")
                rospy.loginfo(item_pose)
                break

        # pick parts
        self._proxy.pick(robotname="b_bot", object_pose=item_pose, grasp_height=0.2, 
            approach_height=0.05, speed_fast=0.2, speed_slow=0.02, gripper_command="")

    def run(self, product_count = 1):
        test = BBotVisionTest()
        test.pick()
        while not rospy.core.is_shutdown():
            rospy.rostime.wallsleep(0.5)

if __name__ == "__main__":
    rospy.init_node('b_bot_vision_test', anonymous=True, log_level=rospy.INFO)
    test = PipelineTest()
    test.run()

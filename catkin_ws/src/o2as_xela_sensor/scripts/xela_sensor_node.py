#!/usr/bin/env python
import os
import copy
import rospy
import rospkg
rospack = rospkg.RosPack()

import actionlib
from o2as_xela_sensor.msg import *
from o2as_xela_sensor.srv import *
from o2as_xela_sensor.xela_sensor import *

class XelaSensorClient(object):
  def __init__(self):
    self._base = [0] * taxel_num * 3
    self._data = [0] * taxel_num * 3
    self._sub_data = rospy.Subscriber("data", XelaSensorStamped, self.data_callback)
    self._sub_base = rospy.Subscriber("base", XelaSensorStamped, self.base_callback)

  def calibrate(self, sample_num, log_filename):
    calibrate_action_client = actionlib.SimpleActionClient('calibrate', o2as_xela_sensor.msg.CalibrateAction)
    calibrate_action_client.wait_for_server()
    goal = o2as_xela_sensor.msg.CalibrateGoal()
    goal.sample_num = sample_num
    goal.log_filename = log_filename
    calibrate_action_client.send_goal(goal)
    calibrate_action_client.wait_for_result()

  def data_callback(self, msg_in):
    self._data = copy.deepcopy(msg_in.data)
  
  def base_callback(self, msg_in):
    self._base = copy.deepcopy(msg_in.data)

  @property
  def data(self):
    return self._data
  
  @property
  def base(self):
    return self._base
class XelaSensorNode(XelaSensorClient):
  def __init__(self):
    super(XelaSensorNode, self).__init__()
    # Connect to the sensor and trigger to start acquisition
    board_id = rospy.get_param("board_id", 1)
    self._sensor = XelaSensor(board_id)
    self._sensor.start_data_acquisition()
    
    # Calibrate once
    data_dir = os.path.join(rospack.get_path("o2as_xela_sensor"), "data")
    filename = os.path.join(data_dir, "log{}.csv".format(board_id))
    self._sensor.calibrate(sample_num=100, filename=filename)

    # Publishers
    self._pub_base = rospy.Publisher("~base", XelaSensorStamped, queue_size=1)
    self._pub_data = rospy.Publisher("~data", XelaSensorStamped, queue_size=1)
    self._board_id = board_id

    # Calibrate action (for re-calibration)
    self._calibrate_action_name = "~calibrate"
    self._calibrate_action_server = actionlib.SimpleActionServer(self._calibrate_action_name, CalibrateAction, execute_cb=self.calibrate_action_callback, auto_start = False)
    self._calibrate_action_server.start()
    rospy.loginfo('Action server {} started.'.format(self._calibrate_action_name))
    self._calibrate_action_result = CalibrateResult()

  def calibrate_action_callback(self, goal):
    rospy.loginfo('Executing {}. request sent:'.format(self._calibrate_action_name))
    rospy.loginfo(goal)
    res = self._sensor.calibrate(sample_num=goal.sample_num, filename=goal.log_filename)
    self._calibrate_action_result.success = res
    self._calibrate_action_result.base = self._sensor.base
    self._calibrate_action_server.set_succeeded(self._calibrate_action_result)
    rospy.loginfo('Action server {} finished.'.format(self._calibrate_action_name))

  def free_run(self):
    # Constants
    scale_xy = 25  # Coefficient to convert from sensor data to Newton
    scale_z = 25   # Coefficient to convert from sensor data to Newton
    taxel_area = 0.0047*0.0047

    while not rospy.is_shutdown():
      ## Calculate forces
      force_diff = np.array(self._sensor.get_data(sensor_list)) - np.array(self.base)
      f_x = force_diff.reshape((taxel_num,3))[:,0]
      f_y = force_diff.reshape((taxel_num,3))[:,1]
      f_z = force_diff.reshape((taxel_num,3))[:,2]

      # is_under_pressure = (f_z > 18000)   # 16x1 boolean vector
      # cop = [is_under_pressure*(f_z.*x_coord), is_under_pressure*(f_z.*y_coord)]
      # for i in f_z:
      #   if i>18000:
      #     print (i)

      # Publish base (is this necessary?)
      base_msg = XelaSensorStamped()
      base_msg.board_id = self._board_id
      base_msg.header.stamp = rospy.Time.now()
      base_msg.data = self._sensor.base
      self._pub_base.publish(base_msg)

      # Publish data
      data_msg = XelaSensorStamped()
      data_msg.board_id = self._board_id
      data_msg.header.stamp = rospy.Time.now()
      data_msg.data = force_diff
      data_msg.f_n = np.sum(f_z)
      data_msg.f_t = [np.sum(f_x), np.sum(f_y)]
      data_msg.center_of_pressure = [0, 0]
      self._pub_data.publish(data_msg)

      rospy.sleep(.001)

if __name__ == '__main__':
  # defect_sensor = [[int(x.strip(' ')) for x in ss.lstrip(' [,').split(', ')] for ss in ignore_sensor.rstrip(']').split(']')]
  rospy.init_node('xela_robotics_sensor', anonymous=True, log_level=rospy.DEBUG)
  node = XelaSensorNode()
  # Read the input sensor number, which are defect and create a list
  if len(rospy.get_param("~defect_sensor", None)) > 0:
    sensor_list = map(int, rospy.get_param("~defect_sensor", None).split(","))
    print("Selected sensor numbers, which are set to zero: ", sensor_list)
  else:
    sensor_list = None
  node.free_run()

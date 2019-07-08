#!/usr/bin/env python


import lib_robotis_xm430 as xm430
import sys
import time

dynamixel1 = xm430.USB2Dynamixel_Device( '/dev/ttyUSB0' )
p1 = xm430.Robotis_Servo2( dynamixel1, 1, series = "XM" )#inner gripper
p2 = xm430.Robotis_Servo2( dynamixel1, 2, series = "XM" )#outer gripper
p3 = xm430.Robotis_Servo2( dynamixel1, 3, series = "XM" )#outer gripper
p4 = xm430.Robotis_Servo2( dynamixel1, 4, series = "XM" )#linear actuator

# import double_jaw_gripper as djg
# import time

# if __name__ == '__main__':
#     djg.outer_gripper_disable_torque()
#     # djg.inner_gripper_disable_torque()
#     # djg.linear_motor_disable_torque()

#!/usr/bin/env python

import lib_robotis_xm430 as xm430
import sys
import time
import rospy
from o2as_msgs.srv import *

class PrecisionGripper:
    def __init__(self, serial_port = '/dev/ttyUSB0'):
        self.dynamixel = xm430.USB2Dynamixel_Device( serial_port )
        self.p1 = xm430.Robotis_Servo2(self.dynamixel, 1, series = "XM" )#inner gripper
        self.p2 = xm430.Robotis_Servo2(self.dynamixel, 2, series = "XM" )#outer gripper
        self.p3 = xm430.Robotis_Servo2(self.dynamixel, 3, series = "XM" )#outer gripper
        self.p4 = xm430.Robotis_Servo2(self.dynamixel, 4, series = "XM" )#linear actuator
        
        name = rospy.get_name()
        self.pitch = rospy.get_param(name + "/pitch", 1.98)
        self.pgc_linear_zero = rospy.get_param(name + "/pgc_linear_zero", 3325)
        self.og_open_motor_pos_ = rospy.get_param(name + "/og_open_motor_pos_", 1024)
        return

    def my_callback(self, req):
        rospy.loginfo("Combined gripper callback has been called")
        res = PrecisionGripperCommandResponse()

        if req.stop:
            self.inner_gripper_disable_torque()
            self.outer_gripper_disable_torque()
        elif req.open_outer_gripper_fully and not(req.close_outer_gripper_fully):
            self.outer_gripper_open_fully(60)
        elif not(req.open_outer_gripper_fully) and (req.close_outer_gripper_fully):
            self.outer_gripper_close_new(60)
        elif req.open_inner_gripper_fully and not(req.close_inner_gripper_fully):
            self.inner_gripper_open_fully(6)
        elif not(req.open_inner_gripper_fully) and (req.close_inner_gripper_fully):
            self.inner_gripper_close_fully(4)
        elif req.stop:
            self.inner_gripper_disable_torque()
            self.outer_gripper_disable_torque()
            self.linear_motor_disable_torque()
        elif req.linear_motor_position > 0.0:
            self.linear_motor_move(self.linear_length_conversion(self.pgc_linear_zero, (req.linear_motor_position * 1000.0)))
        elif req.outer_gripper_opening_width > 0.0:
            rospy.logwarn("This is not implemented fully (opening width value is assumed to be the motor position)")
            self.outer_gripper_move_to(60,req.outer_gripper_opening_width)
        elif req.inner_gripper_opening_width > 0.0:
            rospy.logwarn("This is not implemented fully (opening width value is assumed to be the motor position)")
            self.inner_gripper_move_to(60,req.inner_gripper_opening_width)
        else:
            rospy.logerr('No command sent to the gripper, service request was empty.')
            res.success = False
            return res
            
        res.success = True
        return res

    ######################################################################################################
    #outer gripper related functions
    def outer_gripper_close_new(self,current):
        try:
            self.p2.set_operating_mode("currentposition")
            self.p3.set_operating_mode("currentposition")
            self.p2.set_current(current)#0.3Nm
            self.p3.set_current(current)
            i2=self.p2.read_current_position()
            i3=self.p3.read_current_position()
            i_avg=int((i2+i3)/2)
            while i_avg<2500:
                self.p2.set_goal_position(i_avg)
                self.p3.set_goal_position(i_avg)
                current2=int(self.p2.read_current())
                current3=int(self.p3.read_current())
                #rospy.logerr("current2="+str(current2))
                #rospy.logerr("current3="+str(current3))
                if current2>(current-10) and current2< 60000 and current3>(current-10) and current3<60000:
                    break
                i_avg=i_avg+4
        except:
            rospy.logerr("Failed to run commands.")

    def outer_gripper_close_fully(self,current):
        try:
            self.p2.set_operating_mode("current")
            self.p3.set_operating_mode("current")
            self.p2.set_current(current)#0.3Nm
            self.p3.set_current(current)
        except:
            rospy.logerr("Failed to run commands.")

    def outer_gripper_read_current_position(self):
        try:
            x=self.p2.read_current_position()
            y=self.p3.read_current_position()
            rospy.logerr("id2="+str(x))
            rospy.logerr("id3="+str(y))
        except:
            rospy.logerr("Failed to run commands.")

    def outer_gripper_open_fully(self,current):
        try:
            self.p2.set_operating_mode("currentposition")
            self.p3.set_operating_mode("currentposition")
            self.p2.set_current(current)
            self.p3.set_current(current)
            self.p2.set_goal_position(self.og_open_motor_pos_)
            self.p3.set_goal_position(self.og_open_motor_pos_)
        except:
            rospy.logerr("Failed to run commands.")
    def outer_gripper_disable_torque(self):
        try:
            self.p2.disable_torque()
            self.p3.disable_torque()
        except:
            rospy.logerr("Failed to run commands.")
    def outer_gripper_move_to(self,current,location):#still editing
        rospy.logwarn("Interpreting the input as an integer")
        location = int(location)
        try:
            self.p2.set_operating_mode("currentposition")
            self.p3.set_operating_mode("currentposition")
            self.p2.set_current(current)
            self.p3.set_current(current)
            self.p2.set_goal_position(location)
            self.p3.set_goal_position(location)
        except:
            rospy.logerr("Failed to run commands.")
    ###############################################################################################################
    #inner gripper related functions
    def inner_gripper_open_fully(self,current):
        try:
            self.p1.set_operating_mode("current")
            self.p1.set_positive_direction("cw")
            self.p1.set_current(current)
            # self.p1.set_goal_position(self.og_open_motor_pos_)
        except:
            rospy.logerr("Failed to run commands.")
    def inner_gripper_move_to(self,current_position):
        rospy.logwarn("Interpreting the input as an integer")
        current_position = int(current_position)
        try:
            self.p1.set_operating_mode("currentposition")
            self.p1.set_current(8)
            self.p1.set_goal_position(current_position)
        except:
            rospy.logerr("Failed to run commands.")
    def inner_gripper_close_fully(self,current):
        try:
            self.p1.set_operating_mode("current")
            self.p1.set_positive_direction("ccw")
            self.p1.set_current(current)
        except:
            rospy.logerr("Failed to run commands.")
    def inner_gripper_disable_torque(self):
        try:
            self.p1.disable_torque()
        except:
            rospy.logerr("Failed to run commands.")
    ####linear motor
    ##########################################################################################
    def linear_motor_move(self,distance):#distance in rotation
        try:
            self.p4.set_operating_mode("currentposition")
            self.p4.set_positive_direction("cw")
            self.p4.set_current(400)
            self.p4.set_goal_position(distance)
        except:
            rospy.logerr("Failed to run commands.")
    def linear_motor_disable_torque(self):
        try:
            self.p4.disable_torque()
        except:
            rospy.logerr("Failed to run commands.")
    #for calibration of linear motor
    def linear_motor_calibration_one_move_backward(self,):
        try:
            self.p4.set_operating_mode("current")
            self.p4.set_positive_direction("ccw")
            self.p4.set_current(100)
        except:
            rospy.logerr("Failed to run commands.")
    def linear_motor_calibration_two(self,):#positive
        try:
            self.p4.set_operating_mode("position")
            self.p4.set_positive_direction("cw")
        except:
            rospy.logerr("Failed to run commands.")
    def linear_motor_read_current_position(self):#negative#distance in mm
        try:
            pos=self.p4.read_current_position()
            return int(pos)
        except:
            rospy.logerr("Failed to run commands.")
    def linear_length_conversion(self,linear_zero,length_in_mm):
        if length_in_mm<0:
            rospy.logerr("error during length conversion from mm to rotation")
        else:
            length_in_rotation=linear_zero+int(length_in_mm*4096.0/self.pitch)
            return length_in_rotation


if __name__ == "__main__":
    #initialise the class here
    
    rospy.init_node("combined_gripper_server")
    serial_port = rospy.get_param("combined_gripper_server/serial_port", "/dev/ttyUSB0")
    rospy.loginfo("Starting up on serial port: " + serial_port)
    gripper = PrecisionGripper(serial_port)

    my_service = rospy.Service('combined_gripper_command', PrecisionGripperCommand, gripper.my_callback)
    rospy.loginfo("Service combined_gripper is ready")
    rospy.spin()

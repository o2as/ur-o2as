#! /usr/bin/env python
import lib_robotis_xm430 as xm430
import sys
import time
import rospy
import actionlib
import o2as_msgs.msg


class ToolsAction:
    def __init__(self):
        name = rospy.get_name()
        serial_port = rospy.get_param(name + "/serial_port", "/dev/for_docker/blacktool")
        rospy.loginfo("Starting up on serial port: " + serial_port)

        self.setScrew_motor_id = rospy.get_param(name + "/setScrew_motor_id", 75)
        self.peg_motor_id = rospy.get_param(name + "/peg_motor_id", 1)
        self.m10_nut_motor_id = rospy.get_param(name + "/m10_nut_motor_id", 2)
        self.m6_nut_motor_id = rospy.get_param(name + "/m6_nut_motor_id", 3)

        self.dynamixel = xm430.USB2Dynamixel_Device( serial_port, baudrate = 57600 )
        self.p1 = xm430.Robotis_Servo2( self.dynamixel, self.peg_motor_id, series = "XM" )  #Peg (big nut??)
        # self.p2 = xm430.Robotis_Servo2( self.dynamixel, self.m10_nut_motor_id, series = "XM" )  #Big nut
        # self.p3 = xm430.Robotis_Servo2( self.dynamixel, self.m6_nut_motor_id, series = "XM" )  #small nut
        self.p75 = xm430.Robotis_Servo2( self.dynamixel, self.setScrew_motor_id, series = "XM" ) # set screw

        self._feedback = o2as_msgs.msg.ToolsCommandFeedback()
        self._result = o2as_msgs.msg.ToolsCommandResult()
        #define the action
        self._action_name = "nut_tools_action"
        self._action_server = actionlib.SimpleActionServer(self._action_name, o2as_msgs.msg.ToolsCommandAction, execute_cb=self.action_callback, auto_start = False)
        self._action_server.start()
        rospy.loginfo('Action server '+ str(self._action_name)+" started.")

        return

    def action_callback(self, goal):
        # publish info to the console for the user
        rospy.loginfo('Executing'+ str(self._action_name)+"."+"request sent:")
        rospy.loginfo(goal)

        # start executing the action
        command_is_sent = False
        if goal.stop:
            rospy.loginfo("Turning off torque.")
            command_is_sent1 = self.peg_disable_torque()
            command_is_sent2 = self.big_nut_disable_torque()
            command_is_sent3 = self.small_nut_disable_torque()
            command_is_sent75 = self.setScrew_disable_torque()
            
            if command_is_sent1 and command_is_sent2 and command_is_sent3 and command_is_sent75 is True:
                command_is_sent = True
            else:
                command_is_sent = False
                
        elif goal.peg_fasten:
            command_is_sent = self.peg_fasten(50)
            
        elif goal.big_nut_fasten:
            command_is_sent = self.big_nut_fasten(50)
            
        elif goal.small_nut_fasten:
            command_is_sent = self.small_nut_fasten(50)
        
        elif goal.setScrew_fasten:        
            command_is_sent = self.setScrew_fasten(50)
        else:
            rospy.logerr('No command is sent, service request was empty.')
            command_is_sent = False
        
        success = command_is_sent
        if success:
            if goal.stop:
                self._feedback.motor_speed = -1 #an arbitary number higher than self.speed_limit
  
            elif goal.peg_fasten or goal.big_nut_fasten or goal.small_nut_fasten or goal.setScrew_fasten:  
                if goal.peg_fasten:
                    self._feedback.motor_speed = self.p1.read_current_velocity()
                if goal.big_nut_fasten:
                    self._feedback.motor_speed = self.p2.read_current_velocity()
                if goal.small_nut_fasten:
                    self._feedback.motor_speed = self.p3.read_current_velocity()
                if goal.setScrew_fasten:
                    self._feedback.motor_speed = self.p75.read_current_velocity()
             
            self._feedback.countTime = 0
            while self._feedback.motor_speed > 10 and self._feedback.countTime < 100:
                rospy.sleep(0.1)
                self._feedback.countTime += 1
                # check that preempt has not been requested by the client
                if self._action_server.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self._action_server.set_preempted()
                    success = False
                    break
                    
                if goal.peg_fasten or goal.big_nut_fasten or goal.small_nut_fasten or goal.setScrew_fasten:  
                    if goal.peg_fasten:
                        self._feedback.motor_speed = self.p1.read_current_velocity()
                    elif goal.big_nut_fasten:
                        self._feedback.motor_speed = self.p2.read_current_velocity()
                    elif goal.small_nut_fasten:
                        self._feedback.motor_speed = self.p3.read_current_velocity()
                    elif goal.setScrew_fasten:
                        self._feedback.motor_speed = self.p75.read_current_velocity()
                        
                # publish the feedback
                self._action_server.publish_feedback(self._feedback)
                
            if success:
                self._result.success = True
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self._action_server.set_succeeded(self._result)
        else:
            self._action_server.set_preempted()
        self.peg_disable_torque()
        # self.big_nut_disable_torque()
        # self.small_nut_disable_torque()
        self.setScrew_disable_torque()
  
    ######################################################

    def peg_fasten(self, current):
        try:
            self.p1.set_operating_mode("current")
            self.p1.set_positive_direction("ccw")
            self.p1.set_current(current)
            rospy.sleep(0.1)
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False

    def big_nut_fasten(self, current):
        try:
            self.p2.set_operating_mode("current")
            self.p2.set_positive_direction("ccw")
            self.p2.set_current(current)
            rospy.sleep(0.1)
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False
            
    def small_nut_fasten(self, current):
        try:
            self.p3.set_operating_mode("current")
            self.p3.set_positive_direction("ccw")
            self.p3.set_current(current)
            rospy.sleep(0.1)
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False

    def peg_disable_torque(self):
        try:
            self.p1.disable_torque()
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False
            
    def big_nut_disable_torque(self):
        try:
            self.p2.disable_torque()
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False
            
    def small_nut_disable_torque(self):
        try:
            self.p3.disable_torque()
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False

    def setScrew_fasten(self, current):
        try:
            self.p75.set_operating_mode("current")
            self.p75.set_positive_direction("ccw")
            self.p75.set_current(current)
            rospy.sleep(0.1)
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False

    def setScrew_disable_torque(self):
        try:
            self.p75.disable_torque()
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False

        
if __name__ == '__main__':
    rospy.init_node('tools_server')
    server = ToolsAction()
    rospy.spin()

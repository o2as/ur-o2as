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
        serial_port = rospy.get_param(name + "/serial_port", "/dev/ttyUSB0")
        rospy.loginfo("Starting up on serial port: " + serial_port)

        self.setScrew_motor_id = rospy.get_param(name + "/setScrew_motor_id", 75)

        self.dynamixel = xm430.USB2Dynamixel_Device( serial_port, baudrate = 57600 )
        self.p1 = xm430.Robotis_Servo2( self.dynamixel, self.setScrew_motor_id, series = "XM" )  

        self._feedback = o2as_msgs.msg.ToolsCommandFeedback()
        self._result = o2as_msgs.msg.ToolsCommandResult()
        #define the action
        self._action_name = "setScrew_tools_action"
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
            command_is_sent1 = self.setScrew_disable_torque()
            
            if command_is_sent1 and command_is_sent2 and command_is_sent3 is True:
                command_is_sent = True
            else:
                command_is_sent = False
                
        elif goal.setScrew_fasten:        
            command_is_sent = self.setScrew_fasten(30)

        else:
            rospy.logerr('No command is sent, service request was empty.')
            command_is_sent = False
        
        success = command_is_sent
        if success:
            if goal.stop:
                self._feedback.motor_speed = -1 #an arbitary number higher than self.speed_limit
  
            elif goal.setScrew_fasten :  
                if goal.setScrew_fasten:
                    self._feedback.motor_speed = self.p1.read_current_velocity()
             
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
                    
                if goal.setScrew_fasten:  
                    if goal.setScrew_fasten:
                        self._feedback.motor_speed = self.p1.read_current_velocity()

                        
                # publish the feedback
                self._action_server.publish_feedback(self._feedback)
                
            if success:
                self._result.success = True
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self._action_server.set_succeeded(self._result)
        else:
            self._action_server.set_preempted()
        self.setScrew_disable_torque()
  
    ######################################################

    def setScrew_fasten(self, current):
        try:
            self.p1.set_operating_mode("current")
            self.p1.set_positive_direction("ccw")
            self.p1.set_current(current)
            rospy.sleep(0.1)
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False


    def setScrew_disable_torque(self):
        try:
            self.p1.disable_torque()
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False

        
if __name__ == '__main__':
    rospy.init_node('tools_server')
    server = ToolsAction()
    rospy.spin()

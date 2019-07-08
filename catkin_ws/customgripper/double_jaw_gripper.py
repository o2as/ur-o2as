import lib_robotis_xm430 as xm430
import sys
import time
pitch=1.98#9./5.*1.05*1.02#found from experiment
#%%
#joshua
#20180620
dynamixel1 = xm430.USB2Dynamixel_Device( '/dev/ttyUSB0' )
#xm.find_servos(dyn)
#%%
p1 = xm430.Robotis_Servo2( dynamixel1, 1, series = "XM" )#inner gripper
#%%
p2 = xm430.Robotis_Servo2( dynamixel1, 2, series = "XM" )#outer gripper
#%%
p3 = xm430.Robotis_Servo2( dynamixel1, 3, series = "XM" )#outer gripper
#%%
p4 = xm430.Robotis_Servo2( dynamixel1, 4, series = "XM" )#linear actuator
#%%
#outer gripper related functions
def outer_gripper_close_new(goal_position):
    try:
        #%%
        p2.set_operating_mode("currentposition")
        p3.set_operating_mode("currentposition")
        #%%
        p2.set_current(150)#0.3Nm
        p3.set_current(150)
        i2=p2.read_current_position()
        i3=p3.read_current_position()
        i_avg=int((i2+i3)/2)
        while i_avg<goal_position:
            p2.set_goal_position(i_avg)
            p3.set_goal_position(i_avg)
            current2=int(p2.read_current())
            current3=int(p3.read_current())
            #print "current2="+str(current2)
            #print "current3="+str(current3)
            if current2>140 and current2< 60000 and current3>140 and current3<60000:
                break;
            i_avg=i_avg+4
        #%%
    except:
        print "Failed to run commands."
def outer_gripper_close_force():
    try:
        #%%
        p2.set_operating_mode("current")
        p3.set_operating_mode("current")
        #%%
        p2.set_current(150)#0.3Nm
        p3.set_current(150)
        #%%
    except:
        print "Failed to run commands."
def outer_gripper_left_right(goal_position):
    try:
        #%%
        p2.set_operating_mode("currentposition")
        p3.set_operating_mode("currentposition")
        #%%
        p2.set_current(250)#0.3Nm
        p3.set_current(250)
        i2=p2.read_current_position()
        i3=p3.read_current_position()
        i_avg=int((i2+i3)/2)
        i=0
        while i<10:
            if i%2 is 0:
                p2.set_goal_position(i_avg-16)
                p3.set_goal_position(i_avg+16)
            else:
                p2.set_goal_position(i_avg+16)
                p3.set_goal_position(i_avg-16)
            i=i+1
            time.sleep(1)
        #%%
    except:
        print "Failed to run commands."
def outer_gripper_read_current_position():
    try:
        #%%
        x=p2.read_current_position()
        y=p3.read_current_position()
        print "id2="+str(x)
        print "id3="+str(y)
    except:
        print "Failed to run commands."
def outer_gripper_self_centering():
    try:
        #%%
        p2.set_operating_mode("position")
        p3.set_operating_mode("position")
        p2.set_operating_mode("currentposition")
        p3.set_operating_mode("currentposition")
        #%%
        p2.set_current(100)
        p3.set_current(100)
        #%%
        y=p2.read_current_position()
        x=p3.read_current_position()
        x=(x+y)/2
        p2.set_goal_position(x)
        p2.set_goal_position(x)
        y=p2.read_current_position()
        x=p3.read_current_position()
        print "x="+str(x)
        print "y="+str(y)
    except:
        print "Failed to run commands."
def outer_gripper_open_fully():
    try:
        #%%
        p2.set_operating_mode("currentposition")
        p3.set_operating_mode("currentposition")
        #%%
        p2.set_current(100)
        p3.set_current(100)
        #%%
        #%%
        p2.set_goal_position(1024)
        p3.set_goal_position(1024)
        #%%
    except:
        print "Failed to run commands."
def outer_gripper_disable_torque():
    try:
        #%%
        p2.disable_torque()
        p3.disable_torque()
        #%%
    except:
        print "Failed to run commands."
def outer_gripper_open_to(location):#still editing
    try:
        #%%
        p2.set_operating_mode("currentposition")
        p3.set_operating_mode("currentposition")
        #%%
        p2.set_current(100)
        p3.set_current(100)
        #%%
        #%%
        p2.set_goal_position(location)
        p3.set_goal_position(location)
        #%%
    except:
        print "Failed to run commands."
#inner gripper related functions
def inner_gripper_open_fully():
    try:
        #%%
        p1.set_operating_mode("currentposition")
        #%%
        p1.set_current(100)
        #%%
        #%%
        p1.set_goal_position(1024)
        #%%
    except:
        print "Failed to run commands."    
def inner_gripper_open_slightly():
    try:
        #%%
        p1.set_operating_mode("currentposition")
        #%%
        p1.set_current(30)
        current_position=p1.read_current_position()
        #%%
        current_position=current_position-800
        p1.set_goal_position(current_position)
        #%%
    except:
        print "Failed to run commands."
def inner_gripper_close_force():
    try:
        #%%
        p1.set_operating_mode("current")
        #%%
        p1.set_current(100)
        #%%
    except:
        print "Failed to run commands."
def inner_gripper_close_new():
    try:
        #%%
        p1.set_operating_mode("currentposition")
        #%%
        p1.set_current(160)#0.3Nm
        i1=p1.read_current_position()
        while i1<3072:
            p1.set_goal_position(i1)
            current1=int(p1.read_current())
            print current1
            if current1>150 and current1<65000:
                break;
            i1=i1+8
        '''
        while i_avg>goal_position:
            i_avg=i_avg-1
            p2.set_goal_position(i_avg)
            p3.set_goal_position(i_avg)
            current2=p2.read_current()
            current3=p3.read_current()
            if current2>100 and current3>100:
                break;
        '''
        #%%
    except:
        print "Failed to run commands."
def inner_gripper_disable_torque():
    try:
        #%%
        p1.disable_torque()
        #%%
    except:
        print "Failed to run commands."
####linear motor
'''def linear_motor_move_forward():
    try:
        #%%
        p4.set_operating_mode("current")
        #%%
        p4.set_positive_direction("cw")
        p4.set_current(100)
    except:
        print "Failed to run commands."'''
def linear_motor_move(distance):#distance in rotation
    try:
        #%%
        p4.set_operating_mode("currentposition")
        #%%
        p4.set_positive_direction("cw")
        p4.set_current(400)
        p4.set_goal_position(distance)
        #%%
    except:
        print "Failed to run commands."
def linear_motor_disable_torque():
    try:
        #%%
        p4.disable_torque()
        #%%
    except:
        print "Failed to run commands."
#for calibration of linear motor
def linear_motor_calibration_one_move_backward():
    try:
        #%%
        p4.set_operating_mode("current")
        #%%
        p4.set_positive_direction("ccw")
        p4.set_current(100)
        #%%
    except:
        print "Failed to run commands."
def linear_motor_calibration_two():#positive
    try:
        #%%
        p4.set_operating_mode("position")
        p4.set_positive_direction("cw")
        #%%
    except:
        print "Failed to run commands."
def linear_motor_read_current_position():#negative#distance in mm
    try:
        #%%
        pos=p4.read_current_position()
        return int(pos)
    except:
        print "Failed to run commands."
def linear_length_convertion(linear_zero,length_in_mm):
    if length_in_mm<0:
        print "error during length convertion from mm to rotation"
    else:
        length_in_rotation=linear_zero+int(length_in_mm*4096/pitch)
        return length_in_rotation

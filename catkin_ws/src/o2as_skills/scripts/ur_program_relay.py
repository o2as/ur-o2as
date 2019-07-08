#!/usr/bin/env python

import roslib
import rospy
# roslib.load_manifest('ur_program_relay')

import moveit_msgs.msg
import std_msgs.msg
import o2as_msgs.srv
import tf

import os, sys, rospkg


# Node example class.
class URScriptRelay():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.listener = tf.TransformListener()
        self.publishers = {
            'a_bot':rospy.Publisher("/a_bot_controller/ur_driver/URScript", std_msgs.msg.String, queue_size=1), 
            'b_bot':rospy.Publisher("/b_bot_controller/ur_driver/URScript", std_msgs.msg.String, queue_size=1), 
            'c_bot':rospy.Publisher("/c_bot_controller/ur_driver/URScript", std_msgs.msg.String, queue_size=1) }
        rospy.sleep(0.5) # Wait for the publishers to be registered and the listener to receive 
        
        self.rospack = rospkg.RosPack()
        self.read_templates()
        s = rospy.Service('o2as_skills/sendScriptToUR', o2as_msgs.srv.sendScriptToUR, self.srv_callback)

        # Main while loop.
        while not rospy.is_shutdown():
            rospy.sleep(.1)

    # Create a callback function for the service.
    def srv_callback(self, req):
        # Interpret the service parameters, construct the program, send it to the UR
        if not req.robot_name:
            rospy.logerr("robot_name was not defined in the service call to sendScriptToUR!")
            return False
        elif not req.program_id:
            rospy.logerr("No program ID was defined!")
            return False

        if req.program_id == "horizontal_insertion":
            program_front = self.horizontal_insertion_template
            program_back = ""

            # Assign defaults
            if not req.max_force:
                req.max_force = 10.0
            if not req.force_direction:
                req.force_direction = "Y-"
            if not req.forward_speed:
                req.forward_speed = .02
            if not req.max_approach_distance:
                req.max_approach_distance = .1
            if not req.max_radius:
                req.max_radius = .007            # in m
            if not req.radius_increment:
                req.radius_increment = 0.0003    # in m
            # if not req.peck_mode:
            #     req.peck_mode = True
            if not req.max_insertion_distance:
                req.max_insertion_distance = 0.035
            if not req.impedance_mass:
                req.impedance_mass = 10

            ### Function definitions, for reference:
            ### rq_linear_search(direction="Z+",force = 10, speed = 0.004, max_distance = 0.02 )
            ### rq_spiral_search_new(max_insertion_distance, force_threshold = 3, max_radius = 5.0, radius_incr=0.3, peck_mode = False):

            
            program_back += "    rq_zero_sensor()\n"
            program_back += "    textmsg(\"Approaching.\")\n"
            program_back += "    rq_linear_search(\"" + req.force_direction + "\"," \
                                + str(req.max_force) + "," \
                                + str(req.forward_speed) + "," \
                                + str(req.max_approach_distance) + ")\n"
            program_back += "    max_insertion_distance = " + str(req.max_insertion_distance) + "\n"
            program_back += "    textmsg(\"Spiral searching.\")\n"
            program_back += "    sleep(3.0)\n"
            program_back += "    if rq_spiral_search_new(max_insertion_distance," + str(req.max_force) \
                                + ", " + str(req.max_radius*1000) \
                                + ", " + str(req.radius_increment*1000) \
                                + ", peck_mode=" + str(req.peck_mode) + "):\n"
            program_back += "        #Insert the Part into the bore#\n"
            program_back += "        textmsg(\"Impedance insert\")\n"
            program_back += "        sleep(3.0)\n" 
            program_back += "        rq_impedance(max_insertion_distance, " + str(req.impedance_mass) + ")\n"
            program_back += "    end\n"
            program_back += "    textmsg(\"Done. Exiting.\")\n"
            program_back += "end\n"

            

            program = program_front + "\n" + program_back
        elif req.program_id == "insertion" or req.program_id == "insert":
            program_front = self.insertion_template
            program_back = ""

            # Assign defaults
            if not req.max_force:
                req.max_force = 10.0
            if not req.force_direction:
                req.force_direction = "Z+"
            if not req.forward_speed:
                req.forward_speed = .02
            if not req.max_approach_distance:
                req.max_approach_distance = .1
            if not req.max_radius:
                req.max_radius = .004            # in m
            if not req.radius_increment:
                req.radius_increment = 0.0003    # in m
            if not req.peck_mode:
                req.peck_mode = False
            if not req.max_insertion_distance:
                req.max_insertion_distance = 0.035
            if not req.impedance_mass:
                req.impedance_mass = 10

            ### Function definitions, for reference:
            ### rq_linear_search(direction="Z+",force = 10, speed = 0.004, max_distance = 0.02 )
            ### rq_spiral_search_new(max_insertion_distance, force_threshold = 3, max_radius = 5.0, radius_incr=0.3, peck_mode = False):

            
            program_back += "    rq_zero_sensor()\n"
            program_back += "    textmsg(\"Approaching.\")\n"
            program_back += "    rq_linear_search(\"" + req.force_direction + "\"," \
                                + str(req.max_force) + "," \
                                + str(req.forward_speed) + "," \
                                + str(req.max_approach_distance) + ")\n"
            program_back += "    max_insertion_distance = " + str(req.max_insertion_distance) + "\n"
            program_back += "    textmsg(\"Spiral searching.\")\n"
            program_back += "    sleep(3.0)\n"
            program_back += "    if rq_spiral_search_new(max_insertion_distance," + str(req.max_force) \
                                + ", " + str(req.max_radius*1000) \
                                + ", " + str(req.radius_increment*1000) \
                                + ", peck_mode=" + str(req.peck_mode) + "):\n"
            program_back += "        #Insert the Part into the bore#\n"
            program_back += "        textmsg(\"Impedance insert\")\n"
            program_back += "        sleep(3.0)\n" 
            program_back += "        rq_impedance(max_insertion_distance, " + str(req.impedance_mass) + ")\n"
            program_back += "    end\n"
            program_back += "    textmsg(\"Done. Exiting.\")\n"
            program_back += "end\n"

            program = program_front + "\n" + program_back
        elif req.program_id == "linear_push":
            program_front = self.linear_push_template
            program_back = ""

            # Assign defaults
            if not req.max_force:
                req.max_force = 10.0
            if not req.force_direction:
                req.force_direction = "Z+"
            if not req.forward_speed:
                req.forward_speed = .02
            if not req.max_approach_distance:
                req.max_approach_distance = .1

            ### Function definitions, for reference:
            ### rq_linear_search(direction="Z+",force = 10, speed = 0.004, max_distance = 0.02 )
            
            program_back += "    rq_zero_sensor()\n"
            program_back += "    textmsg(\"Approaching linearly.\")\n"
            program_back += "    rq_linear_search(\"" + req.force_direction + "\"," \
                                + str(req.max_force) + "," \
                                + str(req.forward_speed) + "," \
                                + str(req.max_approach_distance) + ")\n"
            program_back += "    textmsg(\"Done.\")\n"
            program_back += "end\n"

            program = program_front + "\n" + program_back
        elif req.program_id == "lin_move":
            rospy.logdebug("UR script lin move uses the ee_link of the robot, not the EE of the move group.") 
            if not req.acceleration:
                req.acceleration = 0.5
            if not req.velocity:
                req.velocity = .03
            rospy.logdebug("original pose:")
            rospy.logdebug(req.target_pose)
            transform_success = False
            counter = 50
            while not transform_success:
                try:
                    counter += 1
                    robot_pose = self.listener.transformPose(req.robot_name + "_base", req.target_pose)
                    transform_success = True
                except:
                    rospy.logdebug("Failed to transform from frame " + req.target_pose.header.frame_id + ". Waiting for .1 seconds")
                    rospy.sleep(.1)
            xyz = [robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z]
            
            q = [robot_pose.pose.orientation.x, robot_pose.pose.orientation.y, 
                 robot_pose.pose.orientation.z, robot_pose.pose.orientation.w]
            rpy = tf.transformations.euler_from_quaternion(q)
            # rpy needs to be in axis-angle representation
            # http://www.zacobria.com/universal-robots-knowledge-base-tech-support-forum-hints-tips/python-code-example-of-converting-rpyeuler-angles-to-rotation-vectorangle-axis-for-universal-robots/
            rospy.logdebug("q in robot base:")
            rospy.logdebug(q)

            # This seems to work, but it uses the ee_link TCP of the robot.
            program = ""
            program += "def move_to_pose_lin():\n"
            program += "    textmsg(\"Move_l to a pose.\")\n"
            program += "    rv = rpy2rotvec([" + str(rpy[0]) + "," + str(rpy[1]) + "," + str(rpy[2]) + "])\n"
            program += "    target_pos=p[" + str(xyz[0]) + "," + str(xyz[1]) + "," + str(xyz[2]) + "," \
                                      "rv[0], rv[1], rv[2]]\n"
            program += "    movel(pose_trans(p[0.0,0.0,0.0,0.0,0.0,0.0], target_pos), " + \
                            "a = " + str(req.acceleration) + ", v = " + str(req.velocity) + ")\n"
            # program += "    movel(target_pos, " + \
            #                 "a = " + str(req.acceleration) + ", v = " + str(req.velocity) + ")\n"
            program += "    textmsg(\"Done.\")\n"
            program += "end\n"
            rospy.logdebug(program)
        elif req.program_id == "lin_move_rel":
            if not req.acceleration:
                req.acceleration = 0.5
            if not req.velocity:
                req.velocity = .03
            # rospy.logwarn("The frame_id of the Point is ignored!")
            xyz = [req.relative_translation.x, req.relative_translation.y, req.relative_translation.z]

            program = ""
            program += "def move_lin_rel():\n"
            program += "    textmsg(\"Move_l via relative translation.\")\n"
            program += "    current_pos = get_actual_tcp_pose()\n"
            program += "    offset_pose = p[" + str(xyz[0]) + ", " + str(xyz[1]) + ", " + str(xyz[2]) + ", 0.0, 0.0, 0.0]\n"
            program += "    movel(pose_trans(current_pos, offset_pose), " + \
                            "a = " + str(req.acceleration) + ", v = " + str(req.velocity) + ")\n"
            program += "    textmsg(\"Done.\")\n"
            program += "end\n"
        elif req.program_id == "spiral_press":
            rospy.logerr("SPIRAL PRESS IS NOT IMPLEMENTED YET") # TODO: Is this not just the "insertion" script with peck_mode=False?
        elif req.program_id == "spiral_motion":
            program_front = self.spiral_motion_template
            program_back = ""
            if (req.radius_increment < 0.0001) or (req.radius_increment > 0.005):
                rospy.logerr("radius_incr needs to be between 0.0001 and 0.005 but is " + str(req.radius_increment))
            if not req.acceleration:
                req.acceleration = 0.1
            if not req.velocity:
                req.velocity = .03
            if not req.max_radius:
                req.max_radius = .0065
            if not req.radius_increment:
                req.radius_increment = .002
            if not req.theta_increment:
                req.theta_increment = 30
            if not req.spiral_axis:
                req.spiral_axis = "Z"
            
            program_back += "    textmsg(\"Performing spiral motion.\")\n"
            program_back += "    spiral_motion(" + str(req.max_radius) \
                                + ", " + str(req.radius_increment) \
                                + ", " + str(req.velocity) \
                                + ", " + str(req.acceleration) \
                                + ", \"" + req.spiral_axis + "\"" \
                                + ", " + str(req.theta_increment) + ")\n"
            program_back += "    textmsg(\"Done.\")\n"
            program_back += "end\n"

            program = program_front + "\n" + program_back
        elif req.program_id == "test":
            program = ""
            program_file = open(os.path.join(self.rospack.get_path("o2as_examples"), "scripts/urscript", "move_back_forth_5cm.script"), 'rb')
            program_line = program_file.read(1024)
            while program_line:
                program += program_line
                program_line = program_file.read(1024)
        elif req.program_id == "move_j":

            if not len(req.joint_positions) == 6:
                rospy.logwarn("Joint pose vector not of the correct length")
                return False
            if not req.acceleration:
                req.acceleration = 0.5
            if not req.velocity:
                req.velocity = 0.5

            program = ""
            program += "def move_to_joint_pose():\n"
            program += "    textmsg(\"Move_j to a pose.\")\n"
            program += "    target_pos=[" + str(req.joint_positions[0]) + "," + str(req.joint_positions[1]) + "," + str(req.joint_positions[2]) + "," \
                                      + str(req.joint_positions[3]) + "," + str(req.joint_positions[4]) + "," + str(req.joint_positions[5]) + "]\n"
            program += "    movej(target_pos, " + \
                            "a = " + str(req.acceleration) + ", v = " + str(req.velocity) + ")\n"
            program += "    textmsg(\"Done.\")\n"
            program += "end\n"
            rospy.logdebug(program)
        else:
            rospy.logerr("The program could not be recognized: " + req.program_id)
            return False

        # Send the program to the robot
        program_msg = std_msgs.msg.String()
        program_msg.data = program

        rospy.loginfo("Sending UR robot program " + req.program_id)
        # rospy.logdebug("Program is:")
        # rospy.logdebug(program)
        self.publishers[req.robot_name].publish(program_msg)
        return True

    def read_templates(self):
        # Read the files containing the program templates into memory
        self.insertion_template = self.read_template("peginholespiral_imp_osx.script")
        self.horizontal_insertion_template = self.read_template("peginholespiral_imp_osx_y_negative.script")
        self.linear_push_template = self.read_template("linear_search_short.script")
        self.spiral_motion_template = self.read_template("spiral_motion.script")
        return True

    def read_template(self, filename):
        program_template_file = open(os.path.join(self.rospack.get_path("o2as_skills"), "src/urscript", filename), 'rb')
        program_line = program_template_file.read(1024)
        linecounter = 0
        template = ""
        while program_line:
            template += program_line
            program_line = program_template_file.read(1024)
        return template


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('o2as_urscript_construction_node')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = URScriptRelay()
    except rospy.ROSInterruptException: pass

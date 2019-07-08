#!/usr/bin/env python2

import rospy
from tf import TransformBroadcaster, TransformListener, TransformerROS, transformations as tfs
from geometry_msgs.msg import Transform
from math import radians, degrees
from easy_handeye.handeye_calibration import HandeyeCalibration

#########################################################################
#  local functions                                                      #
#########################################################################
def get_transform(dst_frm, src_frm):
    listener = TransformListener()
    now = rospy.Time.now()
    listener.waitForTransform(dst_frm, src_frm, now, rospy.Duration(10))
    return listener.lookupTransform(dst_frm, src_frm, now)

def print_mat(mat):
    xyz = tfs.translation_from_matrix(mat)
    rpy = map(degrees, tfs.euler_from_matrix(mat))
    print "<origin xyz=\"{0[0]} {0[1]} {0[2]}\" rpy=\"${{{1[0]}*pi/180}} ${{{1[1]}*pi/180}} ${{{1[2]}*pi/180}}\"/>".format(xyz, rpy)
    q   = tfs.quaternion_from_matrix(mat)
    print xyz, q
    
#########################################################################
#  main part                                                            #
#########################################################################
rospy.init_node('o2as_handeye_calibration_publisher')
while rospy.get_time() == 0.0:
    pass

calib = HandeyeCalibration()
calib.from_file()

if calib.eye_on_hand:
    overriding_robot_effector_frame = rospy.get_param('robot_effector_frame')
    if overriding_robot_effector_frame != "":
        calib.transformation.header.frame_id = overriding_robot_effector_frame
else:
    overriding_robot_base_frame = rospy.get_param('robot_base_frame')
    if overriding_robot_base_frame != "":
        calib.transformation.header.frame_id = overriding_robot_base_frame
overriding_tracking_base_frame = rospy.get_param('tracking_base_frame')
if overriding_tracking_base_frame != "":
    calib.transformation.child_frame_id = overriding_tracking_base_frame

rospy.loginfo('loading calibration parameters into namespace {}'.format(rospy.get_namespace()))
calib.to_parameters()

# Estimated camera -> robot(effector or base_link) transformation
result_tf = calib.transformation.transform
trns      = result_tf.translation.x, result_tf.translation.y, \
            result_tf.translation.z
rot       = result_tf.rotation.x, result_tf.rotation.y, \
            result_tf.rotation.z, result_tf.rotation.w
bot       = calib.transformation.header.frame_id  # effector or base link
optEst    = calib.transformation.child_frame_id   # tracking_base_frame

# Compute camera base -> camera_optical transformation
opt_base  = get_transform(rospy.get_param('camera_optical_frame'),
                          rospy.get_param('camera_base_frame'))
# Compute tracking_base_frame -> o2as_ground transformation
if not calib.eye_on_hand:
    grnd_bot  = get_transform('o2as_ground', bot)

broad     = TransformBroadcaster()
rate      = rospy.Rate(50)
baseEst   = rospy.get_param('camera_body_frame')

try:
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        broad.sendTransform(trns, rot, now, optEst, bot)  # ..., child, parent
        broad.sendTransform(opt_base[0], opt_base[1], now, baseEst, optEst)
        rate.sleep()
except rospy.ROSInterruptException:
    transformer = TransformerROS()
    mat = transformer.fromTranslationRotation(trns, rot)
    print "\n=== Estimated camera -> robot(effector or base_link) transformation ==="
    print_mat(mat)

    if not calib.eye_on_hand:
        mat = tfs.concatenate_matrices( \
                transformer.fromTranslationRotation(*grnd_bot), mat,
                transformer.fromTranslationRotation(*opt_base))
        print "\n=== Estimated camera_base -> ground transformation ==="
        print_mat(mat)

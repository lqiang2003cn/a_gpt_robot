#!/usr/bin/env python

import rospy
from yahboomcar_msgs.srv import RobotArmArray, RobotArmArrayRequest
from yahboomcar_msgs.msg import ArmJoint
from time import sleep
import tf


def get_current_angles():
    curr_angle = rospy.ServiceProxy("CurrentAngle", RobotArmArray)
    request = RobotArmArrayRequest()
    response = curr_angle.call(request)
    return list(response.angles)


def publish_joints(joints):
    joints_pub = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1000)
    arm_joint = ArmJoint()
    arm_joint.run_time = 5000
    arm_joint.joints = joints
    for i in range(10):
        joints_pub.publish(arm_joint)
        sleep(0.1)


def query_pose(listener, target_frame, source_frame):
    pos, rot = None, None
    while not rospy.is_shutdown():
        try:
            pos, rot = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    return pos, rot

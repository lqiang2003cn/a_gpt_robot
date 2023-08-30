#!/usr/bin/env python

from __future__ import print_function

import copy

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped, Pose
from tf.transformations import euler_from_quaternion
from copy import deepcopy


def query_pose(tf_lst, target_frame, source_frame):
    tf_pos, tf_rot = None, None
    while not rospy.is_shutdown():
        try:
            tf_pos, tf_rot = tf_lst.lookupTransform(target_frame, source_frame, rospy.Time(0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    return tf_pos, tf_rot


def static_transform_for_tool(parent_frame, child_frame, ):
    listener.transformPoint("tube_frame", )

    pre_pick_pose = deepcopy(obj_pose)
    pre_pick_pose.position.z = obj_pose.position.z + 0.15
    grasp_pos, grasp_ori = query_pose(listener, "map", "gripper_left_grasping_frame")

    center_pos = deepcopy(grasp_pos)
    center_ori = deepcopy(grasp_ori)
    center_pos[0] += 0.05



    # gripper_center =
    print("c")
    # static_transformStamped = TransformStamped()
    #
    # static_transformStamped.header.stamp = rospy.Time.now()
    # static_transformStamped.header.frame_id = parent_frame
    # static_transformStamped.child_frame_id = child_frame
    #
    # static_transformStamped.transform.translation.x = x
    # static_transformStamped.transform.translation.y = y
    # static_transformStamped.transform.translation.z = z
    #
    # static_transformStamped.transform.rotation.x = qx
    # static_transformStamped.transform.rotation.y = qy
    # static_transformStamped.transform.rotation.z = qz
    # static_transformStamped.transform.rotation.w = qw
    #
    broadcaster.sendTransform(static_transformStamped)


if __name__ == '__main__':
    try:
        rospy.init_node('tiago_frame_publisher')
        rate = rospy.Rate(10)
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        listener = tf.TransformListener()
        while not rospy.is_shutdown():
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = -3.80, 8.00, 0.84
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = 0, 0, 0, 1
            static_transform_for_tool("map", "arm_left_tool_pick_frame", pose)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

# listener = tf.TransformListener()
#
# pos, rot = query_pose(listener, "map", "tube_pre_pick")
# print("pos:", pos)
# print("rot quat:", rot)
# print("rot euler:\n", euler_from_quaternion(rot))
# m = quaternion_matrix(rot)
# print("rot matrix:\n", m)
#
# m_x = m[0:3, 0]
# m_y = m[0:3, 1]
# m_z = m[0:3, 2]
# m_new = np.zeros((4, 4))
# m_new[3, 3] = 1
# m_new[0:3, 0] = -1 * m_x
# m_new[0:3, 1] = 1 * m_y
# m_new[0:3, 2] = -1 * m_z
# new_quat = quaternion_from_matrix(m_new)
# print("rot new quat:\n", new_quat)

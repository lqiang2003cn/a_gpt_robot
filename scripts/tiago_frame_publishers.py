#!/usr/bin/env python

from __future__ import print_function

import numpy as np
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf.transformations import *

from tiago_services import get_object_above_pose
from utils import query_pose


def static_transform_for_tool(parent_frame, child_frame, obj_pose):
    new_ps = None
    while not rospy.is_shutdown():
        try:
            tube_ps = listener.transformPose("map", obj_pose)
            tube_prepick_frame = listener.transformPose("map", obj_pose)
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    return new_ps


def get_transform_pose_stamped(trans_form_frame_id, x, y, z, qx, qy, qz, qw):
    ps_temp = PoseStamped()
    ps_temp.header.frame_id = trans_form_frame_id
    ps_temp.pose.position.x, ps_temp.pose.position.y, ps_temp.pose.position.z = x, y, z
    ps_temp.pose.orientation.x, ps_temp.pose.orientation.y, ps_temp.pose.orientation.z, ps_temp.pose.orientation.w = qx, qy, qz, qw
    return ps_temp


def get_matrix_from_pose(pos, quat):
    t_mat = translation_matrix(pos)
    q_mat = quaternion_matrix(quat)
    p_mat = numpy.dot(t_mat, q_mat)
    return p_mat


def create_transform_stamped(parent_frame_id, child_frame_id, obj_pos, obj_quat):
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = parent_frame_id
    transform_stamped.child_frame_id = child_frame_id
    transform_stamped.transform.translation.x = obj_pos[0]
    transform_stamped.transform.translation.y = obj_pos[1]
    transform_stamped.transform.translation.z = obj_pos[2]
    transform_stamped.transform.rotation.x = obj_quat[0]
    transform_stamped.transform.rotation.y = obj_quat[1]
    transform_stamped.transform.rotation.z = obj_quat[2]
    transform_stamped.transform.rotation.w = obj_quat[3]
    return transform_stamped


if __name__ == '__main__':
    try:
        rospy.init_node('tiago_frame_publisher')
        rate = rospy.Rate(10)
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        listener = tf.TransformListener()
        prepick_diff = np.array([0, 0, -0.25])

        tube_pos, tube_quat = query_pose(listener, "odom", "tube_frame")
        # tube_frame_mat = get_matrix_from_pose(tube_pos, tube_quat)
        #
        # bf_pos, bf_quat = query_pose(listener, "odom", "base_footprint")
        # bf_frame_mat = get_matrix_from_pose(bf_pos, bf_quat)
        #
        # gg_pos, gg_quat = query_pose(listener, "odom", "gripper_left_grasping_frame")
        # gg_frame_mat = get_matrix_from_pose(gg_pos, gg_quat)
        while not rospy.is_shutdown():
            prepick_pos, prepick_quat = get_object_above_pose(listener, tube_pos, tube_quat, prepick_diff)
            # m_new = numpy.eye(4, 4)
            #
            # tube_x = tube_frame_mat[0:3, 0]
            # tube_y = tube_frame_mat[0:3, 1]
            # tube_z = tube_frame_mat[0:3, 2]
            #
            # bf_x = bf_frame_mat[0:3, 0]
            #
            # angle = angle_between(bf_x, tube_x)
            #
            # if angle < math.pi / 2:
            #     m_new[0:3, 0] = tube_x
            #     m_new[0:3, 1] = -tube_y
            #     m_new[0:3, 2] = -tube_z
            # else:
            #     m_new[0:3, 0] = -tube_x
            #     m_new[0:3, 1] = tube_y
            #     m_new[0:3, 2] = -tube_z
            #
            # prepick_pos = tube_pos + np.array([0, 0, 0.2])
            # prepick_quat = quaternion_from_matrix(m_new)

            prepick_transform = create_transform_stamped("odom", "prepick_pose", prepick_pos, prepick_quat)
            broadcaster.sendTransform(prepick_transform)

            # prepick_frame_mat = get_matrix_from_pose(prepick_pos, prepick_quat)
            # prepick_to_tool_mat = get_matrix_from_pose([-0.201, 0, 0], [-0.707, -0.000, -0.000, 0.707])
            # tool_frame_mat = numpy.dot(prepick_frame_mat, prepick_to_tool_mat)
            # tool_pos = translation_from_matrix(tool_frame_mat)
            # tool_quat = quaternion_from_matrix(tool_frame_mat)
            # tool_transform = create_transform_stamped("odom", "tool_pose", tool_pos, tool_quat)
            # broadcaster.sendTransform(tool_transform)

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


# trans1, rot1) = tf.lookupTransform(l2, l1, t)
# trans1_mat = tf.transformations.translation_matrix(trans1)
# rot1_mat   = tf.transformations.quaternion_matrix(rot1)
# mat1 = numpy.dot(trans1_mat, rot1_mat)
#
# (trans2, rot2) = tf.lookupTransform(l4, l3, t)
# trans2_mat = tf.transformations.translation_matrix(trans2)
# rot2_mat    = tf.transformations.quaternion_matrix(rot2)
# mat2 = numpy.dot(trans2_mat, rot2_mat)
#
# mat3 = numpy.dot(mat1, mat2)
# trans3 = tf.transformations.translation_from_matrix(mat3)
# rot3 = tf.transformations.quaternion_from_matrix(mat3)
#
# br = tf.TransformBroadcaster()
# br.sendTransform(
# trans3,
# rot3,
# t,
# "target",
# "source");


# pre_pick_pose = deepcopy(obj_pose)
# pre_pick_pose.position.z = obj_pose.position.z + 0.15
# grasp_pos, grasp_ori = query_pose(listener, "map", "gripper_left_grasping_frame")
#
# center_pos = deepcopy(grasp_pos)
# center_ori = deepcopy(grasp_ori)
# center_pos[0] += 0.05

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
# broadcaster.sendTransform(static_transformStamped)

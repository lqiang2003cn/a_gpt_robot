#!/usr/bin/env python

from __future__ import print_function

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf.transformations import *


def query_pose(tf_lst, target_frame, source_frame):
    tf_pos, tf_rot = None, None
    while not rospy.is_shutdown():
        try:
            tf_pos, tf_rot = tf_lst.lookupTransform(target_frame, source_frame, rospy.Time(0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    return tf_pos, tf_rot


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


if __name__ == '__main__':
    try:
        rospy.init_node('tiago_frame_publisher')
        rate = rospy.Rate(10)
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        listener = tf.TransformListener()
        while not rospy.is_shutdown():
            tube_pos = [-3.8, 8.0, 0.84]
            tube_quat = [0, 0, 0, 1]
            tube_frame_mat = get_matrix_from_pose(tube_pos, tube_quat)

            prepick_pos = tube_pos + [0, 0, 0.2]
            m_x = tube_frame_mat[0:3, 0]
            m_y = tube_frame_mat[0:3, 1]
            m_z = tube_frame_mat[0:3, 2]
            m_new = numpy.zeros((4, 4))
            m_new[3, 3] = 1
            m_new[0:3, 0] = -1 * m_x
            m_new[0:3, 1] = 1 * m_y
            m_new[0:3, 2] = -1 * m_z
            prepick_quat = quaternion_from_matrix(m_new)

            prepick_frame_mat = get_matrix_from_pose(prepick_pos, prepick_quat)
            prepick_to_tool_mat = get_matrix_from_pose([-0.201, 0, 0], [-0.707, -0.000, -0.000, 0.707])
            tool_frame_mat = numpy.dot(prepick_frame_mat, prepick_to_tool_mat)
            tool_pos = translation_from_matrix(tool_frame_mat)
            tool_quat = quaternion_from_matrix(tool_frame_mat)

            static_transformStamped = TransformStamped()
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "map"
            static_transformStamped.child_frame_id = "prepick_for_tool"
            static_transformStamped.transform.translation.x = tool_pos[0]
            static_transformStamped.transform.translation.y = tool_pos[1]
            static_transformStamped.transform.translation.z = tool_pos[2]
            static_transformStamped.transform.rotation.x = tool_quat[0]
            static_transformStamped.transform.rotation.y = tool_quat[1]
            static_transformStamped.transform.rotation.z = tool_quat[2]
            static_transformStamped.transform.rotation.w = tool_quat[3]
            broadcaster.sendTransform(static_transformStamped)

            # grasping_frame = query_pose(listener, "map", "gripper_left_grasping_frame")
            # grasping_frame_mat = get_matrix_from_pose(grasping_frame[0], grasping_frame[1])
            # grasping_to_center_trans = get_matrix_from_pose([0.05, 0, 0], [0, 0, 0, 1])
            # center_frame_mat = numpy.dot(grasping_frame_mat, grasping_to_center_trans)

            # ps = get_transform_pose_stamped("map", -3.8, 8.0, 0.84, 0, 0, 0, 1)
            # static_transform_for_tool("map", "arm_left_tool_pick_frame", ps)
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

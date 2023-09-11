#!/usr/bin/env python

import math

import numpy as np
import rospy
import tf
from tf.transformations import translation_matrix, quaternion_matrix, quaternion_from_matrix, translation_from_matrix


def get_pos_and_quat_from_matrix(obj_mat):
    pos = translation_from_matrix(obj_mat)
    quat = quaternion_from_matrix(obj_mat)
    return pos, quat


def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def abs_angle_between(v1, v2):
    angle = angle_between(v1, v2)
    if angle > math.pi / 2:
        angle = math.pi - angle
    return angle


def query_pose(listener, target_frame, source_frame):
    pos, rot = None, None
    while not rospy.is_shutdown():
        try:
            pos, rot = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    return pos, rot


def get_matrix_from_pos_and_quat(pos, quat):
    t_mat = translation_matrix(pos)
    q_mat = quaternion_matrix(quat)
    p_mat = np.dot(t_mat, q_mat)
    return p_mat


def get_object_above_pose(listener, obj_pose, prepick_diff):
    m_new = np.eye(4, 4)
    obj_pos, obj_quat = get_pos_and_quat_from_matrix(obj_pose)

    bf_pos, bf_quat = query_pose(listener, "odom", "base_footprint")
    bf_frame_mat = get_matrix_from_pos_and_quat(bf_pos, bf_quat)
    bf_x = bf_frame_mat[0:3, 0]
    bf_z = bf_frame_mat[0:3, 2]

    tube_frame_mat = get_matrix_from_pos_and_quat(obj_pos, obj_quat)

    # up axis
    up_axis = -1
    abs_up_angle_min = np.inf
    up_direction = None
    for i in range(3):
        axis = tube_frame_mat[0:3, i]
        abs_up_angle = abs_angle_between(bf_z, axis)
        if abs_up_angle < abs_up_angle_min:
            abs_up_angle_min = abs_up_angle
            up_axis = i
            up_angle = angle_between(bf_z, axis)
            if up_angle <= math.pi / 2:
                up_direction = 1
            else:
                up_direction = -1

    # front axis
    front_axis = -1
    abs_front_angle_min = np.inf
    front_direction = None
    for i in range(3):
        if i != up_axis:
            axis = tube_frame_mat[0:3, i]
            abs_front_angle = abs_angle_between(bf_x, axis)
            if abs_front_angle < abs_front_angle_min:
                abs_front_angle_min = abs_front_angle
                front_axis = i
                front_angle = angle_between(bf_x, axis)
                if front_angle <= math.pi / 2:
                    front_direction = 1
                else:
                    front_direction = -1

    m_new[0:3, 0] = tube_frame_mat[0:3, front_axis] * front_direction
    m_new[0:3, 2] = -1 * tube_frame_mat[0:3, up_axis] * up_direction
    m_new[0:3, 1] = np.cross(m_new[0:3, 2], m_new[0:3, 0])
    m_new_quat = quaternion_from_matrix(m_new)
    prepick_mat = get_matrix_from_pos_and_quat(obj_pos, m_new_quat)

    prepick_transform_mat = get_matrix_from_pos_and_quat(prepick_diff, [0, 0, 0, 1])
    prepick_mat = np.dot(prepick_mat, prepick_transform_mat)
    # prepick_pose_pos, prepick_pose_quat = get_pose_from_matrix(prepick_mat)

    return prepick_mat


def pose_by_diff(pose_mat, pos_diff, quat_diff):
    diff_mat = get_matrix_from_pos_and_quat(pos_diff, quat_diff)
    # pose_mat = get_matrix_from_pose(pose_pos, pose_quat)
    transformed_mat = np.dot(pose_mat, diff_mat)
    return transformed_mat


def center_to_tool(center_pose, center_to_tool_transform):
    res = np.dot(center_pose, center_to_tool_transform)
    return res

    # angle = angle_between(sr_table_333_front_x, tube_x)
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
    # prepick_pos = obj_pos + prepick_diff
    # prepick_quat = quaternion_from_matrix(m_new)
    #
    # prepick_frame_mat = get_matrix_from_pose(prepick_pos, prepick_quat)
    # prepick_to_tool_mat = get_matrix_from_pose(gripper_center_to_tool_pos, gripper_center_to_tool_quat)
    # tool_frame_mat = np.dot(prepick_frame_mat, prepick_to_tool_mat)
    # return get_pose_from_matrix(tool_frame_mat)

# def call_service_func(sn, args):
#     rospy.wait_for_service(sn)
#     try:
#         if sn not in services:
#             services[sn] = roslibpy.Service(ros_client, sn, ros_client.get_service_type(sn))
#         req = roslibpy.ServiceRequest(args)
#         resp = services[sn].call(req)
#         rospy.sleep(2)
#         return resp
#     except Exception as e:
#         print("Service call failed: %s" % e)

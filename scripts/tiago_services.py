#!/usr/bin/env python

import sys

import actionlib
import geometry_msgs
import moveit_commander
import numpy as np
import rospy
import tf
import tf2_ros
from a_gpt_robot.srv import MovePose, MovePoseResponse
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, Empty
from tf.transformations import translation_matrix, quaternion_matrix, quaternion_from_matrix, translation_from_matrix, \
    quaternion_from_euler


def wait_for_state_update(object_name, box_is_known=False, box_is_attached=False, timeout=4):
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in attached objects
        attached_objects = scene.get_attached_objects([object_name])
        is_attached = len(attached_objects.keys()) > 0

        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        is_known = object_name in scene.get_known_object_names()

        # Test if we are in the expected state
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False


def move_base_to_pose(srv_request):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    pose = pose_dict[srv_request.pose_str]

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = pose["frame_id"]
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pose["position"][0]
    goal.target_pose.pose.position.y = pose["position"][1]
    goal.target_pose.pose.orientation.x = pose["orientation"][0]
    goal.target_pose.pose.orientation.y = pose["orientation"][1]
    goal.target_pose.pose.orientation.z = pose["orientation"][2]
    goal.target_pose.pose.orientation.w = pose["orientation"][3]

    client.send_goal(goal)
    client.wait_for_result()
    return MovePoseResponse(1)


def move_arm_to_pose(srv_request):
    arm_pose = pose_dict[srv_request.pose_str]

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = arm_pose["frame_id"]
    pose_goal.pose.position.x = arm_pose["position"][0]
    pose_goal.pose.position.y = arm_pose["position"][1]
    pose_goal.pose.position.z = arm_pose["position"][2]
    pose_goal.pose.orientation.x = arm_pose["orientation"][0]
    pose_goal.pose.orientation.y = arm_pose["orientation"][1]
    pose_goal.pose.orientation.z = arm_pose["orientation"][2]
    pose_goal.pose.orientation.w = arm_pose["orientation"][3]
    move_group.set_pose_target(pose_goal)

    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()


def close_gripper(trigger):
    assert trigger is not None
    rospy.wait_for_service('/parallel_gripper_left_controller/grasp')
    try:
        grasp = rospy.ServiceProxy('/parallel_gripper_left_controller/grasp', Empty)
        resp1 = grasp()
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def open_gripper(trigger):
    assert trigger is not None
    rospy.wait_for_service('/parallel_gripper_left_controller/release')
    try:
        release = rospy.ServiceProxy('/parallel_gripper_left_controller/release', Empty)
        resp1 = release()
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def create_tiago_services():
    rospy.Service('move_base_to_pose', MovePose, move_base_to_pose)
    rospy.Service('move_arm_to_pose', MovePose, move_arm_to_pose)
    rospy.Service('close_gripper', Trigger, close_gripper)
    rospy.Service('open_gripper', Trigger, open_gripper)
    print "tiago services are ready"
    rospy.spin()


def static_transform(parent_frame_id, child_frame_id, transform_pose):
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = parent_frame_id
    static_transformStamped.child_frame_id = child_frame_id

    static_transformStamped.transform.translation.x = transform_pose['position'][0]
    static_transformStamped.transform.translation.y = transform_pose['position'][1]
    static_transformStamped.transform.translation.z = transform_pose['position'][2]

    static_transformStamped.transform.rotation.x = transform_pose['orientation'][0]
    static_transformStamped.transform.rotation.y = transform_pose['orientation'][1]
    static_transformStamped.transform.rotation.z = transform_pose['orientation'][2]
    static_transformStamped.transform.rotation.w = transform_pose['orientation'][3]

    broadcaster.sendTransform(static_transformStamped)


def get_matrix_from_pose(pos, quat):
    t_mat = translation_matrix(pos)
    q_mat = quaternion_matrix(quat)
    p_mat = np.dot(t_mat, q_mat)
    return p_mat


def get_object_prepick(obj_pos, obj_quat):
    tube_frame_mat = get_matrix_from_pose(obj_pos, obj_quat)

    prepick_pos = np.array(obj_pos) + np.array(prepick_diff)
    m_x = tube_frame_mat[0:3, 0]
    m_y = tube_frame_mat[0:3, 1]
    m_z = tube_frame_mat[0:3, 2]
    m_new = np.zeros((4, 4))
    m_new[3, 3] = 1
    # m_new[0:3, 0] = -1 * m_x
    m_new[0:3, 0] = m_x
    m_new[0:3, 1] = 1 * m_y
    m_new[0:3, 2] = -1 * m_z
    prepick_quat = quaternion_from_matrix(m_new)

    prepick_frame_mat = get_matrix_from_pose(prepick_pos, prepick_quat)
    prepick_to_tool_mat = get_matrix_from_pose(gripper_center_to_tool_pos, gripper_center_to_tool_quat)
    tool_frame_mat = np.dot(prepick_frame_mat, prepick_to_tool_mat)
    tool_pos = translation_from_matrix(tool_frame_mat)
    # tool_pos[1] -= 0.08
    tool_quat = quaternion_from_matrix(tool_frame_mat)
    return tool_pos, tool_quat


def add_table(object_name, object_pos, object_euler, timeout=4):
    object_pose = geometry_msgs.msg.PoseStamped()
    object_pose.header.frame_id = "odom"

    object_pose.pose.position.x = object_pos[0]
    object_pose.pose.position.y = object_pos[1]
    object_pose.pose.position.z = object_pos[2]

    q = quaternion_from_euler(object_euler[0], object_euler[1], object_euler[2])
    object_pose.pose.orientation.x = q[0]
    object_pose.pose.orientation.y = q[1]
    object_pose.pose.orientation.z = q[2]
    object_pose.pose.orientation.w = q[3]

    scene.add_box(object_name, object_pose, size=(1 + 1, 0.8 + 0.1, 0.83))
    return wait_for_state_update(object_name, box_is_known=True, timeout=timeout)


def add_box(object_name, timeout=4):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "odom"

    box_pose.pose.position.x = orange_pos[0]
    box_pose.pose.position.y = orange_pos[1]
    box_pose.pose.position.z = orange_pos[2]

    box_pose.pose.orientation.x = orange_quat[0]
    box_pose.pose.orientation.y = orange_quat[1]
    box_pose.pose.orientation.z = orange_quat[2]
    box_pose.pose.orientation.w = orange_quat[3]

    scene.add_box(object_name, box_pose, size=(0.05, 0.05, 0.05))
    return wait_for_state_update(object_name, box_is_known=True, timeout=timeout)


if __name__ == "__main__":
    rospy.init_node('create_tiago_services')
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(2)
    move_group = moveit_commander.MoveGroupCommander("arm_left_torso")
    gripper_group = moveit_commander.MoveGroupCommander("gripper_left")
    listener = tf.TransformListener()

    office_room_table_111_x = 4.2
    office_room_table_111_y = -3
    office_room_table_26_x = -4.2
    office_room_table_26_y = -3
    office_room_table_63_x = -4.2
    office_room_table_63_y = 3
    office_room_table_238_x = 4.2
    office_room_table_238_y = 3
    stock_room_table_333_x = -4
    stock_room_table_333_y = 8

    orange_pos = [0.8, 0, 0.84]
    orange_quat = [0, 0, 0, 1]
    table_x_diff = 1.05
    prepick_diff = [0, 0, 0.2]
    gripper_center_to_tool_pos = [-0.201, 0, 0]
    gripper_center_to_tool_quat = [-0.707, -0.000, -0.000, 0.707]

    prepick_tool_pos, prepick_tool_quat = get_object_prepick(orange_pos, orange_quat)

    pose_dict = {
        "office room table 111": {
            "frame_id": "map",
            "position": [office_room_table_111_x - table_x_diff, office_room_table_111_y, 0],
            "orientation": [0, 0, 0, 1]
        },
        "office room table 26": {
            "frame_id": "map",
            "position": [office_room_table_26_x + table_x_diff, office_room_table_26_y, 0],
            "orientation": [0, 0, 1, 0]
        },
        "office room table 63": {
            "frame_id": "map",
            "position": [office_room_table_63_x + table_x_diff, office_room_table_63_y, 0],
            "orientation": [0, 0, 1, 0]
        },
        "office room table 238": {
            "frame_id": "map",
            "position": [office_room_table_238_x - table_x_diff, office_room_table_238_y, 0],
            "orientation": [0, 0, 0, 1]
        },
        "stock room table 333": {
            "frame_id": "map",
            "position": [stock_room_table_333_x + table_x_diff, stock_room_table_333_y, 0],
            "orientation": [0, 0, 1, 0]
        },
        "orange pose": {
            "frame_id": "odom",
            "position": orange_pos,
            "orientation": orange_quat
        },
        "prepick tool pose": {
            "frame_id": "odom",
            "position": prepick_tool_pos,
            "orientation": prepick_tool_quat
        }
    }

    add_table("table", [1, 0, 0.4], [0, 0, 1.57])
    add_box("box")

    create_tiago_services()

# orange_pose_x = -3.6
# orange_pose_y = 7.8
# orange_pose_z = 0.825

# pre_pick_diff_x = 0.2
# pre_pick_diff_y = -0.035
# pre_pick_diff_z = 0.2

# orange_tool_pose_x = orange_pose_x + pre_pick_diff_x
# orange_tool_pose_y = orange_pose_y + pre_pick_diff_y
# orange_tool_pose_z = orange_pose_z + pre_pick_diff_z

# tucked arm in front of stock room table:
# position: -3.1437; 7.7771; 0.54792
# orientation: 0.4855; 0.53661; 0.54734; -0.42043

# orange_pose_pos_x = -3.800000
# orange_pose_pos_y = 8.000018
# orange_pose_pos_z = 0.839077

# orange_prepick_pose_x = orange_pose_pos_x
# orange_prepick_pose_y = orange_pose_pos_y
# orange_prepick_pose_z = orange_pose_pos_z + 0.2

# gripper_center_pos_x = orange_pose_pos_x
# gripper_center_pos_y = orange_pose_pos_y
# gripper_center_pos_z = orange_pose_pos_z + 0.2

# arm_left_tool_pos_x =
# arm_left_tool_pos_y =
# arm_left_tool_pos_z =

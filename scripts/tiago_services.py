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
from tf.transformations import quaternion_from_euler

from utils import get_object_above_pose, get_matrix_from_pose, get_pos_and_quat_from_matrix, center_to_tool, pose_by_diff


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
    pose_pos, pose_quat = get_pos_and_quat_from_matrix(pose["pose"])

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = pose["frame_id"]
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pose_pos[0]
    goal.target_pose.pose.position.y = pose_pos[1]
    goal.target_pose.pose.orientation.x = pose_quat[0]
    goal.target_pose.pose.orientation.y = pose_quat[1]
    goal.target_pose.pose.orientation.z = pose_quat[2]
    goal.target_pose.pose.orientation.w = pose_quat[3]

    client.send_goal(goal)
    client.wait_for_result()
    return MovePoseResponse(1)


def move_arm_to_pose(srv_request):
    arm_pose = pose_dict[srv_request.pose_str]
    pose_pos, pose_quat = get_pos_and_quat_from_matrix(arm_pose["pose"])

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = arm_pose["frame_id"]
    pose_goal.pose.position.x = pose_pos[0]
    pose_goal.pose.position.y = pose_pos[1]
    pose_goal.pose.position.z = pose_pos[2]
    pose_goal.pose.orientation.x = pose_quat[0]
    pose_goal.pose.orientation.y = pose_quat[1]
    pose_goal.pose.orientation.z = pose_quat[2]
    pose_goal.pose.orientation.w = pose_quat[3]
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


def add_object(object_name, object_pos, object_euler, object_size, timeout=4):
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

    scene.add_box(object_name, object_pose, size=object_size)
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

    id_quat = np.array([0, 0, 0, 1])
    table_x_diff = 1.15
    prepick_diff = np.array([0, 0, 0.25])
    preplace_diff = np.array([0, 0, 0.25])
    pick_diff = np.array([0, 0, 0.17])
    holding_diff = np.array([-0.3, 0, 0])
    place_diff = np.array([0, 0, 0.14])
    gripper_center_to_tool_pos = np.array([-0.201, 0, 0])
    gripper_center_to_tool_quat = np.array([-0.707, -0.000, -0.000, 0.707])
    center_to_tool_transform = get_matrix_from_pose(gripper_center_to_tool_pos, gripper_center_to_tool_quat)

    box_pos, box_quat, box_size = np.array([-3.95, 8, 0.865]), np.array([0, 0, 0, 1]), np.array([0.05, 0.05, 0.1])
    stk111_pos, stk111_quat, stk111_size = np.array([4, -3, 0.865]), np.array([0, 0, 0, 1]), np.array([0.05, 0.05, 0.1])
    stk238_pos, stk238_quat, stk238_size = np.array([4, 3, 0.865]), np.array([0, 0, 0, 1]), np.array([0.05, 0.05, 0.1])

    # stock room
    sr_table_333_pos = np.array([-4.2, 8, 0.4])
    sr_table_333_quat = quaternion_from_euler(0, 0, 0)
    sr_table_333_size = np.array([0.8 + 0.1, 1 + 1, 0.815 + 0.05])

    # office room
    # front and left
    of_table_238_pos = np.array([4.2, 3, 0.4])
    of_table_238_quat = quaternion_from_euler(0, 0, 0)
    of_table_238_size = np.array([0.8 + 0.1, 1 + 1, 0.815 + 0.05])

    # behind and left
    of_table_63_pos = np.array([-4.2, 3, 0.4])
    of_table_63_quat = quaternion_from_euler(0, 0, 0)
    of_table_63_size = np.array([0.8 + 0.1, 1 + 1, 0.815 + 0.05])

    # front and right
    of_table_111_pos = np.array([4.2, -3, 0.4])
    of_table_111_quat = quaternion_from_euler(0, 0, 0)
    of_table_111_size = np.array([0.8 + 0.2, 1 + 1, 0.815 + 0.03])

    # behind and right
    of_table_26_pos = np.array([-4.2, -3, 0.4])
    of_table_26_quat = quaternion_from_euler(0, 0, 0)
    of_table_26_size = np.array([0.8 + 0.1, 1 + 1, 0.815 + 0.05])

    # for navigation
    of_table_111_front_pos = np.array([-4.2 + table_x_diff, 3, 0])
    of_table_111_front_quat = np.array([0, 0, 1, 0])

    of_table_238_front_pos = np.array([4.2 - table_x_diff, 3, 0])
    of_table_238_front_quat = np.array([0, 0, 0, 1])

    sr_table_333_front_pos = np.array([-4.2 + table_x_diff, 8, 0])
    sr_table_333_front_quat = np.array([0, 0, 1, 0])

    add_object("box", box_pos, box_quat, box_size)
    add_object("of_table_111", of_table_111_pos, of_table_111_quat, of_table_111_size)
    add_object("of_table_238", of_table_238_pos, of_table_238_quat, of_table_238_size)
    add_object("of_table_26", of_table_26_pos, of_table_26_quat, of_table_26_size)
    add_object("of_table_63", of_table_63_pos, of_table_63_quat, of_table_63_size)
    add_object("sr_table_333", sr_table_333_pos, sr_table_333_quat, sr_table_333_size)

    box_pose = get_matrix_from_pose(box_pos, box_quat)
    above_box_pose = get_object_above_pose(listener, box_pose, prepick_diff)
    above_box_tool_pose = center_to_tool(above_box_pose, center_to_tool_transform)
    pick_box_tool_pose = pose_by_diff(above_box_tool_pose, pick_diff, id_quat)
    holding_pose = pose_by_diff(above_box_tool_pose, holding_diff, id_quat)

    # behind and left
    stk111_pose = get_matrix_from_pose(stk111_pos, stk111_quat)
    above_111_pose = get_object_above_pose(listener, stk111_pose, preplace_diff)
    above_111_tool_pose = center_to_tool(above_111_pose, center_to_tool_transform)
    place_111_tool_pose = pose_by_diff(above_111_tool_pose, place_diff, id_quat)

    # front and left
    stk111_pose = get_matrix_from_pose(stk111_pos, stk111_quat)
    above_111_pose = get_object_above_pose(listener, stk111_pose, preplace_diff)
    above_111_tool_pose = center_to_tool(above_111_pose, center_to_tool_transform)
    place_111_tool_pose = pose_by_diff(above_111_tool_pose, place_diff, id_quat)

    pose_dict = {
        "stock room table 333 front": {
            "frame_id": "odom",
            "position": sr_table_333_front_pos,
            "orientation": sr_table_333_front_quat
        },
        "office room table 111 front": {
            "frame_id": "odom",
            "position": of_table_111_front_pos,
            "orientation": of_table_111_front_quat
        },
        "office room table 238": {
            "frame_id": "map",
            "position": of_table_238_front_pos,
            "orientation": of_table_238_front_quat
        },
        "office room table 26 front": {
            "frame_id": "map",
            "position": of_table_26_pos,
            "orientation": of_table_26_quat
        },
        "office room table 63": {
            "frame_id": "map",
            "position": of_table_63_pos,
            "orientation": of_table_63_quat
        },
        "orange pose": {
            "frame_id": "odom",
            "position": box_pos,
            "orientation": box_quat
        },
        "above box tool pose": {
            "frame_id": "odom",
            "pose": above_box_tool_pose,
        },
        "pick tool pose": {
            "frame_id": "odom",
            "pose": pick_box_tool_pose,
        },
        "above 111 tool pose": {
            "frame_id": "odom",
            "pose": above_111_tool_pose,
        },
        "place 111 tool pose": {
            "frame_id": "odom",
            "pose": place_111_tool_pose,
        },
        "holding pose": {
            "frame_id": "odom",
            "pose": holding_pose,
        },
    }

    create_tiago_services()

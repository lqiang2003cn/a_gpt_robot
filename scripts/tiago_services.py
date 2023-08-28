#!/usr/bin/env python

import sys

import actionlib
from geometry_msgs.msg import PoseStamped
import moveit_commander
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from a_gpt_robot.srv import MovePose, MovePoseResponse
from std_srvs.srv import Trigger, Empty
from tf.transformations import quaternion_from_euler

office_room_table_111_x = 4.2
office_room_table_111_y = -3

office_room_table_26_x = -4.2
office_room_table_26_y = -3

office_room_table_63_x = -4.2
office_room_table_63_y = 3

office_room_table_238_x = 4.2
office_room_table_238_y = 3

table_x_diff = 1.1

pose_dict = {
    "stock room table 111": {
        "frame_id": "map",
        "position": [office_room_table_111_x - table_x_diff, office_room_table_111_y, 0],
        "orientation": [0, 0, 0, 1]
    },
    "stock room table 26": {
        "frame_id": "map",
        "position": [office_room_table_26_x + table_x_diff, office_room_table_26_y, 0],
        "orientation": [0, 0, 1, 0]
    },
    "stock room table 63": {
        "frame_id": "map",
        "position": [office_room_table_63_x + table_x_diff, office_room_table_63_y, 0],
        "orientation": [0, 0, 1, 0]
    },
    "stock room table 238": {
        "frame_id": "map",
        "position": [office_room_table_238_x - table_x_diff, office_room_table_238_y, 0],
        "orientation": [0, 0, 0, 1]
    },
    "orange pose": {
        "frame_id": "map",
        "position": [0.4, 0.3, 0.3],
        "orientation": quaternion_from_euler(1.57, 0, 0)
    },
}


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


if __name__ == "__main__":
    rospy.init_node('create_tiago_services')
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander("arm_left_torso")
    gripper_group = moveit_commander.MoveGroupCommander("gripper_left")
    create_tiago_services()

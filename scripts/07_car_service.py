#!/usr/bin/env python

from __future__ import print_function

import rospy
from arm_moveit_demo.srv import Position, PositionResponse
from yahboomcar_msgs.msg import ArmJoint
from time import sleep
from yahboomcar_msgs.srv import RobotArmArray, RobotArmArrayRequest
from std_srvs.srv import Trigger, TriggerResponse

position_dict = {
    "initial position": [90, 145, 0, 45, 90],

    "front higher position": [90, 110, 75, 0, 90],
    "right higher position": [0, 110, 75, 0, 90],
    "left higher position": [180, 110, 75, 0, 90],

    "front lower position": [90, 150, 0, 36, 90],
    "right lower position": [0, 150, 0, 36, 90],
    "left lower position": [180, 150, 0, 36, 90],

}


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


def handle_go_to_position(position_msg):
    print("Going to position:", position_msg.position)
    try:
        pos_joints = position_dict[position_msg.position]
        curr_joints = get_current_angles()
        curr_joints[0:5] = pos_joints
        publish_joints(curr_joints)
    except KeyError as e:
        print("the position does not exists:", e)
        return PositionResponse(0)

    return PositionResponse(1)


def handle_close_gripper(trigger):
    assert trigger is not None
    print("Closing Gripper")
    try:
        curr_joints = get_current_angles()
        curr_joints[5] = 160
        publish_joints(curr_joints)
    except KeyError as e:
        print("the position does not exists:", e)
        return PositionResponse(0)

    return TriggerResponse()


def handle_open_gripper(trigger):
    assert trigger is not None
    print("Opening Gripper")
    try:
        curr_joints = get_current_angles()
        curr_joints[5] = 30
        publish_joints(curr_joints)
    except KeyError as e:
        print("the position does not exists:", e)
        return PositionResponse(0)

    return TriggerResponse()


def create_go_to_position_service():
    rospy.init_node('create_go_to_position_service')
    rospy.Service('go_to_position', Position, handle_go_to_position)
    rospy.Service('close_gripper', Trigger, handle_close_gripper)
    rospy.Service('open_gripper', Trigger, handle_open_gripper)
    print("Car services are ready")
    rospy.spin()


if __name__ == "__main__":
    create_go_to_position_service()

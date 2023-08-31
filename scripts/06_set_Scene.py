#!/usr/bin/env python
# -*- coding: utf-8 -*-
from math import pi

# import openai
import rospy, rospkg

from time import sleep
# import moveit_commander
# from moveit_msgs.msg import PlanningSceneWorld
# from yahboomcar_msgs.msg import *

# from geometry_msgs.msg import PoseStamped
# from moveit_commander import MoveGroupCommander, PlanningSceneInterface, PlanningScene, PlannerInterfaceDescription
# from sensor_msgs.msg import JointState

# def pubArm(joints, id=10, angle=90, run_time=500):
#     pub_Arm = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1000)
#     armjoint = ArmJoint()
#     armjoint.run_time = run_time
#     if len(joints) != 0:
#         armjoint.joints = joints
#     else:
#         armjoint.id = id
#         armjoint.angle = angle
#         for i in range(10):
#         pub_Arm.publish(arm_joint)
#         sleep(0.1)
#     pub_Arm.publish(armjoint)


if __name__ == "__main__":
    print "i"
    # rospy.init_node('Set_Scene')
    # import requests
    #
    # key = "sk-hreYr15pupx3kkOd3R8X8D7Qt0V88Mni2XvuNM8awFkg0ruU"
    # headers = {"Authorization": f"Bearer {key}"}
    # data = {
    #     'model': 'gpt-3.5-turbo',
    #     'messages': [{"role": "user", "content": "hello!"}]
    # }
    # print(requests.post("https://api.chatanywhere.com.cn/v1/chat/completions", headers=headers, json=data).json())

    # try:
    #     openai.api_key = "sk-hreYr15pupx3kkOd3R8X8D7Qt0V88Mni2XvuNM8awFkg0ruU"
    #     openai.api_base = "https://api.chatanywhere.com.cn"
    #     response = openai.ChatCompletion.create(
    #         model="gpt-3.5-turbo",
    #         messages=[{"role": "user", "content": "hello world"}]
    #     )
    #     print(response)
    # except Exception as e:
    #     print("Oops! Something went wrong with {e}.", e)

    # rospy.init_node('Set_Scene')
    # pub_Arm = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1000)
    # arm_joint = ArmJoint()
    # arm_joint.run_time = 5000

    # joint1: [0,180]: 0: right most, 180: left most, 90: front
    # joint2: [0,180]: 0: back most, 180: forward most, 90: up
    # joint3: [0,180]: 0: back most, 180: forward most, 90: up
    # joint4: [0,180]: 0: back most, 180: forward most, 90: up
    # joint5: [0,180]: 0: left most, 180: right most, 90: front
    # joint6: close=180, open=0

    # inspect position:
    # arm_joint.joints = [90, 120, 0, 0, 90, 150]

    # pick position:
    # arm_joint.joints = [90, 150, 0, 36, 90, 0]

    # front pick:
    # arm_joint.joints = [90, 150, 0, 36, 90, 150]

    # front high position:
    # arm_joint.joints = [90, 110, 75, 0, 90, 150]

    # right high position:(facing people's left)
    # arm_joint.joints = [0, 110, 75, 0, 90, 150]

    # right place position:
    # arm_joint.joints = [0, 150, 0, 36, 90, 150]



    # for i in range(10):
    #     pub_Arm.publish(arm_joint)
    #     sleep(0.1)

    # arm_joint.id = 1
    # arm_joint.angle = 90

    # arm_joint.id = 1
    # arm_joint.angle = 10
    # for i in range(10):
    #     pub_Arm.publish(arm_joint)
    #     sleep(0.1)

# def add_obj(table_pose, obj, table_size, xyz):
#     table_pose.header.frame_id = 'base_footprint'
#     table_pose.pose.position.x = xyz[0]
#     table_pose.pose.position.y = xyz[1]
#     table_pose.pose.position.z = xyz[2]
#     table_pose.pose.orientation.w = 1.0
#     scene.add_box(obj, table_pose, table_size)
#
#
#
# if __name__ == "__main__":
#     moveit_commander.roscpp_initialize(sys.argv)
#     rospy.init_node('Set_Scene')
#     # 仿真
#     pub_joint = rospy.Publisher("/move_group/fake_controller_joint_states", JointState, queue_size=1000)
#     # 真机
#     pub_Arm = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1000)
#     arm_joint = ArmJoint()
#     arm_joint.id = 6
#     arm_joint.angle = 180 - 0.55 * 180 / pi
#     joint_state = JointState()
#     joint_state.name = ["grip_joint"]
#     joint_state.position = [-0.58]
#     for i in range(10):
#         pub_joint.publish(joint_state)
#         pub_Arm.publish(arm_joint)
#         sleep(0.1)
#     yahboomcar = MoveGroupCommander('arm_group')
#     end_effector_link = yahboomcar.get_end_effector_link()
#     scene = PlanningSceneInterface()
#     scene.remove_attached_object(end_effector_link, "tool")
#     scene.remove_world_object()
#     yahboomcar.allow_replanning(True)
#     yahboomcar.set_planning_time(1)
#     yahboomcar.set_num_planning_attempts(10)
#     yahboomcar.set_goal_position_tolerance(0.01)
#     yahboomcar.set_goal_orientation_tolerance(0.01)
#     yahboomcar.set_goal_tolerance(0.01)
#     yahboomcar.set_max_velocity_scaling_factor(1.0)
#     yahboomcar.set_max_acceleration_scaling_factor(1.0)
#     rospy.loginfo("Set Init Pose")
#     yahboomcar.set_named_target("init")
#     yahboomcar.go()
#     p = PoseStamped()
#     p.header.frame_id = end_effector_link
#     p.pose.orientation.w = 1
#     # 添加tool
#     scene.attach_box(end_effector_link, 'tool', p, [0.03, 0.03, 0.03])
#     target_joints1 = [0, -1.18, -1.17, 0.77, 0]
#     target_joints2 = [0, -1.21, 0.52, -0.89, 0]
#     table_list = {
#         "obj0": [[0.08, 0.01, 0.4], [0.4, -0.1, 0.2]],
#         "obj1": [[0.08, 0.01, 0.4], [0.4, 0.1, 0.2]],
#         "obj2": [[0.08, 0.22, 0.01], [0.4, 0, 0.4]],
#         "obj3": [[0.08, 0.22, 0.01], [0.4, 0, 0.29]],
#         "obj4": [[0.08, 0.22, 0.01], [0.4, 0, 0.17]],
#     }
#     # 添加obj
#     for i in range(len(table_list)):
#         add_obj(p, table_list.keys()[i], table_list[table_list.keys()[i]][0],
#                 table_list[table_list.keys()[i]][1])
#     rospy.loginfo("Grip Target")
#     i = 0
#     while i < 5:
#         yahboomcar.set_joint_value_target(target_joints1)
#         yahboomcar.go()
#         yahboomcar.set_joint_value_target(target_joints2)
#         yahboomcar.go()
#         i += 1
#         print ("第 {} 次规划!!!".format(i))
#     scene.remove_attached_object(end_effector_link, 'tool')
#     scene.remove_world_object()
#     moveit_commander.roscpp_shutdown()
#     moveit_commander.os._exit(0)

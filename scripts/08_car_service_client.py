#!/usr/bin/python3

from __future__ import print_function

import json
import os

import requests
import roslibpy
import rospy

ros_client = roslibpy.Ros(host="localhost", port=9090)
ros_client.run()
services = {}


def car_service_client(sn, args):
    rospy.wait_for_service(sn)
    try:
        if sn not in services:
            services[sn] = roslibpy.Service(ros_client, sn, ros_client.get_service_type(sn))
        req = roslibpy.ServiceRequest(args)
        resp = services[sn].call(req)
        rospy.sleep(1.5)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def call_gpt(in_json):
    api_key = "sk-hreYr15pupx3kkOd3R8X8D7Qt0V88Mni2XvuNM8awFkg0ruU"
    api_base = "https://api.chatanywhere.com.cn/v1/chat/completions"
    headers = {"Authorization": "Bearer " + api_key}
    # fp_prompt = os.path.join("~/yahboomcar_ws/src/arm_moveit_demo/scripts/", "car_prompt.txt")
    fp_prompt = os.path.join("", "car_prompt.txt")
    with open(fp_prompt) as f:
        sys_prompt = f.read()
    data = {
        'model': 'gpt-3.5-turbo',
        'messages': [
            {"role": "system", "content": sys_prompt},
            {"role": "user", "content": str(in_json)}
        ]
    }
    resp_msg = requests.post(api_base, headers=headers, json=data).json()['choices'][0]['message']['content']
    json_msg = json.loads(resp_msg, strict=False)
    task_sequence = json_msg['task_cohesion']['task_sequence']
    for t in task_sequence:
        car_service_client(t['action_name'], t['action_param'])


if __name__ == "__main__":
    input_json = {
        "end_effector_pos": "initial position",
        "gripper_status": "open",
        "object_pos": "left lower position",
        "instruction": "pick up the object and place it to the right lower position."
    }
    call_gpt(input_json)

    # curr_angle = rospy.ServiceProxy("CurrentAngle", RobotArmArray)
    # request = RobotArmArrayRequest()
    # response = curr_angle.call(request)
    # print(response)

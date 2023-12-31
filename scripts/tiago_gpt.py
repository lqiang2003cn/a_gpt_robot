#!/usr/bin/env python


import json
import os
import time

import requests
import roslibpy
import rospy

# ros_client = roslibpy.Ros(host="localhost", port=9090)
# ros_client.run()
# services = {}
#
#
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


def call_gpt(in_json):
    headers = {"Authorization": "Bearer " + api_key}
    fp_prompt = os.path.join("", "tiago_prompt_pickplace.txt")
    with open(fp_prompt) as f:
        sys_prompt = f.read()
    data = {
        'model': 'gpt-3.5-turbo-16k',
        'temperature': 0.0,
        'messages': [
            {"role": "system", "content": sys_prompt},
            {"role": "user", "content": str(in_json)}
        ]
    }
    response_str = requests.post(api_base, headers=headers, json=data)
    print response_str.text
    resp_msg = response_str.json()['choices'][0]['message']['content']
    # print resp_msg
    json_msg = json.loads(resp_msg, strict=True)
    time_str = time.strftime("%Y%m%d-%H%M%S")
    json_file_name = "../json_files/result_" + time_str + ".json"
    with open(json_file_name, 'w') as fp:
        json.dump(json_msg, fp)
    print "c"

    # task_sequence = json_msg['task_cohesion']['task_sequence']
    # for t in task_sequence:
    #     call_service_func(t['action_name'], t['action_param'])


if __name__ == "__main__":
    # input_json = {
    #     "environment": {
    #         "objects": [
    #             "water",
    #             "juice",
    #             "cookie",
    #             "bread"
    #         ],
    #         "object_locations": [
    #             "table1",
    #             "table2",
    #             "table3",
    #             "table4"
    #         ],
    #         "commanders": [
    #             "Alice",
    #             "Bob",
    #             "Carl"
    #         ],
    #         "commander_locations": [
    #             "table5",
    #             "table6",
    #             "table7"
    #         ]
    #     },
    #     "command:": "I am Carl. I want some rice"
    # }

    fp_prompt = os.path.join("", "tiago_prompt_sit_user.txt")
    with open(fp_prompt) as f:
        user_prompt = f.read()
    call_gpt(user_prompt)

    # input_json = {
    #     "environment": {
    #         "assets": [
    #             "<table_333>",
    #             "<table_238>"
    #         ],
    #         "objects": [
    #             "<box_123>"
    #         ],
    #         "object_states": {
    #             "<box_123>": ["ON(<table_333>)"]
    #         }
    #     },
    #     "instruction:": "Take the box 123 to table 238"
    # }

    # '''
    # "environment": {
    #     "objects": [
    #         "water",
    #         "juice",
    #         "cookie",
    #         "bread",
    #
    #     ],
    #     "object_locations": [
    #         "table1",
    #         "table2",
    #         "table3",
    #         "table4",
    #     ],
    #     "commanders": [
    #         "Alice",
    #         "Bob",
    #         "Carl",
    #     ],
    #     "commander_locations": [
    #         "table5",
    #         "table6",
    #         "table7",
    #     ],
    # }
    # '''



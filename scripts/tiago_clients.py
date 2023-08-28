from __future__ import print_function

import roslibpy
import rospy

ros_client = roslibpy.Ros(host="localhost", port=9090)
ros_client.run()
services = {}


def call_service_func(sn, args):
    rospy.wait_for_service(sn)
    try:
        if sn not in services:
            services[sn] = roslibpy.Service(ros_client, sn, ros_client.get_service_type(sn))
        req = roslibpy.ServiceRequest(args)
        resp = services[sn].call(req)
        rospy.sleep(2)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    arg = {
        "pose_str": "stock room table1"
    }
    call_service_func("move_to_pose", arg)

#!/usr/bin/env python
import json

import numpy as np
import requests
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import utils


class NumpyEncoder(json.JSONEncoder):
    """ Special json encoder for numpy types """

    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


class Nodo(object):
    def __init__(self):
        # Params
        self.image = None
        self.texts = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        # Publishers
        self.pub = rospy.Publisher('imagetimer', Image, queue_size=10)

        # Subscribers
        rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

    def callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg)

    def request_owl_vit(self):
        pj = {
            "prompt": "hello"
        }
        response = utils.post_json_no_proxy("prompt_len", pj)
        # response_json = requests.post(api_base, headers={"Content-Type": "application/json"}, json=json)
        print "c"

        # api = 'http://192.168.50.96:8080/owl_vit'
        # headers = {'Content-type': 'application/json'}
        # payload = {"image": json.dumps(self.image, cls=NumpyEncoder), "texts": self.texts}
        # response = requests.post(api, json=payload, headers=headers)
        # try:
        #     data = response.json()
        #     print(data)
        # except requests.exceptions.RequestException:
        #     print(response.text)

    def start(self):
        rospy.loginfo("Timing images")
        # rospy.spin()
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image')
            # br = CvBridge()
            if self.image is not None:
                self.pub.publish(self.br.cv2_to_imgmsg(self.image))
            self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.init_node("image_vis", anonymous=True)
    my_node = Nodo()
    my_node.texts = ["red block"]
    my_node.request_owl_vit()

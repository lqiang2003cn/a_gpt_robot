#!/usr/bin/env python
import json

import numpy as np
import requests
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import utils


def default(obj):
    if type(obj).__module__ == np.__name__:
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        else:
            return obj.item()
    raise TypeError('Unknown type:', type(obj))


class Nodo(object):
    def __init__(self):
        # Params
        self.image = None
        self.texts = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1.0)

        # Publishers
        self.pub = rospy.Publisher('imagetimer', Image, queue_size=10)

        # Subscribers
        rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

    def callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg)

    def request_owl_vit(self):
        while self.image is None:
            print "waiting for image"
            self.loop_rate.sleep()

        json_data = {
            "image": self.image.tolist(),
            "texts": self.texts
        }
        response = utils.post_json_no_proxy("owl_vit", json_data)
        try:
            data = response.json()
            print(data)
        except requests.exceptions.RequestException:
            print(response.text)

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

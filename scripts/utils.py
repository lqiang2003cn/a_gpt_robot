#!/usr/bin/env python

import numpy as np
import rospy
import tf


def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def query_pose(listener, target_frame, source_frame):
    pos, rot = None, None
    while not rospy.is_shutdown():
        try:
            pos, rot = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    return pos, rot

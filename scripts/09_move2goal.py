#!/usr/bin/env python

from __future__ import print_function

from copy import copy
from math import pow, sqrt, atan2

import rospy

from geometry_msgs.msg import Twist
import tf
from tf.transformations import euler_from_quaternion

def euclidean_distance(goal_pos, curr_pos):
    """Euclidean distance between current pose and the goal."""
    return sqrt(pow((goal_pos.x - curr_pos.x), 2) + pow((goal_pos.y - curr_pos.y), 2))


class Pose:

    def __init__(self):
        self.x = None
        self.y = None
        self.theta = None


def get_curr_pose(listener):
    curr_pose = Pose()
    while not rospy.is_shutdown():
        try:
            curr_position_coord, rot = listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
            curr_pose.x = curr_position_coord[0]
            curr_pose.y = curr_position_coord[1]
            euler = euler_from_quaternion(rot)
            print("rotation:", rot)
            print("current position:", curr_position_coord)
            print("euler:", euler)
            curr_pose.theta = euler[2]
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    return curr_pose


def linear_vel(goal_pose, curr_pos, constant=1.5):
    """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
    x_vel = constant * euclidean_distance(goal_pose, curr_pos)
    # if x_vel > 0.01:
    #     x_vel = 0.01
    # elif x_vel < -0.01:
    #     x_vel = -0.01
    return x_vel


def steering_angle(goal_pose, curr_pose):
    """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
    return atan2(goal_pose.y - curr_pose.y, goal_pose.x - curr_pose.x)


def angular_vel(goal_pose,  curr_pose, constant=6):
    """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
    return constant * (steering_angle(goal_pose, curr_pose) - curr_pose.theta)


if __name__ == "__main__":
    rospy.init_node('move_node')
    rate = rospy.Rate(10)
    listener = tf.TransformListener()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    distance_tolerance = 0.01
    curr_pose = get_curr_pose(listener)
    goal_pose = copy(curr_pose)
    goal_pose.x += 0.2
    # goal_position.x -= 0.2
    dist = euclidean_distance(goal_pose, curr_pose)
    while dist >= distance_tolerance:
        vel_msg = Twist()
        vel_msg.linear.x = linear_vel(goal_pose, curr_pose)
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = angular_vel(goal_pose, curr_pose)
        pub.publish(vel_msg)

        curr_pose = get_curr_pose(listener)
        dist = euclidean_distance(goal_pose, curr_pose)
        print("dist is:", dist)
        print("goal x is:", goal_pose.x)
        rate.sleep()

    zero_msg = Twist()
    zero_msg.linear.x = 0
    zero_msg.linear.y = 0
    zero_msg.linear.z = 0

    # Angular velocity in the z-axis.
    zero_msg.angular.x = 0
    zero_msg.angular.y = 0
    zero_msg.angular.z = 0
    pub.publish(zero_msg)

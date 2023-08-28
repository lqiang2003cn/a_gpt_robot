import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

pose_dict = {
    "stock room table1": {
        "position": [1, 1, 1],
        "orientation": [0, 0, 0, 1]
    }
}


def move_to_pose(pose_str):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    pose = pose_dict[pose_str]

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pose["position"][0]
    goal.target_pose.pose.position.y = pose["position"][1]
    goal.target_pose.pose.orientation.x = pose["orientation"][0]
    goal.target_pose.pose.orientation.y = pose["orientation"][1]
    goal.target_pose.pose.orientation.z = pose["orientation"][2]
    goal.target_pose.pose.orientation.w = pose["orientation"][3]

    client.send_goal(goal)
    client.wait_for_result()


def create_tiago_services():
    rospy.init_node('create_tiago_services')
    rospy.Service('move_to_pose', Empty, move_to_pose)
    print "tiago services are ready"
    rospy.spin()


if __name__ == "__main__":
    create_tiago_services()

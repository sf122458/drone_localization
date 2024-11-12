#! /home/ps/.conda/envs/hloc/bin/python

import sys
sys.path.append("/home/ps/catkin_ws/src/hloc/scripts")

from controller import DroneController
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped

pose = PoseStamped()
def realtime_pos_cb(msg):
    global pose
    pose = msg
    print(pose.pose.position.x)
    print(pose.pose.position.y)
    print(pose.pose.position.z)
    print(pose.pose.orientation.x)
    print(pose.pose.orientation.y)
    print(pose.pose.orientation.z)
    print(pose.pose.orientation.w)
    

if __name__ == "__main__":

    rospy.init_node("realtime_pos")

    realtime_pos_sub = rospy.Subscriber("drone_pos", PoseStamped, realtime_pos_cb)

    rospy.spin()
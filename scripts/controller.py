#! /home/ps/.conda/envs/hloc/bin/python

import os
import sys

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
# from pose_utils import PoseCalibrator, hlocBuilder
from cv_bridge import CvBridge
import numpy as np
import cv2
from copy import deepcopy

from ros_utils import dist, unwrap_pose

class DroneController(object):
    def __init__(self, rate=20):

        self.rate = rospy.Rate(rate)

        self.current_state = State()
        self.local_pos = PoseStamped()
        self.target_pos = PoseStamped()

        # 订阅无人机位置与姿态xyz, q
        self.local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.local_pos_cb)

        # 发布无人机位置与姿态xyz, q
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        self.state_sub = rospy.Subscriber("mavros/state", State, self.state_cb)

        rospy.wait_for_service("mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # callback functions
    def state_cb(self, msg):
        self.current_state = msg

    def local_pos_cb(self, msg):
        self.local_pos = msg

    # basic cmd
    def init_stream(self, rate, n=100):
        for _ in range(n):
            if(rospy.is_shutdown()):
                break
            pose = PoseStamped()
            self.local_pos_pub.publish(pose)
            rate.sleep()

    def arm(self):
        while not self.current_state.armed:
            self.arming_client(True)
        print("Vehicle Armed")
            

    def disarm(self):
        while self.current_state.armed:
            self.arming_client(False)
        print("Vehicle Disarmed")

    def land(self):
        rospy.loginfo("Landing")
        self.set_mode("AUTO.LAND")


    def set_mode(self, mode):
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = mode
        result = self.set_mode_client.call(offb_set_mode)
        if result.mode_sent:
            rospy.loginfo("Setting mode to %s successful", mode)
        else:
            rospy.loginfo("Setting mode to %s unsuccessful", mode)

    def set_target_pos(self, x=None, y=None, z=None):
        self.target_pos = deepcopy(self.local_pos)
        if x != None:
            self.target_pos.pose.position.x = x
        if y != None:
            self.target_pos.pose.position.y = y
        if z != None:
            self.target_pos.pose.position.z = z

    def publish_target_pos(self):
        self.local_pos_pub.publish(self.target_pos)

    def set_target_pos_and_exec(self, x=None, y=None, z=None):
        self.set_target_pos(x, y, z)
        self.publish_target_pos()
        

    def takeoff(self, z=5):
        rospy.loginfo("Connecting to Autopilot")
        while not self.current_state.connected:
            self.rate.sleep()

        self.arm()

        # NOTE: enter OFFBOARD mode
        self.init_stream(self.rate)
        
        rospy.loginfo("Taking off")

        self.set_target_pos_and_exec(z=z)

        while dist(unwrap_pose(self.local_pos)[0], unwrap_pose(self.target_pos)[0]) > 0.1:
            if self.current_state.mode != "OFFBOARD":
                self.set_mode("OFFBOARD")

            self.publish_target_pos()
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("mavros_drone_test")

    drone = DroneController()

    drone.takeoff(5)
    drone.land()

    rospy.spin()
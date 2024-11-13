#! /home/ps/.conda/envs/hloc/bin/python

import rospy
from controller import BasicDroneController
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import keyboard
from cv_bridge import CvBridge
import cv2
import os

import numpy as np
from scipy.spatial.transform import Rotation as R

class Logwriter:
    def __init__(self, path):
        self.path = path

    def write(self, msg):
        with open(self.path, 'a') as f:
            f.write(msg)

"""
keyboard example:
    q:      +0.5m in z direction
    e:      -0.5m 0.5 in z direction
    w:      +0.5m in x direction
    s:      -0.5m in x direction
    a:      +0.5m in y direction
    d:      -0.5m in y direction
    i:      +10 degree in pitch
    k:      -10 degree in pitch
    j:      -10 degree in yaw
    l:      +10 degree in yaw
    esc:    land
    t:      takeoff
    space:  save image from camera
    c:      print the settings of position and orientation
"""

class KeyboardController(BasicDroneController):
    def __init__(self, image_path, log_path):
        super().__init__()

        self.image_path = image_path
        self.bridge = CvBridge()
        self.idx = 0
        self.yaw = 0
        self.pitch = 0

    def main(self):
        # keyboard.on_release(self.key_callback)
        keyboard.on_press(self.key_callback)    # keep pressing will trigger the callback multiple times
        last_req = rospy.Time.now()
        while not rospy.is_shutdown():

            if rospy.Time.now() - last_req > rospy.Duration(5.0):
                last_req = rospy.Time.now()
                print(f"Position: x:{self.local_pos.pose.position.x}, y:{self.local_pos.pose.position.y}, z:{self.local_pos.pose.position.z}")
                yaw, pitch, roll = R.from_quat([self.local_pos.pose.orientation.x, 
                                                self.local_pos.pose.orientation.y, 
                                                self.local_pos.pose.orientation.z, 
                                                self.local_pos.pose.orientation.w]).as_euler('ZYX')
                print(f"Orientation: yaw:{yaw}, pitch:{pitch}, roll:{roll}")

            self.local_pos_pub.publish(self.target_pos)
            self.rate.sleep()


    def key_callback(self, event):
        distance = 0.5
        if event.name == 'q':
            self.target_pos.pose.position.z = self.target_pos.pose.position.z + distance
            print(f'+{distance} in z direction')
        elif event.name == 'e':
            self.target_pos.pose.position.z = self.target_pos.pose.position.z - distance
            print(f'-{distance} in z direction')
        elif event.name == 'w':
            self.target_pos.pose.position.x = self.target_pos.pose.position.x + distance
            print(f'+{distance} in x direction')
        elif event.name == 's':
            self.target_pos.pose.position.x = self.target_pos.pose.position.x - distance
            print(f'-{distance} in x direction')
        elif event.name == 'a':
            self.target_pos.pose.position.y = self.target_pos.pose.position.y + distance
            print(f'+{distance} in y direction')
        elif event.name == 'd':
            self.target_pos.pose.position.y = self.target_pos.pose.position.y - distance
            print(f'-{distance} in y direction')   
        elif event.name == 'i':
            self.pitch = (self.pitch + np.pi / 18) % (2 * np.pi)
            x, y, z, w = R.from_euler('ZYX', [self.yaw, self.pitch, 0]).as_quat()
            self.target_pos.pose.orientation.x = x
            self.target_pos.pose.orientation.y = y
            self.target_pos.pose.orientation.z = z
            self.target_pos.pose.orientation.w = w
            print(f"Set the pitch to {int(self.pitch / np.pi * 180)} degree")
        elif event.name == 'k':
            self.pitch = (self.pitch - np.pi / 18) % (2 * np.pi)
            x, y, z, w = R.from_euler('ZYX', [self.yaw, self.pitch, 0]).as_quat()
            self.target_pos.pose.orientation.x = x
            self.target_pos.pose.orientation.y = y
            self.target_pos.pose.orientation.z = z
            self.target_pos.pose.orientation.w = w
            print(f"Set the pitch to {int(self.pitch / np.pi * 180)} degree")
        elif event.name == 'j':
            self.yaw = (self.yaw - np.pi / 18) % (2 * np.pi)
            x, y, z, w = R.from_euler('ZYX', [self.yaw, self.pitch, 0]).as_quat()
            self.target_pos.pose.orientation.x = x
            self.target_pos.pose.orientation.y = y
            self.target_pos.pose.orientation.z = z
            self.target_pos.pose.orientation.w = w
            print(f"Set the yaw to {int(self.yaw /np.pi * 180)} degree")
        elif event.name == 'l':
            self.yaw = (self.yaw + np.pi / 18) % (2 * np.pi)
            x, y, z, w = R.from_euler('ZYX', [self.yaw, self.pitch, 0]).as_quat()
            self.target_pos.pose.orientation.x = x
            self.target_pos.pose.orientation.y = y
            self.target_pos.pose.orientation.z = z
            self.target_pos.pose.orientation.w = w
            print(f"Set the yaw to {int(self.yaw / np.pi * 180)} degree")
        elif event.name == 'esc':
            self.land()
        elif event.name == 't':
            # NOTE: probably conflict with other commands, need further security check
            self.takeoff(z=5)
        elif event.name == 'space':
            # RealSense D435
            # msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5.0)

            # Iris
            msg = rospy.wait_for_message("/iris/usb_cam/image_raw", Image, timeout=5.0)
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imwrite(os.path.join(self.image_path, f"idx-{self.idx}.jpg"), image)
            print(f"Saving image idx-{self.idx}.jpg.")
            self.idx = self.idx + 1
        elif event.name == 'c':
            print(f"set_position: x:{self.target_pos.pose.position.x}, {self.target_pos.pose.position.y}, {self.target_pos.pose.position.z}")
            print(f"set_orientation: yaw:{self.yaw}, pitch:{self.pitch}")

        

if __name__ == "__main__":

    rospy.init_node("position_controller")
    # FIXME: relative path does not work as expected
    controller = KeyboardController(image_path="/home/ps/catkin_ws/src/uav_positioning/savefile/img",
                                    log_path="/home/ps/catkin_ws/src/uav_positioning/savefile/log.txt")
    controller.main()
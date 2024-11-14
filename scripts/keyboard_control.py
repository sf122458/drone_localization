#! /home/ps/.conda/envs/hloc/bin/python

import rospy
from controller import BasicDroneController
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
# import keyboard # must run in root environment
import pynput
from cv_bridge import CvBridge
import cv2
import os
import datetime
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

class Logwriter:
    def __init__(self, path):
        self.path = path
        with open(self.path, 'w') as f:
            json.dump({}, f)

    def write(self, new_dict):
        # with open(self.path, 'w') as f:
        #     # f.write(msg)
        #     json.dump(new_dict, f)
        with open(self.path, "r", encoding="utf-8") as f:
            data = json.load(f)
            data.update(new_dict)
        with open(self.path, "w", encoding="utf-8") as f:
            json.dump(data, f)

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
        self.writer = Logwriter(log_path)
        self.bridge = CvBridge()
        self.idx = 0
        self.yaw = 0
        self.pitch = 0
        self.key_listener = pynput.keyboard.Listener(on_press=self.key_callback)
        self.key_listener.start()

    def main(self):
        last_req = rospy.Time.now()
        while not rospy.is_shutdown():

            if rospy.Time.now() - last_req > rospy.Duration(5.0):
                last_req = rospy.Time.now()
                print(f"Position: x:{self.local_pos.pose.position.x}, y:{self.local_pos.pose.position.y}, z:{self.local_pos.pose.position.z}")
                print(f"Orientation: x:{self.local_pos.pose.orientation.x}, y:{self.local_pos.pose.orientation.y}, z:{self.local_pos.pose.orientation.z}, w:{self.local_pos.pose.orientation.w}")
                # yaw, pitch, roll = R.from_quat([self.local_pos.pose.orientation.x, 
                #                                 self.local_pos.pose.orientation.y, 
                #                                 self.local_pos.pose.orientation.z, 
                #                                 self.local_pos.pose.orientation.w]).as_euler('ZYX', degrees=True)
                # print(f"Orientation: yaw:{yaw}, pitch:{pitch}, roll:{roll}")

            self.local_pos_pub.publish(self.target_pos)
            self.rate.sleep()


    def key_callback(self, event):
        distance = 0.5
        if event.char == 'q':
            self.target_pos.pose.position.z = self.target_pos.pose.position.z + distance
            print(f'+{distance} in z direction')
        elif event.char == 'e':
            self.target_pos.pose.position.z = self.target_pos.pose.position.z - distance
            print(f'-{distance} in z direction')
        elif event.char == 'w':
            self.target_pos.pose.position.x = self.target_pos.pose.position.x + distance
            print(f'+{distance} in x direction')
        elif event.char == 's':
            self.target_pos.pose.position.x = self.target_pos.pose.position.x - distance
            print(f'-{distance} in x direction')
        elif event.char == 'a':
            self.target_pos.pose.position.y = self.target_pos.pose.position.y + distance
            print(f'+{distance} in y direction')
        elif event.char == 'd':
            self.target_pos.pose.position.y = self.target_pos.pose.position.y - distance
            print(f'-{distance} in y direction')   
        elif event.char == 'i':
            self.pitch = (self.pitch + np.pi / 18) % (2 * np.pi).as_quat()
            self.target_pos.pose.orientation.x = x
            self.target_pos.pose.orientation.y = y
            self.target_pos.pose.orientation.z = z
            self.target_pos.pose.orientation.w = w
            print(f"Set the pitch to {int(self.pitch / np.pi * 180)} degree")
        elif event.char == 'k':
            self.pitch = (self.pitch - np.pi / 18) % (2 * np.pi)
            x, y, z, w = R.from_euler('ZYX', [self.yaw, self.pitch, 0]).as_quat()
            self.target_pos.pose.orientation.x = x
            self.target_pos.pose.orientation.y = y
            self.target_pos.pose.orientation.z = z
            self.target_pos.pose.orientation.w = w
            print(f"Set the pitch to {int(self.pitch / np.pi * 180)} degree")
        elif event.char == 'j':
            self.yaw = (self.yaw - np.pi / 18) % (2 * np.pi)
            x, y, z, w = R.from_euler('ZYX', [self.yaw, self.pitch, 0]).as_quat()
            self.target_pos.pose.orientation.x = x
            self.target_pos.pose.orientation.y = y
            self.target_pos.pose.orientation.z = z
            self.target_pos.pose.orientation.w = w
            print(f"Set the yaw to {int(self.yaw /np.pi * 180)} degree")
        elif event.char == 'l':
            self.yaw = (self.yaw + np.pi / 18) % (2 * np.pi)
            x, y, z, w = R.from_euler('ZYX', [self.yaw, self.pitch, 0]).as_quat()
            self.target_pos.pose.orientation.x = x
            self.target_pos.pose.orientation.y = y
            self.target_pos.pose.orientation.z = z
            self.target_pos.pose.orientation.w = w
            print(f"Set the yaw to {int(self.yaw / np.pi * 180)} degree")
        elif event.char == 'esc':
            self.land()
        elif event.char == 't':
            # NOTE: probably conflict with other commands, need further security check
            self.takeoff(z=5)
        elif event.char == 'space':
            # RealSense D435
            # msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5.0)

            # Iris
            msg = rospy.wait_for_message("/iris/usb_cam/image_raw", Image, timeout=5.0)
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imwrite(os.path.join(self.image_path, f"idx-{self.idx}.jpg"), image)
            info_dict = {
                f"idx-{self.idx}.jpg": {
                    'px': self.local_pos.pose.position.x,
                    'py': self.local_pos.pose.position.y,
                    'pz': self.local_pos.pose.position.z,
                    'qx': self.local_pos.pose.orientation.x,
                    'qy': self.local_pos.pose.orientation.y,
                    'qz': self.local_pos.pose.orientation.z,
                    'qw': self.local_pos.pose.orientation.w,
                },
            }
            self.writer.write(info_dict)
            print(f"Saving image idx-{self.idx}.jpg.")
            self.idx = self.idx + 1
        elif event.char == 'c':
            print(f"set_position: x:{self.target_pos.pose.position.x}, {self.target_pos.pose.position.y}, {self.target_pos.pose.position.z}")
            print(f"set_orientation: yaw:{self.yaw}, pitch:{self.pitch}")

        

if __name__ == "__main__":

    rospy.init_node("position_controller")
    time = datetime.datetime.now().strftime('%Y-%m-%d-%H:%M:%S')
    root_path = os.path.join(os.getcwd(), f"savefile-{time}")
    os.makedirs(root_path, exist_ok=True)
    controller = KeyboardController(image_path=root_path,
                                    log_path=root_path / "log.json")
    controller.main()
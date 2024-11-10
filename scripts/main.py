#! /usr/bin/env python

import os
import sys
sys.path.join("/home/ps/.conda/envs/hloc/lib/python3.8/site-packages")
sys.path.join("/home/ps/project/Hierarchical-Localization")


import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from pose_utils import PoseCalibrator, hlocBuilder
from cv_bridge import CvBridge
import numpy as np
import cv2


import pycolmap
from hloc.localize_sfm import QueryLocalizer, pose_from_cluster
from hloc import extract_features, match_features, pairs_from_exhaustive


def save_image_rs(img_path):
    os.makedirs(img_path, exist_ok=True)
    msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5.0)
    img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    img_name = "img"
    cv2.imwrite(os.path.join(img_path, f"{img_name}.jpg"))
    rospy.loginfo(f"Save image {img_name}.jpg to {img_path}")


# dataset下基准图片和query尽可能放在同一个大目录下的两个文件夹内
# def get_pose_from_hloc(query_path, model:pycolmap.Reconstruction):
#     references_registered = [model.images[i].name for i in model.reg_image_ids()]
#     extract_features.main(extract_features.confs['disk'], images, image_list=[query_path], feature_path=features, overwrite=True)
#     pairs_from_exhaustive.main(loc_pairs, image_list=[query_path], ref_list=references_registered)
#     match_features.main(match_features.confs['disk+lightglue'], loc_pairs, features=features, matches=matches, overwrite=True)
    
current_state = State()
def state_cb(msg):
    global current_state
    current_state = msg

local_pos = PoseStamped()
def local_pos_cb(msg:PoseStamped):
    global local_pos
    local_pos = msg

if __name__ == "__main__":
    rospy.init_node("camera_shot")

    calibrator = PoseCalibrator()
    hloc = hlocBuilder(ref_image_path='dataset', output_path='output')

    state_sub = rospy.Subscriber("mavros/state", State, state_cb)

    pose = PoseStamped()

    # 订阅无人机位置与姿态xyz, q
    local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, local_pos_cb)

    # 发布无人机位置与姿态xyz, q
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)


    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)




    ####################

    # 飞到多个点位进行拍摄

    ####################



    calibrator.calibrate()


    #####################

    query = None
    rigid = hloc.get_cam_pose(query)
    pose = calibrator.get_drone_pose(rigid)
    local_pos_pub.publish(pose)

    #####################
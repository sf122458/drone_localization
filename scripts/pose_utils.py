import numpy as np
from typing import List, Tuple
from pycolmap import Rigid3d

import pycolmap
from hloc.localize_sfm import QueryLocalizer, pose_from_cluster
from hloc import extract_features, match_features, pairs_from_exhaustive
from geometry_msgs.msg import PoseStamped

# utils for quaternion calculation

def quat2rot(q: Tuple):
    """
        quaterion to 3x3 rotation matrix
        :param q: (x, y, z, w) or [x, y, z, w], or a 2-dim array in shape [N, 4]
        :return: 3x3 rotation matrix if q is a single tuple or a 3-dim array in shape [N, 3, 3] if q is a 2-dim array
    """
    if isinstance(q, tuple) or isinstance(q, list):
        x, y, z, w = q
        return np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
                        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]])
    
    elif isinstance(q, np.ndarray):
        x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
        return np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
                        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]]).transpose(2, 0, 1)

def rot2quat(R: np.ndarray):
    """
        3x3 rotation matrix to quaternion
        :param R: 3x3 rotation matrix or a 3-dim array in shape [N, 3, 3]
        :return: (x, y, z, w) or a 2-dim array in shape [N, 4]
    """
    assert isinstance(R, np.ndarray)
    if len(R.shape) == 2:
        tr = np.trace(R)
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            w = 0.25 * S
            x = (R[2, 1] - R[1, 2]) / S
            y = (R[0, 2] - R[2, 0]) / S
            z = (R[1, 0] - R[0, 1]) / S
        return (x, y, z, w)
    elif len(R.shape) == 3:
        tr = np.trace(R, axis1=1, axis2=2)
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[:, 2, 1] - R[:, 1, 2]) / S
        y = (R[:, 0, 2] - R[:, 2, 0]) / S
        z = (R[:, 1, 0] - R[:, 0, 1]) / S
        return np.stack([x, y, z, w], axis=1)
    else:
        raise ValueError("Input must be a 2-dim or 3-dim array")

def quat_mul(q1, q2):
    """
        Multiply two quaternions
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = x1*w2 + y1*z2 - z1*y2 + w1*x2
    y = -x1*z2 + y1*w2 + z1*x2 + w1*y2
    z = x1*y2 - y1*x2 + z1*w2 + w1*z2
    w = -x1*x2 - y1*y2 - z1*z2 + w1*w2
    return (x, y, z, w)

def inv_quat(q):
    """
        Inverse the quaternion
    """
    x, y, z, w = q
    return (-x, -y, -z, w)


# NOTE: In the following code we use T_A2B to represent the symbol R^{B}_{A}


# Function for determine the coordinate transformation

def get_xyz_cam_in_point(rigid):
    """
        Get the camera central position in the coordinate system of the point cloud from the Rigid3d object.
    """
    assert isinstance(rigid, Rigid3d)
    return -np.linalg.inv(quat2rot(rigid.rotation.quat)) @ rigid.translation


# def calibrate_coord_transform(xyz_a: np.ndarray, xyz_b: np.ndarray):
#     """
#         Determine the coordinate transformation from xyz_a to xyz_b:
#             B = R @ S @ A + T
            
#         :param xyz_a: The coordinate in the coordinate system of point cloud. Shape must be [3, N]
#         :param xyz_b: The coordinate in the local coordinate system of drone. Shape must be [3, N]
#         :return: R_local2point, T_local2point, S
#     """
#     scale = np.sqrt(np.sum(np.square(xyz_b[:, 0] - xyz_b[:, 1])) / np.sum(np.square(xyz_a[:, 0] - xyz_a[:, 1])))
#     xyz_a = xyz_a * scale
#     S = np.diag([scale, scale, scale])
#     centroid_a = np.mean(xyz_a, axis=-1, keepdims=True)
#     centroid_b = np.mean(xyz_b, axis=-1, keepdims=True)
#     H = (xyz_a - centroid_a) @ (xyz_b - centroid_b).T
#     U, _, V = np.linalg.svd(H)
#     R = V.T @ U.T
#     # if np.linalg.det(R) < 0:
#     #   V[2, :] *= -1
#     #   R = V.T @ U.T
#     T = -R @ centroid_a + centroid_b
#     return R, T, S

def calibrate_xyz_transform(xyz_cam_in_point: np.ndarray, xyz_cam_in_local: np.ndarray):
    """
        Determine the coordinate transformation from xyz_a to xyz_b:
            B = R @ S @ A + T
            
        :param xyz_a: The coordinate in the coordinate system of point cloud. Shape must be [3, N]
        :param xyz_b: The coordinate in the local coordinate system of drone. Shape must be [3, N]
        :return: R_local2point, T_local2point, S
    """
    scale = np.sqrt(np.sum(np.square(xyz_cam_in_point[:, 0] - xyz_cam_in_point[:, 1])) / np.sum(np.square(xyz_cam_in_local[:, 0] - xyz_cam_in_local[:, 1])))
    xyz_cam_in_local = xyz_cam_in_local * scale
    S = np.diag([scale, scale, scale])
    centroid_a = np.mean(xyz_cam_in_local, axis=-1, keepdims=True)
    centroid_b = np.mean(xyz_cam_in_point, axis=-1, keepdims=True)
    H = (xyz_cam_in_local - centroid_a) @ (xyz_cam_in_point - centroid_b).T
    U, _, V = np.linalg.svd(H)
    R_point2local = V.T @ U.T
    # if np.linalg.det(R) < 0:
    #   V[2, :] *= -1
    #   R = V.T @ U.T
    T_point2local = -R_point2local @ centroid_a + centroid_b
    q_point2local = rot2quat(R_point2local)
    return R_point2local, T_point2local, S, q_point2local


# Function for determine the quaternion of the drone pose
# NOTE: q_point2cam == rigid.rotation.quat

def calibrate_quat(q_point2cam, q_point2local, q_local2drone):
    """
        Calibrate the rotation of the camera pose according to the drone pose
        return q_cam2drone
    """
    return quat_mul(quat_mul(inv_quat(q_point2cam), q_point2local), q_local2drone)


def get_drone_quat(q_point2cam, q_point2local, q_cam2drone):
    """
        Get the quaternion of the drone pose
        q_{drone2local} = q_{drone2cam} * q_{cam2point} * q_{point2local}
    """

    if isinstance(q_point2cam, Rigid3d):
        q_point2cam = q_point2cam.rotation.quat
    return quat_mul(quat_mul(inv_quat(q_point2local), q_point2cam), q_cam2drone)



# def calc_quat_transform(quat_a, quat_b):
#     """
#         R_B = R @ R_A
#     """
#     # TODO
#     if isinstance(quat_a, tuple) or isinstance(quat_a, list):
#         return quat2rot(quat_b) @ np.linalg.inv(quat2rot(quat_a))
#     else:
#         raise NotImplementedError


# def calc_orientation_2d(quat_a: np.ndarray, quat_b: np.ndarray):
#     """
#         R_B = R @ R_A
#     """


# Inputs contain the class Rigid3d: xyz_cam_in_point = -R^{-1} @ T, q_point2cam = rigid.rotation.quat
class PoseCalibrator:
    def __init__(self):
        self.R_point2local = None
        self.T_point2local = None
        self.S = None
        self.q_point2local = None
        self.q_cam2drone = None

        self.calibration = {
            "xyz_cam_in_point": [],
            "xyz_cam_in_local": [],
            "q_cam": [],
            "q_drone": []
        }

    """ def add_calibration(self, xyz_cam_in_point, xyz_cam_in_local, q_cam, q_drone):
        self.calibration["xyz_cam_in_point"].append(xyz_cam_in_point)
        self.calibration["xyz_cam_in_local"].append(xyz_cam_in_local)
        self.calibration["q_cam"].append(q_cam)
        self.calibration["q_drone"].append(q_drone) """

    def add_calibration(self, rigid, xyz_drone, q_drone):
        self.calibration["xyz_cam_in_point"].append(get_xyz_cam_in_point(rigid))
        self.calibration["xyz_cam_in_local"].append(xyz_drone)
        self.calibration["q_cam"].append(rigid.rotation.quat)
        self.calibration["q_drone"].append(q_drone)

    def callibrate_transform(self):
        xyz_a = np.array(self.calibration["xyz_cam_in_point"])
        xyz_b = np.array(self.calibration["xyz_cam_in_local"])
        scale = np.sqrt(np.sum(np.square(xyz_b[:, 0] - xyz_b[:, 1])) / np.sum(np.square(xyz_a[:, 0] - xyz_a[:, 1])))
        xyz_a = xyz_a * scale
        self.S = np.diag([scale, scale, scale])
        centroid_a = np.mean(xyz_a, axis=-1, keepdims=True)
        centroid_b = np.mean(xyz_b, axis=-1, keepdims=True)
        H = (xyz_a - centroid_a) @ (xyz_b - centroid_b).T
        U, _, V = np.linalg.svd(H)
        R = V.T @ U.T
        # if np.linalg.det(R) < 0:
        #   V[2, :] *= -1
        #   R = V.T @ U.T
        T = -R @ centroid_a + centroid_b
        self.R_point2local = R
        self.T_point2local = T
        self.q_point2local = rot2quat(R)

    def calibrate_quat(self):
        # TODO: how to increase the accuracy of the quaternion calculation with the given data
        q_point2cam = self.calibration["q_cam"]
        q_point2local = self.q_point2local
        q_local2drone = self.calibration["q_drone"]
        self.q_cam2drone = quat_mul(quat_mul(inv_quat(q_point2cam), q_point2local), q_local2drone)

    def calibrate(self):
        self.callibrate_transform()
        self.calibrate_quat()

    def get_drone_pose(self, rigid):
        assert isinstance(rigid, Rigid3d)
        pose = PoseStamped()
        xyz_cam_in_point = self.get_xyz_cam_in_point(rigid)
        xyz_drone = self.R_point2local @ self.S @ xyz_cam_in_point + self.T_point2local
        q_drone = quat_mul(quat_mul(self.inv_quat(self.q_point2local), rigid.rotation.quat), self.q_cam2drone)

        # return xyz_drone, q_drone

        # TODO:
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz_drone
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w \
        = q_drone

        return pose


    def get_xyz_cam_in_point(self, rigid):
        assert isinstance(rigid, Rigid3d)
        return quat_mul(-np.linalg.inv(quat2rot(rigid.rotation.quat)), rigid.translation)

    def inv_quat(self, q):
        """
            Inverse the quaternion
        """
        x, y, z, w = q
        return (-x, -y, -z, w)

    def quat_mul(self, q1, q2):
        """
            Multiply two quaternions
        """
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        x = x1*w2 + y1*z2 - z1*y2 + w1*x2
        y = -x1*z2 + y1*w2 + z1*x2 + w1*y2
        z = x1*y2 - y1*x2 + z1*w2 + w1*z2
        w = -x1*x2 - y1*y2 - z1*z2 + w1*w2
        return (x, y, z, w)

    def rot2quat(self, R):
        assert len(R.shape) == 2
        tr = np.trace(R)
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            w = 0.25 * S
            x = (R[2, 1] - R[1, 2]) / S
            y = (R[0, 2] - R[2, 0]) / S
            z = (R[1, 0] - R[0, 1]) / S
        return (x, y, z, w)

    def reset(self):
        self.R_point2local = None
        self.T_point2local = None
        self.S = None
        self.q_point2local = None
        self.q_cam2drone = None
        self.calibration = {
            "xyz_cam_in_point": [],
            "xyz_cam_in_local": [],
            "q_cam": [],
            "q_drone": []
        }


class hlocBuilder:
    def __init__(self, ref_image_path, output_path, ):
        # self.images = Path('datasets/sjtu')
        self.images = ref_image_path

        # output = Path('output/sjtu')
        output = output_path
        self.sfm_pairs = output / 'pairs-sfm.txt'
        self.loc_pairs = output / 'pairs-loc.txt'
        self.sfm_dir = output / 'sfm'
        self.features = output / 'features.h5'
        self.matches = output / 'matches.h5'


        self.feature_conf = extract_features.confs['disk']
        self.matcher_conf = match_features.confs['disk+lightglue']

        self.model = pycolmap.Reconstruction(self.sfm_dir)

        conf = {
            'estimation': {'ransac': {'max_error': 12}},
            'refinement': {'refine_focal_length': True, 'refine_extra_params': True}
        }
        # FIXME: localizer 能直接初始化吗？
        self.localizer = QueryLocalizer(self.model, conf)
        self.references_registered = [self.model.images[i].name for i in self.model.reg_images_ids()]


    def get_cam_pose(self, query):
        extract_features.main(self.feature_conf, self.images, image_list=[query], feature_path=self.features, overwrite=True)
        pairs_from_exhaustive.main(self.loc_pairs, image_list=[query], ref_list=self.references_registered)
        match_features.main(self.matcher_conf, self.loc_pairs, features=self.features, matches=self.matches, overwrite=True)
        camera = pycolmap.infer_camera_from_image(self.images / query)
        ref_ids = [self.model.find_image_with_name(n).image_id for n in self.references_registered]
        ret, log = pose_from_cluster(self.localizer, query, camera, ref_ids, self.features, self.matches)
        rigid = ret['cam_from_world']

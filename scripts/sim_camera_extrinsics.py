#!/usr/bin/env python3

"""
This script is used to calculate the extrinsics of the camera in the simulation.
"""

import numpy as np
from scipy.spatial.transform import Rotation as Rot
from math import pi

def main():
    
    """
    Main function.
    @return: the extrinsics of the camera.
    
    reference: turtlebot3_waffle.urdf.xacro
    """

    ### From base footprint to base link (base joint)
    xyz_base_footprint_to_base_link = np.array([0.0, 0.0, 0.010])
    rpy_base_footprint_to_base_link = np.array([0.0, 0.0, 0.0])
    R_base_footprint_to_base_link = Rot.from_euler('xyz', rpy_base_footprint_to_base_link).as_matrix()
    T_base_footprint_to_base_link = np.eye(4)
    T_base_footprint_to_base_link[0:3, 0:3] = R_base_footprint_to_base_link
    T_base_footprint_to_base_link[0:3, 3] = xyz_base_footprint_to_base_link

    ### From base link to camera link (camera joint)
    xyz_base_link_to_camera_link = np.array([0.064, -0.065, 0.094])
    rpy_base_link_to_camera_link = np.array([0.0, 0.0, 0.0])
    R_base_link_to_camera_link = Rot.from_euler('xyz', rpy_base_link_to_camera_link).as_matrix()
    T_base_link_to_camera_link = np.eye(4)
    T_base_link_to_camera_link[0:3, 0:3] = R_base_link_to_camera_link
    T_base_link_to_camera_link[0:3, 3] = xyz_base_link_to_camera_link

    ### From camera link to infra1 link (camera_infra1_joint)
    xyz_camera_link_to_infra1_link = np.array([0.0, 0.0, 0.0])
    rpy_camera_link_to_infra1_link = np.array([0.0, 0.0, 0.0])
    R_camera_link_to_infra1_link = Rot.from_euler('xyz', rpy_camera_link_to_infra1_link).as_matrix()
    T_camera_link_to_infra1_link = np.eye(4)
    T_camera_link_to_infra1_link[0:3, 0:3] = R_camera_link_to_infra1_link
    T_camera_link_to_infra1_link[0:3, 3] = xyz_camera_link_to_infra1_link

    ### From infra1 link to infra1 optical frame (infra1_optical_joint)
    xyz_infra1_link_to_infra1_optical_frame = np.array([0.0, 0.0, 0.0])
    rpy_infra1_link_to_infra1_optical_frame = np.array([-pi/2, 0.0, -pi/2])
    R_infra1_link_to_infra1_optical_frame = Rot.from_euler('xyz', rpy_infra1_link_to_infra1_optical_frame).as_matrix()
    T_infra1_link_to_infra1_optical_frame = np.eye(4)
    T_infra1_link_to_infra1_optical_frame[0:3, 0:3] = R_infra1_link_to_infra1_optical_frame
    T_infra1_link_to_infra1_optical_frame[0:3, 3] = xyz_infra1_link_to_infra1_optical_frame

    ### From camera link to infra 2 link (camera_infra2_joint)
    xyz_camera_link_to_infra2_link = np.array([0.0, -0.095, 0.0])
    rpy_camera_link_to_infra2_link = np.array([0.0, 0.0, 0.0])
    R_camera_link_to_infra2_link = Rot.from_euler('xyz', rpy_camera_link_to_infra2_link).as_matrix()
    T_camera_link_to_infra2_link = np.eye(4)
    T_camera_link_to_infra2_link[0:3, 0:3] = R_camera_link_to_infra2_link
    T_camera_link_to_infra2_link[0:3, 3] = xyz_camera_link_to_infra2_link

    ### From infra2 link to infra2 optical frame (infra2_optical_joint)
    xyz_infra2_link_to_infra2_optical_frame = np.array([0.0, 0.0, 0.0])
    rpy_infra2_link_to_infra2_optical_frame = np.array([-pi/2, 0.0, -pi/2])
    R_infra2_link_to_infra2_optical_frame = Rot.from_euler('xyz', rpy_infra2_link_to_infra2_optical_frame).as_matrix()
    T_infra2_link_to_infra2_optical_frame = np.eye(4)
    T_infra2_link_to_infra2_optical_frame[0:3, 0:3] = R_infra2_link_to_infra2_optical_frame
    T_infra2_link_to_infra2_optical_frame[0:3, 3] = xyz_infra2_link_to_infra2_optical_frame

    ### From base link to imu link (imu_joint)
    xyz_base_link_to_imu_link = np.array([0.0, 0.0, 0.068])
    rpy_base_link_to_imu_link = np.array([0.0, 0.0, 0.0])
    R_base_link_to_imu_link = Rot.from_euler('xyz', rpy_base_link_to_imu_link).as_matrix()
    T_base_link_to_imu_link = np.eye(4)
    T_base_link_to_imu_link[0:3, 0:3] = R_base_link_to_imu_link
    T_base_link_to_imu_link[0:3, 3] = xyz_base_link_to_imu_link

    ### Return the extrinsics of the camera
    T_imu_link_to_infra1_optical_frame = np.dot(np.linalg.inv(T_base_link_to_imu_link), np.dot(T_base_link_to_camera_link, np.dot(T_camera_link_to_infra1_link, T_infra1_link_to_infra1_optical_frame)))
    T_imu_link_to_infra2_optical_frame = np.dot(np.linalg.inv(T_base_link_to_imu_link), np.dot(T_base_link_to_camera_link, np.dot(T_camera_link_to_infra2_link, T_infra2_link_to_infra2_optical_frame)))

    return T_imu_link_to_infra1_optical_frame, T_imu_link_to_infra2_optical_frame

if __name__ == '__main__':
    Ts = main()

    print(f"T_imu_link_to_infra1_optical_frame: {Ts[0]}")
    print(f"T_imu_link_to_infra2_optical_frame: {Ts[1]}")
    # print(f"T_base_footprint_to_imu_link: {Ts[2]}")
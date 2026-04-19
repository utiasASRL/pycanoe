import os.path as osp
import numpy as np


class Calib:
    """
    Class for loading and storing calibration matrices.
    """

    def __init__(self, calib_root):
        self.P = np.loadtxt(osp.join(calib_root, "P_cam.txt"))
        self.T_boat_sonar = np.loadtxt(osp.join(calib_root, "T_boat_sonar.txt"))
        self.T_boat_lidar = np.loadtxt(osp.join(calib_root, "T_boat_lidar.txt"))
        self.T_cam_left_lidar = np.loadtxt(osp.join(calib_root, "T_cam_left_lidar.txt"))
        self.T_cam_left_cam_right = np.loadtxt(
            osp.join(calib_root, "T_cam_left_cam_right.txt")
        )
        self.T_lidar_radar = np.loadtxt(osp.join(calib_root, "T_lidar_radar.txt"))
        self.T_lidar_imu = np.loadtxt(osp.join(calib_root, "T_lidar_imu.txt"))
        self.T_novatel_boat = np.loadtxt(osp.join(calib_root, "T_novatel_boat.txt"))
        self.T_boat_motor_left = np.loadtxt(
            osp.join(calib_root, "T_boat_motor_left.txt")
        )
        self.T_boat_motor_right = np.loadtxt(
            osp.join(calib_root, "T_boat_motor_right.txt")
        )
        self.T_boat_sidescan_left = np.loadtxt(
            osp.join(calib_root, "T_boat_sidescan_left.txt")
        )
        self.T_boat_sidescan_mid = np.loadtxt(
            osp.join(calib_root, "T_boat_sidescan_mid.txt")
        )
        self.T_boat_sidescan_right = np.loadtxt(
            osp.join(calib_root, "T_boat_sidescan_right.txt")
        )

    def print_calibration(self):
        print("P:")
        print(self.P)
        print("T_boat_sonar:")
        print(self.T_boat_sonar)
        print("T_boat_lidar:")
        print(self.T_boat_lidar)
        print("T_cam_left_lidar")
        print(self.T_cam_left_lidar)
        print("T_cam_left_cam_right")
        print(self.T_cam_left_cam_right)
        print("T_lidar_radar")
        print(self.T_lidar_radar)
        print("T_lidar_imu")
        print(self.T_lidar_imu)
        print("T_novatel_boat")
        print(self.T_novatel_boat)
        print("T_boat_motor_left")
        print(self.T_boat_motor_left)
        print("T_boat_motor_right")
        print(self.T_boat_motor_right)
        print("T_boat_sidescan_left")
        print(self.T_boat_sidescan_left)
        print("T_boat_sidescan_mid")
        print(self.T_boat_sidescan_mid)
        print("T_boat_sidescan_right")
        print(self.T_boat_sidescan_right)

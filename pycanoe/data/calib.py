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
        self.T_camleft_lidar = np.loadtxt(osp.join(calib_root, "T_cam_lidar.txt"))
        self.T_camleft_camright = np.loadtxt(osp.join(calib_root, "T_left_right.txt"))
        self.T_lidar_radar = np.loadtxt(osp.join(calib_root, "T_lidar_radar.txt"))
        self.T_lidar_imu = np.loadtxt(osp.join(calib_root, "T_lidar_imu.txt"))
        self.T_novatel_boat = np.loadtxt(osp.join(calib_root, "T_novatel_boat.txt"))

    def print_calibration(self):
        print("P:")
        print(self.P)
        print("T_boat_sonar:")
        print(self.T_boat_sonar)
        print("T_boat_lidar:")
        print(self.T_boat_lidar)
        print("T_camleft_lidar")
        print(self.T_camleft_lidar)
        print("T_camleft_camright")
        print(self.T_camleft_camright)
        print("T_lidar_radar")
        print(self.T_lidar_radar)
        print("T_lidar_imu")
        print(self.T_lidar_imu)
        print("T_novatel_boat")
        print(self.T_novatel_boat)

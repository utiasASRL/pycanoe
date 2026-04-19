import os
import os.path as osp

import numpy as np

from pycanoe.data.calib import Calib
from pycanoe.data.sensors import Lidar, Radar, Sonar, Camera
from pycanoe.data.auxsensors import Motor, IMU, AuxCSV
from pycanoe.utils.utils import has_gap


class Sequence:
    """
    Class for working with an individual canoe dataset sequence
    """

    def __init__(self, canoe_root, seqSpec):
        """init
        Args:
            canoe_root (str): path to root folder: /path/to/data/canoe/
            seqSpec (list): defines sequence ID, start_time, and end_time (in microsec)
        """
        self.ID = seqSpec[0]
        if len(seqSpec) > 2:
            assert (
                seqSpec[2] > seqSpec[1]
            ), "Sequence timestamps must go forward in time"
            self.start_ts_micro = str(seqSpec[1])
            self.end_ts_micro = str(seqSpec[2])
        else:
            self.start_ts_micro = "0"  # dummy start and end if not specified
            self.dummy_ts = "9" * 21
            self.end_ts_micro = self.dummy_ts

        self.seq_root = osp.join(canoe_root, self.ID)
        self.novatel_root = osp.join(self.seq_root, "novatel")
        self.calib_root = osp.join(self.seq_root, "calib")

        self.lidar_root = osp.join(self.seq_root, "lidar")
        self.radar_root = osp.join(self.seq_root, "radar")
        self.sonar_root = osp.join(self.seq_root, "sonar")
        self.cam_left_root = osp.join(self.seq_root, "cam_left")
        self.cam_right_root = osp.join(self.seq_root, "cam_right")

        self.motor_csv_path = osp.join(self.seq_root, "motor", "power.csv")
        self.imu_csv_path = osp.join(self.seq_root, "imu", "imu.csv")

        self._check_dataroot_valid()  # Check if folder structure correct

        self.calib = Calib(self.calib_root)

        # Creates list of frame objects for cam, lidar, radar, sonar and inits poses
        self.get_all_frames()

        self._check_download()  # prints warning when sensor data missing

        self._check_gaps()  # prints warning when gaps in sensor timestamps (above threshold)

    def print(self):
        print("SEQ: {}".format(self.ID))
        if self.start_ts_micro != "0":
            print("START: {} END: {}".format(self.start_ts_micro, self.end_ts_micro))
        print("lidar frames: {}".format(len(self.lidar_frames)))
        print("radar frames: {}".format(len(self.radar_frames)))
        print("sonar frames: {}".format(len(self.sonar_frames)))
        print("cam_right frames: {}".format(len(self.cam_right_frames)))
        print("cam_left frames: {}".format(len(self.cam_left_frames)))
        print("motor frames: {}".format(len(self.motor_frames)))
        print("imu frames: {}".format(len(self.imu_frames)))
        print("-------------------------------")

    # region#--- Lidar ---#
    def get_lidar(self, idx):
        self.lidar_frames[idx].load_data()
        return self.lidar_frames[idx]

    @property
    def lidar(self):
        for lidar_frame in self.lidar_frames:
            lidar_frame.load_data()
            yield lidar_frame

    def get_lidar_iter(self):
        """Retrieves an iterator on lidar frames"""
        return iter(self.lidar)

    # endregion

    # region#--- Radar ---#
    def get_radar(self, idx):
        self.radar_frames[idx].load_data()
        return self.radar_frames[idx]

    @property
    def radar(self):
        for radar_frame in self.radar_frames:
            radar_frame.load_data()
            yield radar_frame

    def get_radar_iter(self):
        """Retrieves an iterator on radar frames"""
        return iter(self.radar)

    # endregion

    # region#--- Sonar ---#
    def get_sonar(self, idx):
        self.sonar_frames[idx].load_data()
        return self.sonar_frames[idx]

    @property
    def sonar(self):
        for sonar_frame in self.sonar_frames:
            sonar_frame.load_data()
            yield sonar_frame

    def get_sonar_iter(self):
        """Retrieves an iterator on sonar frames"""
        return iter(self.sonar)

    # endregion

    # region#--- Cam Left ---#
    def get_cam_left(self, idx):
        self.cam_left_frames[idx].load_data()
        return self.cam_left_frames[idx]

    @property
    def cam_left(self):
        for cam_left_frame in self.cam_left_frames:
            cam_left_frame.load_data()
            yield cam_left_frame

    def get_cam_left_iter(self):
        """Retrieves an iterator on cam_left frames"""
        return iter(self.cam_left)

    # endregion

    # region#--- Cam Right ---#
    def get_cam_right(self, idx):
        self.cam_right_frames[idx].load_data()
        return self.cam_right_frames[idx]

    @property
    def cam_right(self):
        for cam_right_frame in self.cam_right_frames:
            cam_right_frame.load_data()
            yield cam_right_frame

    def get_cam_right_iter(self):
        """Retrieves an iterator on cam_right frames"""
        return iter(self.cam_right)

    # endregion

    # region#--- Motor ---#
    def get_motor(self, idx):
        self.motor_frames[idx].load_data()
        return self.motor_frames[idx]

    @property
    def motor(self):
        for motor_frame in self.motor_frames:
            motor_frame.load_data()
            yield motor_frame

    def get_motor_iter(self):
        """Retrieves an iterator on motor frames"""
        return iter(self.motor)

    # endregion

    # region#--- IMU ---#
    def get_imu(self, idx):
        self.imu_frames[idx].load_data()
        return self.imu_frames[idx]

    @property
    def imu(self):
        for imu_frame in self.imu_frames:
            imu_frame.load_data()
            yield imu_frame

    def get_imu_iter(self):
        """Retrieves an iterator on imu frames"""
        return iter(self.imu)

    # endregion

    def _check_dataroot_valid(self):
        """Checks if the sequence folder structure is valid"""
        if not osp.isdir(self.novatel_root):
            print(
                "WARNING: novatel dir missing from sequence dataroot. This may be a test sequence."
            )
            # raise ValueError("ERROR: novatel dir missing from sequence dataroot")
        if not osp.isdir(self.calib_root):
            # print("WARNING: calib dir missing from sequence dataroot")
            raise ValueError(
                f"ERROR: calib dir ({self.calib_root}) missing from sequence dataroot"
            )

    def _check_download(self):
        """Checks if all sensor data has been downloaded, prints a warning otherwise"""
        # --- Regular Sensors ---#
        if osp.isdir(self.lidar_root) and len(os.listdir(self.lidar_root)) < len(
            self.lidar_frames
        ):
            print("WARNING: lidar frames are not all downloaded: {}".format(self.ID))

        if osp.isdir(self.radar_root) and len(os.listdir(self.radar_root)) < len(
            self.radar_frames
        ):
            print("WARNING: radar scans are not all downloaded: {}".format(self.ID))

        if osp.isdir(self.sonar_root) and len(os.listdir(self.sonar_root)) < len(
            self.sonar_frames
        ):
            print("WARNING: sonar scans are not all downloaded: {}".format(self.ID))

        if osp.isdir(self.cam_left_root) and len(os.listdir(self.cam_left_root)) < len(
            self.cam_left_frames
        ):
            print("WARNING: cam_left scans are not all downloaded: {}".format(self.ID))

        if osp.isdir(self.cam_right_root) and len(
            os.listdir(self.cam_right_root)
        ) < len(self.cam_right_frames):
            print("WARNING: cam right scans are not all downloaded: {}".format(self.ID))

        # --- Aux Sensors ---#
        if not osp.exists(self.motor_csv_path):
            print("WARNING: motor input csv not downloaded: {}".format(self.ID))

        if not osp.exists(self.imu_csv_path):
            print("WARNING: imu csv not downloaded: {}".format(self.ID))

        # --- Ground Truth ---#
        gtfile = osp.join(self.novatel_root, "novatel_poses.csv")
        if not osp.exists(gtfile):
            print(
                "WARNING: this may be a test sequence, or the groundtruth is not yet available: {}".format(
                    self.ID
                )
            )

    def _check_gaps(self, max_sec=5):
        """Checks for gaps in sensor data."""
        # --- Regular Sensors ---#
        if osp.isdir(self.lidar_root) and has_gap(self.lidar_frames, max_sec):
            print(f"WARNING: Gap(s) > {max_sec}s detected b/w lidar frames: {self.ID}")

        if osp.isdir(self.radar_root) and has_gap(self.radar_frames, max_sec):
            print(f"WARNING: Gap(s) > {max_sec}s detected b/w radar frames: {self.ID}")

        if osp.isdir(self.sonar_root) and has_gap(self.sonar_frames, max_sec):
            print(f"WARNING: Gap(s) > {max_sec}s detected b/w sonar frames: {self.ID}")

        if osp.isdir(self.cam_left_root) and has_gap(self.cam_left_frames, max_sec):
            print(
                f"WARNING: Gap(s) > {max_sec}s detected b/w cam_left frames: {self.ID}"
            )

        if osp.isdir(self.cam_right_root) and has_gap(self.cam_right_frames, max_sec):
            print(
                f"WARNING: Gap(s) > {max_sec}s detected b/w cam_right frames: {self.ID}"
            )

        # --- Aux Sensors ---#
        if osp.exists(self.motor_csv_path) and has_gap(self.motor_frames, max_sec):
            print(f"WARNING: Gap(s) > {max_sec}s detected b/w motor frames: {self.ID}")

        if osp.exists(self.imu_csv_path) and has_gap(self.imu_frames, max_sec):
            print(f"WARNING: Gap(s) > {max_sec}s detected b/w imu frames: {self.ID}")

    def _get_frames(self, posefile, root, ext, SensorType):
        """Initializes sensor frame objects with their ground truth pose information
        Args:
            posefile (str): path to ../sensor_poses.csv
            root (str): path to the root of the sensor folder ../sensor/
            ext (str): file extension specific to this sensor type
            SensorType (cls): sensor class specific to this sensor type
        Returns:
            frames (list): list of sensor frame objects
        """
        frames = []
        if osp.exists(posefile):
            with open(posefile, "r") as f:
                f.readline()  # header
                for line in f:
                    data = line.split(",")
                    ts = data[0]  # microsec
                    if float(self.start_ts_micro) <= float(ts) and float(ts) <= float(
                        self.end_ts_micro
                    ):
                        frame = SensorType(osp.join(root, ts + ext))
                        frame.init_pose(data)
                        frames.append(frame)
        elif osp.isdir(root):
            framenames = sorted([f for f in os.listdir(root) if f.endswith(ext)])
            for framename in framenames:
                ts = float(framename.split(".")[0])
                if float(self.start_ts_micro) <= ts and ts <= float(self.end_ts_micro):
                    frame = SensorType(osp.join(root, framename))
                    frames.append(frame)
        return frames

    def _get_aux_frames(self, csv_path, AuxSensorType):
        """Initializes aux sensor frame objects using the csv file

        Args:
            csv_path (str): path to <sensor>.csv
            AuxSensorType (cls): aux sensor class specific to this sensor

        Returns:
            frames (list): list of aux sensor frame objects
        """
        if not osp.exists(csv_path):
            print(f"WARNING: CSV path for Aux Sensor ({csv_path}) does not exist.")
            return []

        frames = []
        csv = AuxCSV.get_instance(csv_path)
        timestamps_micro = csv.get_all_timestamps_micro()

        for ts_micro in timestamps_micro:
            if float(self.start_ts_micro) <= ts_micro and ts_micro <= float(
                self.end_ts_micro
            ):
                frame = AuxSensorType(csv_path, ts_micro)
                frames.append(frame)
        return frames

    def get_all_frames(self):
        """Convenience method for retrieving sensor frames of all types"""
        lfile = osp.join(self.novatel_root, "lidar_poses.csv")
        rfile = osp.join(self.novatel_root, "radar_poses.csv")
        sfile = osp.join(self.novatel_root, "sonar_poses.csv")
        clfile = osp.join(self.novatel_root, "cam_left_poses.csv")
        crfile = osp.join(self.novatel_root, "cam_right_poses.csv")

        self.lidar_frames = self._get_frames(lfile, self.lidar_root, ".bin", Lidar)
        self.radar_frames = self._get_frames(rfile, self.radar_root, ".png", Radar)
        self.sonar_frames = self._get_frames(sfile, self.sonar_root, ".png", Sonar)
        self.cam_left_frames = self._get_frames(
            clfile, self.cam_left_root, ".png", Camera
        )
        self.cam_right_frames = self._get_frames(
            crfile, self.cam_right_root, ".png", Camera
        )

        self.motor_frames = self._get_aux_frames(self.motor_csv_path, Motor)
        self.imu_frames = self._get_aux_frames(self.imu_csv_path, IMU)

    def reset_frames(self):
        """Resets all frames, removes downloaded data"""
        self.get_all_frames()

    def synchronize_frames(self, ref="cam_left", threshold=5.0):
        """Simulates having synchronous measurements
        NOTE: measurements still won't be at the exact same timestamp and will have different poses
        However, for a given reference index, the other measurements will be as close to the reference
        in time as they can be.

        Args:
            ref (str): [cam_left, cam_right, lidar, radar, sonar, motor] this determines which sensor's frames will be used as the
                reference for synchronization. This sensor's list of frames will not be modified. However,
                the other lists of sensor frames will be modified so that each index will approximately
                align with the reference in time.
            threshold (float): allowable time difference between frames (seconds)
        """
        sensors = {
            "lidar": self.lidar_frames,
            "radar": self.radar_frames,
            "sonar": self.sonar_frames,
            "cam_left": self.cam_left_frames,
            "cam_right": self.cam_right_frames,
            "motor": self.motor_frames,
            "imu": self.imu_frames,
        }
        synch_sens = {}

        if ref not in sensors.keys():
            raise ValueError(f"Sensor {ref} not valid option for synchronizing frames.")

        if ref == "imu":
            print(
                "WARNING: Synchronizing frames to IMU is NOT recommended as IMU rate is much higher than other sensors."
            )

        ref_stamps = np.array([frame.timestamp for frame in sensors[ref]])
        assert np.all(
            ref_stamps[:-1] <= ref_stamps[1:]
        ), f"{ref} timestamps are not sorted."

        for sens, sens_frames in sensors.items():
            if sens == ref:
                synch_sens[sens] = sens_frames
                continue

            # Empty
            if not sens_frames:
                synch_sens[sens] = sens_frames
                continue

            sens_stamps = np.array([frame.timestamp for frame in sens_frames])
            assert np.all(
                sens_stamps[:-1] <= sens_stamps[1:]
            ), f"{sens} timestamps are not sorted."

            # Vectorized nearest-neighbor lookup
            idxs = np.searchsorted(sens_stamps, ref_stamps, side="left")
            idxs = np.clip(idxs, 0, len(sens_stamps) - 1)
            # Check if the previous index is closer
            prev_idxs = np.clip(idxs - 1, 0, len(sens_stamps) - 1)
            use_prev = np.abs(sens_stamps[prev_idxs] - ref_stamps) < np.abs(
                sens_stamps[idxs] - ref_stamps
            )
            idxs[use_prev] = prev_idxs[use_prev]

            # Warn if any matches exceed threshold
            diffs = np.abs(sens_stamps[idxs] - ref_stamps)
            if np.any(diffs > threshold):
                n = np.sum(diffs > threshold)
                print(
                    f"WARNING: {n} {sens} frames separated from ref by more than {threshold} sec."
                )

            synch_sens[sens] = [sens_frames[i] for i in idxs]

        self.lidar_frames = synch_sens["lidar"]
        self.radar_frames = synch_sens["radar"]
        self.sonar_frames = synch_sens["sonar"]
        self.cam_left_frames = synch_sens["cam_left"]
        self.cam_right_frames = synch_sens["cam_right"]
        self.motor_frames = synch_sens["motor"]
        self.imu_frames = synch_sens["imu"]

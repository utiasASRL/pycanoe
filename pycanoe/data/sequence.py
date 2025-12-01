import os
import os.path as osp

# from pycanoe.data.calib import Calib

# TODO: sonar, cam_left, cam_right
from pycanoe.data.sensors import Lidar, Radar
from pycanoe.utils.utils import get_closest_frame


class Sequence:
    """
    Class for working with an individual canoe dataset sequence
    """

    def __init__(self, canoe_root, seqSpec):
        """init
        Args:
            canoe_root (str): path to root folder ex: /path/to/data/canoe/
            seqSpec (list): defines sequence ID, start_time, and end_time (in microsec)
        """
        self.ID = seqSpec[0]
        if len(seqSpec) > 2:
            assert (
                seqSpec[2] > seqSpec[1]
            ), "Sequence timestamps must go forward in time"
            self.start_ts = str(seqSpec[1])
            self.end_ts = str(seqSpec[2])
        else:
            self.start_ts = "0"  # dummy start and end if not specified
            self.dummy_ts = "9" * 21
            self.end_ts = self.dummy_ts
        self.seq_root = osp.join(canoe_root, self.ID)
        self.novatel_root = osp.join(self.seq_root, "novatel")
        self.calib_root = osp.join(self.seq_root, "calib")
        self.lidar_root = osp.join(self.seq_root, "lidar")
        self.radar_root = osp.join(self.seq_root, "radar")

        self._check_dataroot_valid()  # Check if folder structure correct

        # TODO: uncomment calib
        # self.calib = Calib(self.calib_root)

        # Creates list of frame objects for cam, lidar, radar, sonar and inits poses
        # self.get_all_frames()
        # TODO: fix get frames
        self.lidar_frames = self._get_frames("", self.lidar_root, ".bin", Lidar)
        self.radar_frames = self._get_frames("", self.radar_root, ".png", Radar)

        self._check_download()  # prints warning when sensor data missing

    # TODO: sonar, cam_left, cam_right
    def print(self):
        print("SEQ: {}".format(self.ID))
        if self.end_ts != self.dummy_ts:
            print("START: {} END: {}".format(self.start_ts, self.end_ts))
        print("lidar frames: {}".format(len(self.lidar_frames)))
        print("radar frames: {}".format(len(self.radar_frames)))
        print("-------------------------------")

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

    # TODO: change back to errors
    def _check_dataroot_valid(self):
        """Checks if the sequence folder structure is valid"""
        if not osp.isdir(self.novatel_root):
            print("WARNING: novatel dir missing from dataroot")
            # raise ValueError("ERROR: novatel dir missing from dataroot")
        if not osp.isdir(self.calib_root):
            print("WARNING: calib dir missing from dataroot")
            # raise ValueError("ERROR: calib dir missing from dataroot")

    # TODO: add sonar, cam_l, cam_r
    def _check_download(self):
        """Checks if all sensor data has been downloaded, prints a warning otherwise"""
        if osp.isdir(self.lidar_root) and len(os.listdir(self.lidar_root)) < len(
            self.lidar_frames
        ):
            print("WARNING: lidar frames are not all downloaded: {}".format(self.ID))
        if osp.isdir(self.radar_root) and len(os.listdir(self.radar_root)) < len(
            self.radar_frames
        ):
            print("WARNING: radar scans are not all downloaded: {}".format(self.ID))
        gtfile = osp.join(self.novatel_root, "novatel_poses.csv")
        if not osp.exists(gtfile):
            print(
                "WARNING: this may be a test sequence, or the groundtruth is not yet available: {}".format(
                    self.ID
                )
            )

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
                    ts = data[0]
                    if self.start_ts <= ts and ts <= self.end_ts:
                        frame = SensorType(osp.join(root, ts + ext))
                        frame.init_pose(data)
                        frames.append(frame)
        elif osp.isdir(root):
            framenames = sorted([f for f in os.listdir(root) if f.endswith(ext)])
            for framename in framenames:
                ts = framename.split(",")[0]
                if self.start_ts <= ts and ts <= self.end_ts:
                    frame = SensorType(osp.join(root, framename))
                    frames.append(frame)
        return frames

    # TODO: add sonar, cam_l, cam_r
    def get_all_frames(self):
        """Convenience method for retrieving sensor frames of all types"""
        lfile = osp.join(self.novatel_root, "lidar_poses.csv")
        rfile = osp.join(self.novatel_root, "radar_poses.csv")
        self.lidar_frames = self._get_frames(lfile, self.lidar_root, ".bin", Lidar)
        self.radar_frames = self._get_frames(rfile, self.radar_root, ".png", Radar)

    def reset_frames(self):
        """Resets all frames, removes downloaded data"""
        self.get_all_frames()

    # TODO: add sonar, cam_l, cam_r
    def synchronize_frames(self, ref="camera"):
        """Simulates having synchronous measurements
        Note: measurements still won't be at the exact same timestamp and will have different poses
        However, for a given reference index, the other measurements will be as close to the reference
        in time as they can be.

        Args:
            ref (str): [camera, lidar, or radar] this determines which sensor's frames will be used as the
                reference for synchronization. This sensor's list of frames will not be modified. However,
                the other lists of sensor frames will be modified so that each index will approximately
                align with the reference in time.
        """
        lstamps = [frame.timestamp for frame in self.lidar_frames]
        rstamps = [frame.timestamp for frame in self.radar_frames]

        if ref == "lidar":
            if rstamps:
                self.radar_frames = [
                    get_closest_frame(lstamp, rstamps, self.radar_frames)
                    for lstamp in lstamps
                ]
        elif ref == "radar":
            if lstamps:
                self.lidar_frames = [
                    get_closest_frame(rstamp, lstamps, self.lidar_frames)
                    for rstamp in rstamps
                ]
        else:
            raise ValueError(
                f"Sensor {ref} not a valid option for synchronizing frames."
            )

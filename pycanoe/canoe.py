import multiprocessing
import os
from multiprocessing import Pool

from pycanoe.data.sequence import Sequence


class CanoeDataset:
    def __init__(self, root="/data/canoe/", split=None, verbose=False):
        self.root = root
        self.split = split

        self.lidar_frames = []
        self.radar_frames = []
        self.camleft_frames = []
        self.camright_frames = []
        self.sonar_frames = []

        self.motor_frames = []
        self.imu_frames = []

        self.sequences = []
        self.seqDict = {}  # seq string to index

        if split is None:
            split = sorted(
                [
                    [f]
                    for f in os.listdir(root)
                    if f.startswith("canoe-") and f != "canoe-test-gt"
                ]
            )

        # It takes a few seconds to construct each sequence, so we parallelize this
        global _load_seq

        def _load_seq(seqSpec):
            return Sequence(root, seqSpec)

        pool = Pool(multiprocessing.cpu_count())
        self.sequences = list(pool.map(_load_seq, split))
        self.sequences.sort(key=lambda x: x.ID)

        for seq in self.sequences:
            self.lidar_frames += seq.lidar_frames
            self.radar_frames += seq.radar_frames
            self.camleft_frames += seq.camleft_frames
            self.camright_frames += seq.camright_frames
            self.sonar_frames += seq.sonar_frames
            self.motor_frames += seq.motor_frames
            self.imu_frames += seq.imu_frames
            self.seqDict[seq.ID] = len(self.seqDict)
            if verbose:
                seq.print()

        if verbose:
            print("total cam left frames: {}".format(len(self.camleft_frames)))
            print("total cam right frames: {}".format(len(self.camright_frames)))
            print("total lidar frames: {}".format(len(self.lidar_frames)))
            print("total radar frames: {}".format(len(self.radar_frames)))
            print("total sonar frames: {}".format(len(self.sonar_frames)))
            print("total motor frames: {}".format(len(self.motor_frames)))
            print("total imu frames: {}".format(len(self.imu_frames)))

    def get_seq_from_ID(self, ID):
        return self.sequences[self.seqDict[ID]]

    def get_seq(self, idx):
        return self.sequences[idx]

    def get_cam_left(self, idx):
        self.camleft_frames[idx].load_data()
        return self.camleft_frames[idx]

    def get_cam_right(self, idx):
        self.camright_frames[idx].load_data()
        return self.camright_frames[idx]

    def get_lidar(self, idx):
        self.lidar_frames[idx].load_data()
        return self.lidar_frames[idx]

    def get_radar(self, idx):
        self.radar_frames[idx].load_data()
        return self.radar_frames[idx]

    def get_sonar(self, idx):
        self.sonar_frames[idx].load_data()
        return self.sonar_frames[idx]

    def get_motor(self, idx):
        self.motor_frames[idx].load_data()
        return self.motor_frames[idx]

    def get_imu(self, idx):
        self.imu_frames[idx].load_data()
        return self.imu_frames[idx]

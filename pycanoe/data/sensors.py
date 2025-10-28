import os.path as osp
from pathlib import Path
import cv2

from pycanoe.data.pointcloud import PointCloud
from pycanoe.utils.utils import (
    get_time_from_filename,
    get_time_from_filename_microseconds,
    load_lidar,
    load_radar,
)
from pycanoe.utils.vis_utils import vis_lidar, vis_camera


class Sensor:
    def __init__(self, path):
        self.path = path
        p = Path(path)
        self.frame = p.stem
        self.sensType = None
        self.seq_root = None
        self.sensor_root = None
        self.seqID = None

        # TODO: Fix for stereo
        # Parse directories
        if len(p.parts) >= 2:
            self.sensType = p.parts[-2]
            self.seq_root = str(Path(*p.parts[:-2]))
            self.sensor_root = osp.join(self.seq_root, self.sensType)
        if len(p.parts) >= 3:
            self.seqID = p.parts[-3]

        # TODO: self.pose, self.velocity, self.body_rate

        # TODO: Add try/except (catching specific exception)
        self.timestamp = get_time_from_filename(self.frame)
        self.timestamp_micro = get_time_from_filename_microseconds(self.frame)

    # TODO: Implement init pose
    def init_pose(self, data=None):
        raise NotImplementedError


class Lidar(Sensor, PointCloud):
    def __init__(self, path):
        Sensor.__init__(self, path)
        self.points = None

    def load_data(self):
        self.points = load_lidar(self.path)
        return self.points

    def visualize(self, **kwargs):
        return vis_lidar(self, **kwargs)

    def unload_data(self):
        self.points = None


class Camera(Sensor):
    def __init__(self, path):
        Sensor.__init__(self, path)
        self.img = None

    def load_data(self):
        img = cv2.imread(self.path)
        self.img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return self.img

    def visualize(self, **kwargs):
        return vis_camera(self, **kwargs)

    def unload_data(self):
        self.img = None


class Radar(Sensor):
    def __init__(self, path):
        Sensor.__init__(self, path)
        self.timestamps = None
        self.azimuths = None
        self.polar = None
        self.cartesian = None
        self.resolution = None  # TODO

    def load_data(self):
        self.timestamps, self.azimuths, _, self.polar, self.resolution = load_radar(
            self.path
        )

        cart_path = osp.join(self.sensor_root, "cart", self.frame + ".png")
        if osp.exists(cart_path):
            self.cartesian = cv2.imread(cart_path, cv2.IMREAD_GRAYSCALE)

        return self.timestamps, self.azimuths, self.polar

    def unload_data(self):
        self.timestamps = None
        self.azimuths = None
        self.polar = None
        self.cartesian = None

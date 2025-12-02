import os.path as osp
from pathlib import Path
import cv2
import yaml

from pycanoe.data.pointcloud import PointCloud
from pycanoe.utils.utils import (
    get_time_from_filename,
    get_time_from_filename_microseconds,
    get_gt_data_for_frame,
    get_state_from_gt_data,
    load_lidar,
    load_radar,
    load_sonar,
    radar_polar_to_cartesian,
    sonar_polar_to_cartesian,
)
from pycanoe.utils.vis_utils import vis_lidar, vis_camera, vis_radar, vis_sonar


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

    def init_pose(self, data=None):
        """Initializes pose variables with ground-truth novatel data

        Args:
            data (list): A list of floats corresponding to the line from the sensor_pose.csv file
                with the matching timestamp
        """
        if data is not None:
            gt = [float(x) for x in data]
        else:
            gt = get_gt_data_for_frame(self.seq_root, self.sensType, self.frame)

        self.pose, self.velocity, self.body_rate = get_state_from_gt_data(gt)

        return self.pose, self.velocity, self.body_rate


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
        self.config_path = osp.join(self.seq_root, "calib", "radar_config.yaml")

        self.timestamps = None
        self.azimuths = None
        self.polar = None
        self.resolution = None
        self.cartesian = None

        # Radar output: 8-bit power values per range bin
        # Range: (bin number * range resolution * gain) + offset
        self.gain = None
        self.offset = None
        self.encoder_size = None
        self.max_range = None

        self.min_range = 2.5  # m

    def load_config(self):
        with open(osp.join(self.config_path), "r") as f:
            config = yaml.load(f, yaml.SafeLoader)

        self.gain = config["range_gain"]
        self.offset = config["range_offset"]
        self.encoder_size = config["encoder_size"]
        self.max_range = config["max_range"][self.seqID]

        return self.gain, self.offset, self.encoder_size, self.max_range

    def load_data(self):

        self.load_config()

        self.timestamps, self.azimuths, _, self.polar, self.resolution = load_radar(
            self.path, self.encoder_size, self.min_range, self.max_range
        )

        cart_path = osp.join(self.sensor_root, "cart", self.frame + ".png")
        if osp.exists(cart_path):
            self.cartesian = cv2.imread(cart_path, cv2.IMREAD_GRAYSCALE)

        return self.timestamps, self.azimuths, self.polar

    def visualize(self, **kwargs):
        return vis_radar(self, **kwargs)

    def unload_data(self):
        self.timestamps = None
        self.azimuths = None
        self.polar = None
        self.cartesian = None
        self.resolution = None

        self.encoder_size = None
        self.max_range = None
        self.gain = None
        self.offset = None

    def polar_to_cart(
        self, cart_resolution, cart_pixel_width, polar=None, in_place=True
    ):
        """Converts a polar scan from polar to Cartesian format

        Args:
            cart_resolution (float): resolution of the output Cartesian image in (m / pixel)
            cart_pixel_width (int): width of the output Cartesian image in pixels
            polar (np.ndarray): if supplied, this function will use this input and not self.polar.
            in_place (bool): if True, self.cartesian is updated.
        """
        if polar is None:
            polar = self.polar
        cartesian = radar_polar_to_cartesian(
            self.azimuths, polar, self.resolution, cart_resolution, cart_pixel_width
        )
        if in_place:
            self.cartesian = cartesian
        return cartesian


class Sonar(Sensor):
    def __init__(self, path):
        Sensor.__init__(self, path)

        # Model: M3000d (freq: 1.2MHz)
        #  Max range: 30m
        #  Max range: 0.1m
        #  Num bearings = 512, from -65deg to 65deg
        #  Bearings per degree = 100
        #  Image:  378 (range bins) x 512 (beams)
        self.deg_per_enc = 0.01
        self.min_range = 0.1
        self.max_range = 30

        self.azimuths = None
        self.polar = None
        self.resolution = None
        self.cartesian = None

    def load_data(self):
        self.azimuths, self.polar, self.resolution = load_sonar(
            self.path, self.deg_per_enc, self.min_range, self.max_range
        )

    def visualize(self, **kwargs):
        return vis_sonar(self, **kwargs)

    def polar_to_cart(
        self, cart_resolution, cart_pixel_height, polar=None, in_place=True
    ):
        if polar is None:
            polar = self.polar

        cartesian = sonar_polar_to_cartesian(
            self.azimuths,
            polar,
            self.resolution,
            cart_resolution,
            cart_pixel_height,
        )
        if in_place:
            self.cartesian = cartesian
        return cartesian

    def unload_data(self):
        self.azimuths = None
        self.polar = None
        self.resolution = None
        self.cartesian = None

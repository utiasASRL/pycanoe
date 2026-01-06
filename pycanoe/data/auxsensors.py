from pathlib import Path
import os.path as osp
import numpy as np

from pycanoe.utils.utils import micro_to_sec, get_closest_index


class AuxSensor:
    """Class for auxillary sensors.

    Sensor data stored in csv (instead of individual timestamped files as in regular Sensors). Missing some fields compared to main Sensors (e.g., pose, body rate, velocity).
    """

    def __init__(self, csv_path, timestamp_micro):
        self.csv_path = csv_path

        self.sensType = None
        self.seq_root = None
        self.sensor_root = None
        self.seqID = None

        self.frame = np.int64(timestamp_micro)
        self.timestamp_micro = np.int64(timestamp_micro)
        self.timestamp = micro_to_sec(timestamp_micro)

        # Parse directories
        p = Path(csv_path)
        if len(p.parts) >= 2:
            self.sensType = p.parts[-2]
            self.seq_root = str(Path(*p.parts[:-2]))
            self.sensor_root = osp.join(self.seq_root, self.sensType)
        if len(p.parts) >= 3:
            self.seqID = p.parts[-3]

        # NOTE: Aux Sensors have no pose, velocity, bodyrate


class AuxCSV:
    """Singleton class for managing the CSVs that hold the auxillary sensor data.

    Assumes first row is header and first column is timestamp in microseconds (integer).
    """

    _instances = {}

    def __init__(self, csv_path):
        raise RuntimeError("Use AuxCSV.get_instance(csv_path) to load class.")

    @classmethod
    def get_instance(cls, csv_path):
        """Manage & return instances of csv (one instance per unique csv file)"""
        # Normalize for comparison
        norm_path = osp.realpath(osp.expanduser(csv_path))

        # New instance for each unique csv
        if norm_path not in cls._instances:
            cls._instances[norm_path] = cls._create(norm_path)

        return cls._instances[norm_path]

    @classmethod
    def _create(cls, norm_path):
        """Internal: Create new instance, bypassing __init__"""
        instance = object.__new__(cls)
        instance._initialize(norm_path)
        return instance

    def _initialize(self, norm_path):
        """Internal: Initialize new instance, bypassing __init__"""
        self.csv_path = norm_path
        self.data = None
        self.timestamps_micro = None
        # Dictionary of timestamp -> index for O(1) retrieval
        self.timestamps_micro_index = None

        if not osp.exists(self.csv_path):
            print(f"WARNING: Could not find file {self.csv_path}")

    def load_csv(self):
        """Load csv data, populate timestamps (micro) and timestamp micro lookup dict"""
        if self.data is None:
            data = np.loadtxt(self.csv_path, delimiter=",", skiprows=1)
            self.data = data
            self.timestamps_micro = data[:, 0].astype(np.int64)
            self.timestamps_micro_index = {
                ts: idx for idx, ts in enumerate(self.timestamps_micro)
            }

    def get_at_timestamp_micro(self, ts_micro: int):
        self.load_csv()

        if ts_micro not in self.timestamps_micro_index:
            raise ValueError(f"Timestamp {ts_micro} not found in CSV.")

        idx = self.timestamps_micro_index[ts_micro]
        return self.data[idx]

    def get_closest(self, ts_micro: int):
        self.load_csv()

        # Exact
        if ts_micro in self.timestamps_micro_index:
            idx = self.timestamps_micro_index[ts_micro]
        # Closest
        else:
            idx = get_closest_index(ts_micro, self.timestamps_micro)
            if (
                ts_micro < self.timestamps_micro[0]
                or ts_micro > self.timestamps_micro[-1]
            ):
                print(
                    f"WARNING: Timestamp {ts_micro} is out of CSV range "
                    f"{[self.timestamps_micro[0], self.timestamps_micro[-1]]}."
                )
        return self.data[idx]

    def get_all_timestamps_micro(self):
        self.load_csv()
        return self.timestamps_micro


# TODO: grab all within range
# TODO: (future) second multiplier to enable different granularity


class Motor(AuxSensor):
    """Motor power data.

    Attributes:
        starboard (float): Starboard motor power
        port (float): Port motor power
        total (float): Total power
    """

    def __init__(self, csv_path, timestamp_micro):
        AuxSensor.__init__(self, csv_path, timestamp_micro)
        self.starboard = None
        self.port = None
        self.total = None
        self.max = 500

    def load_data(self):
        csv = AuxCSV.get_instance(self.csv_path)
        line = csv.get_at_timestamp_micro(self.timestamp_micro)
        _, self.starboard, self.port, self.total = line

    def unload_data(self):
        self.starboard = None
        self.port = None
        self.total = None


class IMU(AuxSensor):
    """IMU data

    Attributes:
        body_angvel (np.array): body angular velocity (rad/s)
        body_acc (np.array): body linear acceleration, gravity not removed (m/s^2)
    """

    def __init__(self, csv_path, timestamp_micro):
        AuxSensor.__init__(self, csv_path, timestamp_micro)
        self.body_angvel = None
        self.body_acc = None

    def load_data(self):
        csv = AuxCSV.get_instance(self.csv_path)
        line = csv.get_at_timestamp_micro(self.timestamp_micro)
        _, wx, wy, wz, ax, ay, az = line
        self.body_angvel = np.array([wx, wy, wz])
        self.body_acc = np.array([ax, ay, az])

    def unload_data(self):
        self.body_angvel = None
        self.body_acc = None

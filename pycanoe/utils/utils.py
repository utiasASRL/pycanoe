# import os.path as osp
from pathlib import Path
import numpy as np
from bisect import bisect_left


def get_time_from_filename(file: str) -> float:
    """Retrieve an epoch time from a file name in seconds."""
    tstr = str(Path(file).stem)
    if len(tstr) < 10:
        raise ValueError("Expected at least 10 digits for epoch time.")
    # Manipulate w/ string to reduce rounding error
    tstr_dec = ".".join([tstr[:10], tstr[10:]])
    return float(tstr_dec)


def get_time_from_filename_microseconds(file: str) -> int:
    """Retrieve an epoch time from a file name in microseconds."""
    tstr = str(Path(file).stem)
    if len(tstr) != 16:
        raise ValueError("Expected 16 digits for epoch time in microseconds.")
    return int(tstr)


def _get_times_from_sensor_azimuths(timestamps):
    """Convert sensor azimuth timestamps from nanosec to seconds w/ microsec precision."""
    return (timestamps / 1e9).round(6)


def get_closest_index(query, targets):
    """Retrieve the index of the element in targets that is closest to query O(log n)

    Args:
        query (float): query value
        targets (list): sorted list of float values

    Returns:
        idx (int): index of the closest element in the array to x
    """
    # Check sorted
    assert is_sorted(targets), "Sorted list of targets required."

    idx = bisect_left(targets, query)

    # If reached end, set to last element
    if idx >= len(targets):
        idx = len(targets) - 1

    # Check if index above or below is closer to query
    d = abs(targets[idx] - query)
    if targets[idx] < query and idx < len(targets) - 1:
        if abs(targets[idx + 1] - query) < d:
            return idx + 1
    elif targets[idx] > query and idx > 0:
        if abs(targets[idx - 1] - query) < d:
            return idx - 1
    return idx


def get_closest_frame(query_time, frame_times, frames, threshold=3.0):
    """Retrieve the closest frame to query_time.

    Args:
        query_time (float): timestamp
        frame_times (list): sorted list of timestamps which corresponds to the frames list
        frames (list): list of frames

    Returns:
        closest_frame: frame with closest timestamp to query.
    """
    closest = get_closest_index(query_time, frame_times)
    assert (
        abs(query_time - frame_times[closest]) < threshold
    ), "Query and closest target frame separted by more than {}".format(threshold)
    return frames[closest]


def is_sorted(x):
    """Return True is x is a sorted list, otherwise False."""
    return (np.diff(x) >= 0).all()


def load_lidar(path):
    """Load LiDAR pointcloud (N,7) (np.ndarray, np.float64) from path.

    Load LiDAR pointcloud (np.ndarray, np.float64) (N, 7) from path.
    Format: [x, y, z, intensity, timestamp, reflectivity, ambient].
    """

    # Custom lidar data storage type
    bin_dtype = np.dtype(
        [
            ("x", np.float64),
            ("y", np.float64),
            ("z", np.float64),
            ("intensity", np.uint16),
            ("timestamp", np.uint64),
            ("reflectivity", np.uint16),
            ("ambient", np.uint16),
        ]
    )

    data = np.fromfile(path, dtype=bin_dtype)

    # Convert to np.ndarray of np.float64
    points = np.stack(
        (
            data["x"],
            data["y"],
            data["z"],
            data["intensity"].astype(np.float64),
            _get_times_from_sensor_azimuths(data["timestamp"].astype(np.float64)),
            data["reflectivity"].astype(np.float64),
            data["ambient"].astype(np.float64),
        ),
        axis=1,
    )

    return points

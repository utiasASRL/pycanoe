# import os.path as osp
from pathlib import Path
import numpy as np
import cv2
from bisect import bisect_left
import os.path as osp
import csv


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


def get_gt_data_for_frame(seq_root, sens_type, frame):
    """Retrieve ground-truth novatel (GPS) data for a given sensor frame

    Args:
        seq_root (str): path to the sequence root
        sens_type (str): lidar, radar, cam_left, cam_right, or sonar
        frame (str): name/timestamp (microsec) of the given sensor frame (without the extension)

    Returns:
        gt (list): A list of ground truth values from the applanix sensor_poses.scv
    """
    posepath = osp.join(seq_root, "novatel", sens_type + "_poses.csv")

    with open(posepath, "r") as f:
        reader = csv.reader(f)
        next(reader)  # header
        for line in reader:
            if line[0] == frame:
                return line

    assert 0, "gt not found for seq_root: {} sens_type: {} frame: {}".format(
        seq_root, sens_type, frame
    )
    return None


# TODO: Verify this mapping of the sensor_poses.py csv****
def get_state_from_gt_data(gt):
    """
    Convert line from sensor_poses.py into sensor state

    Input Line:
        0: utc_microsec [microsec]
        1-3: x,y,z (r_s_enuref_in_enuref. i.e., sensor in enuref) [m]
        4-6: ang_x, ang_y, ang_z (xyz angles from sensor to enuref w/ 321 sequence, C_es) [rad]
        7-9: vel_x_enuref, vel_y_enuref, vel_z_enuref (v_s_enuref_in_enuref, i.e., sensor speed in enuref) [m/s]
        10-12: angvel_x, angvel_y, angvel_z (w_s_enuref_in_s, i.e., angvel in sensor frame) [rad/s]
        13-15: vel_x_body, vel_y_body, vel_z_body (v_s_enuref_in_s) [m/s]

    Output:
        pose (T_enuref_sensor),

    """
    data = np.array(gt, dtype=np.float64)

    (
        __,
        x,
        y,
        z,
        ang_x,
        ang_y,
        ang_z,
        vel_x_enuref,
        vel_y_enuref,
        vel_z_enuref,
        angvel_x,
        angvel_y,
        angvel_z,
        vel_x_body,
        vel_y_body,
        vel_z_body,
    ) = data[0:16]

    T_enuref_sens = np.eye(4)
    T_enuref_sens[0:3, 0:3] = C_enuref_sens = zyx_ang_to_C(z=ang_z, y=ang_y, x=ang_x)
    T_enuref_sens[0:3, 3] = np.array([x, y, z])
    pose = T_enuref_sens

    v_sens_enuref_in_enuref = np.array([vel_x_enuref, vel_y_enuref, vel_z_enuref])
    v_sens_enuref_in_sens = np.array([vel_x_body, vel_y_body, vel_z_body])

    w_sens_enuref_in_sens = np.array([angvel_x, angvel_y, angvel_z])
    w_sens_enuref_in_enuref = C_enuref_sens @ w_sens_enuref_in_sens

    # Velocity = [v w] = varpi_sens_enuref_in_enuref
    velocity = np.concatenate([v_sens_enuref_in_enuref, w_sens_enuref_in_enuref])
    velocity = velocity[:, np.newaxis]

    # Body rate = [v w] = varpi_sens_enuref_in_sens
    body_rate = np.concatenate([v_sens_enuref_in_sens, w_sens_enuref_in_sens])
    body_rate = body_rate[:, np.newaxis]

    return pose, velocity, body_rate


def get_inverse_tf(T):
    """Returns the inverse of a given 4x4 homogeneous transform.
    Args:
        T (np.ndarray): 4x4 transformation matrix
    Returns:
        np.ndarray: inv(T)
    """
    T2 = np.identity(4, dtype=T.dtype)
    R = T[0:3, 0:3]
    t = T[0:3, 3].reshape(3, 1)
    T2[0:3, 0:3] = R.transpose()
    T2[0:3, 3:] = np.matmul(-1 * R.transpose(), t)
    return T2


def C1(ang_x):
    return np.array(
        [
            [1, 0, 0],
            [0, np.cos(ang_x), np.sin(ang_x)],
            [0, -np.sin(ang_x), np.cos(ang_x)],
        ],
        dtype=np.float64,
    )


def C2(ang_y):
    return np.array(
        [
            [np.cos(ang_y), 0, -np.sin(ang_y)],
            [0, 1, 0],
            [np.sin(ang_y), 0, np.cos(ang_y)],
        ],
        dtype=np.float64,
    )


def C3(ang_z):
    return np.array(
        [
            [np.cos(ang_z), np.sin(ang_z), 0],
            [-np.sin(ang_z), np.cos(ang_z), 0],
            [0, 0, 1],
        ],
        dtype=np.float64,
    )


def zyx_ang_to_C(z, y, x):
    """z,y,x angles to C matrix using 321 convention"""
    return C1(x) @ C2(y) @ C3(z)


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


# TODO: add gain & offset
def load_radar(path, encoder_size, min_range, max_range):
    """Decode a radar image w/ Oxford convention for encoding.

    Args:
        path: path to radar range image following Oxford encoding format
        encoder_size (int): number of discrete encoder positions
        min_range (float): range below which measurements are clipped (in meters)
        max_range (float): maximum range of the radar (in meters)

    Returns:
        timestamps (np.ndarray): Timestamp for each azimuth in int64 (UNIX time)
        azimuths (np.ndarray): Rotation for each polar radar azimuth (radians)
        valid (np.ndarray) Mask of whether azimuth data is an original sensor reading or interpolated from adjacent
            azimuths
        fft_data (np.ndarray): Radar power readings along each azimuth
    """
    # t = get_time_from_filename(path)
    raw_data = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    timestamps = raw_data[:, :8].copy().view(np.int64)
    azimuths = (
        raw_data[:, 8:10].copy().view(np.uint16) / float(encoder_size) * 2 * np.pi
    ).astype(np.float32)
    valid = raw_data[:, 10:11] == 255

    # Range data: (H,W), np.float32, 0-1
    fft_data = raw_data[:, 11:].astype(np.float32)[:, :, np.newaxis] / 255.0
    # Resolution = range/width (m/pix)
    resolution = max_range / fft_data.shape[1]
    min_pixel = int(round(min_range / resolution))
    fft_data[:, :min_pixel] = 0
    fft_data = np.squeeze(fft_data)
    return timestamps, azimuths, valid, fft_data, resolution

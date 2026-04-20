# This module is adapted from the Pyboreas codebase.
# Original source: https://github.com/utiasASRL/pyboreas
# Credit to the Pyboreas authors.

import os
import os.path as osp
from pathlib import Path

import numpy as np
from pylgmath import se3op

from pyboreas.utils.utils import (
    enforce_orthog,
    get_inverse_tf,
    yawPitchRollToRot,
)

def get_sequence_poses_gt(path, seq, dim, sonar=False):
    """Retrieves a list of the poses corresponding to the given sequences in the given file path with the CANOE dataset
    directory structure.
    Args:
        path (string): directory path to root directory of CANOE dataset
        seq (List[string]): list of sequence file names
        dim (int): dimension for evaluation. Set to '3' for 3D or '2' for 2D
    Returns:
        all_poses (List[np.ndarray]): list of 4x4 poses from all sequence files
        all_times (List[int]): list of times in microseconds from all sequence files
        seq_lens (List[int]): list of sequence lengths
        crop (List[Tuple]): sequences are cropped to prevent extrapolation, this list holds start and end indices
    """

    # loop for each sequence
    all_poses = []
    all_times = []
    seq_lens = []
    crop = []
    for filename in seq:
        # determine path to gt file
        dir = filename[:-4]  # assumes last four characters are '.txt'
        T_novatel_boat = np.loadtxt(osp.join(path, dir, "calib/T_novatel_boat.txt"))
        if dim == 3:
            filepath = osp.join(
                path, dir, "novatel/lidar_poses.csv"
            )  # use 'lidar_poses.csv' for groundtruth
            T_boat_lidar = np.loadtxt(osp.join(path, dir, "calib/T_boat_lidar.txt"))
            T_calib = T_boat_lidar
            poses, times = read_traj_file_gt(filepath, T_calib, dim)
            times_np = np.stack(times)

            # TO DO: ask Mia about cam eval
            filepath = osp.join(path, dir, 'applanix/camera_poses.csv')  # read in timestamps of camera groundtruth
            _, ctimes = read_traj_file_gt(filepath, np.identity(4), dim)
            if len(ctimes) == 0:
                # Something went wrong with loading camera timestamps, throw error
                raise ValueError(f"No camera timestamps found for sequence {dir}.")
            istart = np.searchsorted(times_np, ctimes[0])
            iend = np.searchsorted(times_np, ctimes[-1])
            poses = poses[istart:iend]
            times = times[istart:iend]
            crop += [(istart, iend)]

        elif dim == 2:
            if sonar:
                filepath = osp.join(
                    path, dir, "novatel/sonar_poses.csv"
                )  # use 'sonar_poses.csv' for groundtruth
                T_boat_sonar = np.loadtxt(osp.join(path, dir, "calib/T_boat_sonar.txt"))
                T_calib = T_boat_sonar
            else: # radar
                filepath = osp.join(
                    path, dir, "novatel/radar_poses.csv"
                )  # use 'radar_poses.csv' for groundtruth
                T_calib = np.identity(4) # to do: ask about this? does 2d need to be identity
            poses, times = read_traj_file_gt(filepath, T_calib, dim)
            crop += [(0, len(poses))]
        else:
            raise ValueError(
                "Invalid dim value in get_sequence_poses_gt. Use either 2 or 3."
            )

        seq_lens.append(len(times))
        all_poses.extend(poses)
        all_times.extend(times)
    return all_poses, all_times, seq_lens, crop


def read_traj_file_gt(path, T_ab, dim):
    """Reads trajectory from a comma-separated file, see CANOE documentation for format
    Args:
        path (string): file path including file name
        T_ab (np.ndarray): 4x4 transformation matrix for calibration. Poses read are in frame 'b', output in frame 'a'
        dim (int): dimension for evaluation. Set to '3' for 3D or '2' for 2D
    Returns:
        (List[np.ndarray]): list of 4x4 poses (from world to sensor frame)
        (List[int]): list of times in microseconds
    """
    with open(path, "r") as f:
        lines = f.readlines()
    poses = []
    times = []

    T_ab = enforce_orthog(T_ab)
    for line in lines[1:]:
        pose, time = convert_line_to_pose(line, dim)
        poses += [
            enforce_orthog(T_ab @ get_inverse_tf(pose))
        ]  # convert T_iv to T_vi and apply calibration
        times += [int(time)]  # microseconds
    return poses, times


def read_traj_file_gt2(path, dim=3):
    """Reads trajectory from a comma-separated file, see CANOE documentation for format
    Args:
        path (string): file path including file name
        dim (int): dimension for evaluation. Set to '3' for 3D or '2' for 2D
    Returns:
        (List[np.ndarray]): list of 4x4 poses
        (List[int]): list of times in microseconds
    """
    with open(path, "r") as f:
        lines = f.readlines()
    poses = []
    times = []
    for line in lines[1:]:
        pose, time = convert_line_to_pose(line, dim)
        poses.append(pose)
        times.append(time)  # microseconds
    return poses, times


def convert_line_to_pose(line, dim=3):
    """Reads trajectory from list of strings (single row of the comma-separeted groundtruth file). See CANOE
    documentation for format
    Args:
        line (List[string]): list of strings
        dim (int): dimension for evaluation. Set to '3' for 3D or '2' for 2D
    Returns:
        (np.ndarray): 4x4 SE(3) pose
        (int): time in nanoseconds
    """
    # returns T_iv
    line = line.replace("\n", ",").split(",")
    line = [float(i) for i in line[:-1]]
    # x, y, z -> 1, 2, 3
    # roll, pitch, yaw -> 4, 5, 6
    T = np.eye(4, dtype=np.float64)
    T[0, 3] = line[1]  # x
    T[1, 3] = line[2]  # y
    if dim == 3:
        T[2, 3] = line[3]  # z
        T[:3, :3] = yawPitchRollToRot(line[6], line[5], line[4])
    elif dim == 2:
        T[:3, :3] = yawPitchRollToRot(
            line[6],
            np.round(line[5] / np.pi) * np.pi,
            np.round(line[4] / np.pi) * np.pi,
        )
    else:
        raise ValueError(
            "Invalid dim value in convert_line_to_pose. Use either 2 or 3."
        )
    time = int(line[0])
    return T, time

def get_sequence_velocities_gt(path, seq, dim, sonar=False):
    """Retrieves a list of the velocities corresponding to the given sequences in the given file path with the CANOE dataset
    directory structure.
    Args:
        path (string): directory path to root directory of CANOE dataset
        seq (List[string]): list of sequence file names
        dim (int): dimension for evaluation. Set to '3' for 3D or '2' for 2D
    Returns:
        all_velocities (List[np.ndarray]): list of 4x4 poses from all sequence files
        all_times (List[int]): list of times in microseconds from all sequence files
        seq_lens (List[int]): list of sequence lengths
        crop (List[Tuple]): sequences are cropped to prevent extrapolation, this list holds start and end indices
    """

    # loop for each sequence
    all_velocities = []
    all_times = []
    seq_lens = []
    crop = []
    for filename in seq:
        # determine path to gt file
        dir = filename
        if dir.endswith('.txt'): 
            dir = dir[:-4]  # assumes last four characters are '.txt'
        if dim == 3:
            filepath = osp.join(
                path, dir, "novatel/lidar_poses.csv"
            )  # use 'lidar_poses.csv' for groundtruth
            T_calib = np.loadtxt(osp.join(path, dir, "calib/T_boat_lidar.txt"))
            velocities, times = read_vel_file_gt(filepath, T_calib, dim)
            times_np = np.stack(times)

            # TO DO: ask Mia about cam eval
            filepath = osp.join(path, dir, 'applanix/camera_poses.csv')  # read in timestamps of camera groundtruth
            _, ctimes = read_vel_file_gt(filepath, np.identity(4), dim)
            if len(ctimes) == 0:
                # Something went wrong with loading camera timestamps, throw error
                raise ValueError(f"No camera timestamps found for sequence {dir}.")
            istart = np.searchsorted(times_np, ctimes[0])
            iend = np.searchsorted(times_np, ctimes[-1])
            velocities = velocities[istart:iend]
            times = times[istart:iend]
            crop += [(istart, iend)]

        elif dim == 2:
            if sonar:
                filepath = osp.join(
                    path, dir, "novatel/sonar_poses.csv"
                )  # use 'sonar_poses.csv' for groundtruth
                T_calib = np.loadtxt(osp.join(path, dir, "calib/T_boat_sonar.txt")) # should this be identity too?
            else: # radar
                filepath = osp.join(
                    path, dir, "novatel/radar_poses.csv"
                )  # use 'radar_poses.csv' for groundtruth
                T_calib = np.identity(4)
            velocities, times = read_vel_file_gt(filepath, T_calib, dim)
            crop += [(0, len(velocities))]
        else:
            raise ValueError(
                "Invalid dim value in get_sequence_poses_gt. Use either 2 or 3."
            )

        seq_lens.append(len(times))
        all_velocities.extend(velocities)
        all_times.extend(times)

    return all_velocities, all_times, seq_lens, crop


def read_vel_file_gt(path, T_ab, dim):
    """Reads velocity from a comma-separated file, see CANOE documentation for format
    Args:
        path (string): file path including file name
        T_ab (np.ndarray): 4x4 transformation matrix for calibration. Velocities read are in frame 'b', output in frame 'a'
        dim (int): dimension for evaluation. Set to '3' for 3D or '2' for 2D
    Returns:
        (List[np.ndarray]): list of 4x4 poses (from world to sensor frame)
        (List[int]): list of times in microseconds
    """
    with open(path, "r") as f:
        lines = f.readlines()
    velocities = []
    times = []

    T_ab = enforce_orthog(T_ab)
    for line in lines[1:]:
        vel, time = convert_line_to_vel(line, dim)
        vel = se3op.tranAd(T_ab) @ vel
        # If 2D, zero out z, roll, pitch
        if dim == 2:
            vel[2:5] = 0.0
        velocities += [
            vel
        ]  # convert T_iv to T_vi and apply calibration
        times += [int(time)]  # microseconds
    return velocities, times


def convert_line_to_vel(line, dim=3):
    """Reads velocities from list of strings (single row of the comma-separeted groundtruth file). See CANOE
    documentation for format
    Args:
        line (List[string]): list of strings
        dim (int): dimension for evaluation. Set to '3' for 3D or '2' for 2D
    Returns:
        (np.ndarray): 4x4 SE(3) pose
        (int): time in nanoseconds
    """
    # returns T_iv
    line = line.replace("\n", ",").split(",")
    line = [float(i) for i in line[:-1]]

    # Get pose first
    # x, y, z -> 1, 2, 3
    # roll, pitch, yaw -> 4, 5, 6
    T = np.eye(4, dtype=np.float64)
    T[0, 3] = line[1]  # x
    T[1, 3] = line[2]  # y
    if dim == 3:
        T[2, 3] = line[3]  # z
        T[:3, :3] = yawPitchRollToRot(line[6], line[5], line[4])
    elif dim == 2:
        T[:3, :3] = yawPitchRollToRot(
            line[6],
            np.round(line[5] / np.pi) * np.pi,
            np.round(line[4] / np.pi) * np.pi,
        )
    else:
        raise ValueError(
            "Invalid dim value in convert_line_to_vel. Use either 2 or 3."
        )

    vbar = np.array([line[4], line[5], line[6]]).reshape(3, 1)
    vbar = np.matmul(T[:3, :3].T, vbar).squeeze()
    body_rate = np.array(
        [vbar[0], vbar[1], vbar[2], line[12], line[11], line[10]]
    ).reshape(6, 1)

    time = int(line[0])
    return body_rate, time
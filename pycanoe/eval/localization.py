# This module is adapted from the Pyboreas codebase.
# Original source: https://github.com/utiasASRL/pyboreas
# Credit to the Pyboreas authors.

import argparse
import os
import os.path as osp
from pathlib import Path

import numpy as np

from pycanoe.utils.odometry import read_traj_file_gt2
from pycanoe.data.splits import loc_test_reference
from pyboreas.utils.odometry import (
    plot_loc_stats,
    read_traj_file2,
)
from pyboreas.utils.utils import (
    SE3Tose3,
    get_closest_index,
    get_inverse_tf,
    rotToRollPitchYaw
)

LOC_SENSORS = ["lidar", "radar", "camera", "sonar"]

# Sequences whose xy RMSE norm exceeds this are left out of the overall RMSE (counted as failures).
MAX_XY_RMSE = 5.0


def get_Tbs(gtpath, seq, sensor="lidar"):
    """Returns T_boat_sensor from the sequence's calib folder."""
    if sensor == "sonar":
        return np.loadtxt(osp.join(gtpath, seq, "calib", "T_boat_sonar.txt"))
    T_boat_lidar = np.loadtxt(osp.join(gtpath, seq, "calib", "T_boat_lidar.txt"))
    if sensor == "camera":
        T_cam_left_lidar = np.loadtxt(osp.join(gtpath, seq, "calib", "T_cam_left_lidar.txt"))
        return np.matmul(T_boat_lidar, get_inverse_tf(T_cam_left_lidar))
    elif sensor == "radar":
        T_lidar_radar = np.loadtxt(osp.join(gtpath, seq, "calib", "T_lidar_radar.txt"))
        return np.matmul(T_boat_lidar, T_lidar_radar)
    return T_boat_lidar


def poses_file(sensor):
    """Ground truth poses file for a sensor; camera results are in the left camera frame."""
    return "cam_left_poses.csv" if sensor == "camera" else sensor + "_poses.csv"


def calib_files(sensor):
    """Calibration files get_Tbs reads for a sensor."""
    if sensor == "sonar":
        return ["calib/T_boat_sonar.txt"]
    extra = {"camera": ["calib/T_cam_left_lidar.txt"], "radar": ["calib/T_lidar_radar.txt"]}
    return ["calib/T_boat_lidar.txt"] + extra.get(sensor, [])


def check_time_match(pred_times, gt_times):
    assert len(pred_times) == len(
        gt_times
    ), f"pred time {len(pred_times)} is not equal to gt time {len(gt_times)}"
    p = np.array(pred_times)
    g = np.array(gt_times)
    assert np.sum(p - g) == 0


def check_ref_time_match(ref_times, gt_ref_times):
    indices = np.searchsorted(gt_ref_times, ref_times)
    p = np.array(ref_times)
    g = np.array(gt_ref_times)
    assert np.sum(g[indices] - p) == 0, f"{g[indices].shape} and {p.shape}"


def get_T_enu_s1(query_time, gt_times, gt_poses):
    closest = get_closest_index(query_time, gt_times)
    assert query_time == gt_times[closest], "query: {}".format(query_time)
    return gt_poses[closest]


def compute_errors(Te):
    r, p, y = rotToRollPitchYaw(Te[:3, :3])
    return [
        Te[0, 3],
        Te[1, 3],
        Te[2, 3],
        r * 180 / np.pi,
        p * 180 / np.pi,
        y * 180 / np.pi,
    ]


def root_mean_square(errs):
    return np.sqrt(np.mean(np.power(np.array(errs), 2), axis=0)).squeeze()


def eval_local(
    predpath,
    gtpath,
    gt_ref_seq=None,
    ref_sensor="lidar",
    test_sensor="lidar",
    dim=3,
    plot_dir=None,
):
    """Evaluates metric localization against a reference (map) sequence.

    Args:
        predpath: folder of <sequence>.txt prediction files
        gtpath: folder of ground truth sequences (test and reference)
        gt_ref_seq: reference sequence for every prediction file, which may be any CANOE sequence.
            By default (the benchmark), only loc_test sequences are scored, each against its
            route's reference from pycanoe.data.splits.loc_test_reference
        ref_sensor, test_sensor: one of LOC_SENSORS
        dim: 3 for SE(3) or 2 for SE(2)
        plot_dir: if set, per-sequence plots and localization_results.txt are written here

    Returns:
        rmse: mean RMSE over successful sequences [x, y, z (m), roll, pitch, yaw (deg)], with the
            mean consistency appended when every sequence includes covariances
        seq_rmse: per-sequence RMSE, one row per entry of gt_seqs
        gt_seqs: evaluated sequences
        num_removed: sequences left out of rmse because their xy RMSE exceeds MAX_XY_RMSE
    """
    if plot_dir is not None:
        os.makedirs(plot_dir, exist_ok=True)
    if gt_ref_seq:
        is_test_seq = lambda seq: seq.startswith("canoe-20")
    else:
        is_test_seq = lambda seq: seq in loc_test_reference
    pred_files = sorted(
        f
        for f in os.listdir(predpath)
        if f.endswith(".txt") and "err" not in f and is_test_seq(Path(f).stem.split(".")[0])
    )
    gt_seqs = []
    for predfile in pred_files:
        predfile_seq = Path(predfile).stem.split(".")[0]
        if predfile_seq not in os.listdir(gtpath):
            raise Exception(
                f"prediction file {predfile} doesn't match ground truth sequence list"
            )
        gt_seqs.append(predfile_seq)
    if not gt_seqs:
        expected = "canoe-<date>.txt" if gt_ref_seq else "one per loc_test sequence"
        raise Exception(f"no prediction files ({expected}) found in {predpath}")

    seq_rmse = []
    seq_consist = []
    seqs_have_cov = True
    for predfile, seq in zip(pred_files, gt_seqs):
        ref_seq = gt_ref_seq or loc_test_reference[seq]
        print("Processing {} (reference {})...".format(seq, ref_seq))
        gt_ref_poses, gt_ref_times = read_traj_file_gt2(
            osp.join(gtpath, ref_seq, "novatel", poses_file(ref_sensor)), dim=dim
        )
        T_bs = get_Tbs(gtpath, seq, ref_sensor) # T_boat_sensor
        T_sb = get_inverse_tf(T_bs)
        pred_poses, pred_times, ref_times, cov_matrices, has_cov = read_traj_file2(
            osp.join(predpath, predfile)
        )
        seqs_have_cov *= has_cov
        gt_poses, gt_times = read_traj_file_gt2(
            osp.join(gtpath, seq, "novatel", poses_file(test_sensor)), dim=dim
        )
        # check that pred_times is a 1-to-1 match with gt_times
        check_time_match(pred_times, gt_times)
        # check that each ref time matches to one gps_ref_time
        check_ref_time_match(ref_times, gt_ref_times)
        errs = []
        consist = []
        T_gt_seq = []
        T_pred_seq = []
        Xi = []
        Cov = []
        for j, pred_T_s1_s2 in enumerate(pred_poses):
            gt_T_enu_s2 = gt_poses[j]
            T_gt_seq.append(get_inverse_tf(gt_T_enu_s2))

            gt_T_enu_s1 = get_T_enu_s1(ref_times[j], gt_ref_times, gt_ref_poses)
            T_pred_seq.append(get_inverse_tf(gt_T_enu_s1 @ pred_T_s1_s2))

            gt_T_s1_s2 = get_inverse_tf(gt_T_enu_s1) @ gt_T_enu_s2
            T = pred_T_s1_s2 @ get_inverse_tf(gt_T_s1_s2)
            Te = T_bs @ T @ T_sb
            errs.append(compute_errors(Te))
            # If the user submitted a covariance matrix, calculate consistency
            if has_cov:
                if abs(np.sum(cov_matrices[j] - np.identity(6))) < 1e-3:
                    consist.append(1)
                xi = SE3Tose3(T)
                Xi.append(xi.squeeze())
                c = xi.T @ cov_matrices[j] @ xi
                consist.append(
                    c[0, 0]
                )  # assumes user has uploaded inverse covariance matrices
                Cov.append(1 / cov_matrices[j].diagonal())
        Xi = np.array(Xi)
        Cov = np.array(Cov)
        if plot_dir is not None:
            plot_loc_stats(
                seq, plot_dir, T_pred_seq, T_gt_seq, errs, consist, Xi, Cov, has_cov
            )
        rmse = root_mean_square(errs)
        seq_rmse.append(rmse)
        print(
            "RMSE: x: {} m y: {} m z: {} m roll: {} deg pitch: {} deg yaw: {} deg".format(
                rmse[0], rmse[1], rmse[2], rmse[3], rmse[4], rmse[5]
            )
        )
        c = -1
        if has_cov:
            c = np.sqrt(max(0, np.mean(consist) / 6.0))
            print("Consistency: {}".format(c))
        seq_consist.append(c)
        print(" ")

    seq_rmse = np.array(seq_rmse)

    # Sequences that failed to localize would dominate the mean; count them instead.
    valid = np.linalg.norm(seq_rmse[:, :2], axis=1) <= MAX_XY_RMSE
    num_removed = int(np.sum(~valid))
    if num_removed > 0:
        print(f"Removed {num_removed} sequences with xy RMSE > {MAX_XY_RMSE} m from the overall RMSE.")
    if np.any(valid):
        rmse = np.mean(seq_rmse[valid], axis=0).squeeze()
        print(
            "Overall RMSE: x: {} m y: {} m z: {} m roll: {} deg pitch: {} deg yaw: {} deg".format(
                rmse[0], rmse[1], rmse[2], rmse[3], rmse[4], rmse[5]
            )
        )
    else:
        rmse = np.full(6, np.nan)
        print("No sequence was localized within {} m; overall RMSE is undefined.".format(MAX_XY_RMSE))
    if seqs_have_cov:
        c = np.mean(seq_consist)
        print("Overall Consistency: {}".format(c))
        rmse = np.append(rmse, c)

    if plot_dir is not None:
        with open(osp.join(plot_dir, "localization_results.txt"), "w") as f:
            f.write("seq, x_rmse, y_rmse, z_rmse, roll_rmse, pitch_rmse, yaw_rmse")
            f.write(", consistency\n" if seqs_have_cov else "\n")
            for seq, row, c in zip(gt_seqs, seq_rmse, seq_consist):
                line = seq + ", " + ", ".join(f"{x:.4f}" for x in row)
                f.write(line + (f", {c:.4f}\n" if seqs_have_cov else "\n"))

    return rmse, seq_rmse, gt_seqs, num_removed


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--pred", type=str, help="path to prediction files")
    parser.add_argument("--gt", type=str, help="path to groundtruth sequences")
    parser.add_argument(
        "--ref_seq",
        default=None,
        type=str,
        help="Reference sequence for every test sequence (default: each route's own reference)",
    )
    parser.add_argument(
        "--ref_sensor",
        default="lidar",
        type=str,
        help="Which sensor to use as a reference (camera|lidar|radar|sonar)",
    )
    parser.add_argument(
        "--test_sensor",
        default="lidar",
        type=str,
        help="Which sensor to use as the test sensor (camera|lidar|radar|sonar)",
    )
    parser.add_argument("--dim", default=3, type=int, help="SE(3) or SE(2)")
    parser.add_argument("--plot", type=str, help="path to save plots")
    args = parser.parse_args()
    assert args.ref_sensor in LOC_SENSORS
    assert args.test_sensor in LOC_SENSORS
    assert args.dim in [2, 3]
    if args.ref_sensor == "radar" or args.test_sensor == "radar":
        assert args.dim == 2
    if args.ref_sensor == "sonar" or args.test_sensor == "sonar":
        assert args.dim == 2
    eval_local(
        args.pred,
        args.gt,
        args.ref_seq,
        args.ref_sensor,
        args.test_sensor,
        args.dim,
        args.plot,
    )

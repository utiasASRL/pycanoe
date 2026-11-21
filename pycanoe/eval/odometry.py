# This module is adapted from the Pyboreas codebase.
# Original source: https://github.com/utiasASRL/pyboreas
# Credit to the Pyboreas authors.

import argparse
import numpy as np
import matplotlib.pyplot as plt
import os.path as osp

from pycanoe.utils.odometry import (
    get_sequence_poses_gt,
    get_sequence_velocities_gt,
)

from pyboreas.utils.odometry import (
    compute_kitti_metrics,
    get_sequence_poses,
    get_sequences,
    get_sequence_velocities,
    compute_vel_metrics
)


# Like the Boreas benchmark, sequences at or above this translation error (%) count as failures and are
# left out of the overall error.
MAX_TRANS_ERR = 3.0


def _take(vals, idx):
    """vals[idx] for a list or an array."""
    return vals[idx] if isinstance(vals, np.ndarray) else [vals[i] for i in idx]


def align_to_timestamps(gt_vals, times_gt, lens_gt, pred_vals, times_pred, lens_pred, seq, verbose=True):
    """For each sequence whose ground truth and prediction lengths differ, keeps the predictions
    within the ground truth's time span and pairs each with the closest ground truth entry, so runs
    that skip or drop frames can still be scored. Inputs are concatenated over sequences; returns
    (gt_vals, lens_gt, pred_vals, lens_pred) in the same layout, with equal lengths per sequence.
    """
    times_gt = np.asarray(times_gt, dtype=np.int64)
    times_pred = np.asarray(times_pred, dtype=np.int64)
    gt_idx, pred_idx, new_lens_gt, new_lens_pred = [], [], [], []
    g0 = p0 = 0
    for s, n_gt, n_pred in zip(seq, lens_gt, lens_pred):
        g = np.arange(g0, g0 + n_gt)
        p = np.arange(p0, p0 + n_pred)
        if n_gt != n_pred:
            tg, tp = times_gt[g], times_pred[p]
            p = p[(tp >= tg[0]) & (tp <= tg[-1])]
            tp = times_pred[p]
            nearest = np.clip(np.searchsorted(tg, tp), 1, n_gt - 1)
            nearest -= (tp - tg[nearest - 1]) < (tg[nearest] - tp)
            g = g0 + nearest
            gap = int(np.max(np.abs(tg[nearest] - tp))) if len(p) else 0
            if verbose:
                print(f"Warning: {s} has {n_gt} ground truth poses and {n_pred} predictions; scoring "
                      f"the {len(p)} predictions in the ground truth's time span, matched by timestamp "
                      f"(largest gap {gap} us). Results may be inaccurate.")
        gt_idx.extend(g)
        pred_idx.extend(p)
        new_lens_gt.append(len(g))
        new_lens_pred.append(len(p))
        g0, p0 = g0 + n_gt, p0 + n_pred
    return _take(gt_vals, gt_idx), new_lens_gt, _take(pred_vals, pred_idx), new_lens_pred


def eval_odom(pred="test/demo/pred/3d", gt="test/demo/gt", radar=False, sonar=False,
              align_timestamps=False):
    """Returns the mean translation (%) and rotation (deg/m) error over successful sequences, and the
    number of sequences left out because their translation error is at least MAX_TRANS_ERR.

    align_timestamps: score predictions that skip frames by matching each one to the closest ground
    truth pose (see align_to_timestamps). Off for benchmark submissions, which must have every frame.
    """
    # evaluation mode
    dim = 2 if radar else 3

    # parse sequences
    seq = get_sequences(pred, ".txt")
    T_pred, times_pred, seq_lens_pred = get_sequence_poses(pred, seq)

    # For sequences where there are extractor specific parameters in the name
    # Ex: 'canoe-2025-08-21-19-16_test1.txt' -> 'canoe-2025-08-21-19-16.txt'
    seq_gt = []
    for s in seq:
        curr_seq = s.split('_')[0]
        if ".txt" != curr_seq[-4:]: 
            curr_seq=curr_seq+".txt"
        seq_gt.append(curr_seq)

    # get corresponding groundtruth poses
    T_gt, times_gt, seq_lens_gt, crop = get_sequence_poses_gt(gt, seq_gt, dim, sonar)
    if align_timestamps:
        T_gt, seq_lens_gt, T_pred, seq_lens_pred = align_to_timestamps(
            T_gt, times_gt, seq_lens_gt, T_pred, times_pred, seq_lens_pred, seq
        )

    # compute errors
    _, _, err_list = compute_kitti_metrics(
        T_gt, T_pred, seq_lens_gt, seq_lens_pred, seq, pred, dim, crop
    )

    # average over sequences that didn't fail (per-sequence [t_err, r_err])
    errors = np.asarray(err_list, dtype=np.float64).reshape(-1, 2)
    valid = errors[:, 0] < MAX_TRANS_ERR
    num_removed = int(np.sum(~valid))
    if num_removed > 0:
        print(f"Removed {num_removed} sequences with translation error >= {MAX_TRANS_ERR}% from the overall error.")
    t_err, r_err = np.mean(errors[valid], axis=0) if np.any(valid) else (np.nan, np.nan)

    # print out results
    print("Evaluated sequences: ", seq)
    print("Overall error: ", t_err, " %, ", r_err, " deg/m")

    return t_err, r_err, num_removed

def eval_odom_vel(pred="test/demo/pred/3d", gt="test/demo/gt", radar=False, sonar=False,
                  align_timestamps=False):
    # evaluation mode
    dim = 2 if radar or sonar else 3

    # parse sequences
    seq = get_sequences(pred, ".txt")
    vel_pred, times_pred, seq_lens_pred = get_sequence_velocities(pred, seq, dim)

    # For sequences where there are extractor specific parameters in the name
    # Ex: 'canoe-2025-08-21-19-16_test1.txt' -> 'canoe-2025-08-21-19-16.txt'
    seq_gt = []
    for s in seq:
        curr_seq = s.split('_')[0]
        if ".txt" != curr_seq[-4:]: 
            curr_seq=curr_seq+".txt"
        seq_gt.append(curr_seq)
    
    # get corresponding groundtruth poses
    vel_gt, times_gt, seq_lens_gt, crop = get_sequence_velocities_gt(gt, seq_gt, dim, sonar)
    if align_timestamps:
        # trim the prediction times with the same indices as the velocities
        args = (times_gt, seq_lens_gt)
        _, _, times_pred_kept, _ = align_to_timestamps(
            times_gt, *args, times_pred, times_pred, seq_lens_pred, seq, verbose=False
        )
        vel_gt, seq_lens_gt, vel_pred, seq_lens_pred = align_to_timestamps(
            vel_gt, *args, vel_pred, times_pred, seq_lens_pred, seq
        )
        times_pred = times_pred_kept

    # compute errors
    v_rmse, v_mean = compute_vel_metrics(vel_gt, vel_pred, times_pred, seq_lens_gt, seq_lens_pred, seq, pred, dim, crop)

    # print out results
    print("Evaluated sequences: ", seq)
    print("Overall velocity RMSE: ", np.round(v_rmse, 3), " [m/s, m/s, m/s, deg/s, deg/s, deg/s]")
    print("Overall velocity mean: ", np.round(v_mean, 3), " [m/s, m/s, m/s, deg/s, deg/s, deg/s]")

    return v_rmse, v_mean


if __name__ == "__main__":
    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--pred", default="test/demo/pred/3d", type=str, help="path to prediction files"
    )
    parser.add_argument(
        "--gt", default="test/demo/gt", type=str, help="path to groundtruth files"
    )
    parser.add_argument(
        "--radar",
        dest="radar",
        action="store_true",
        help="evaluate radar odometry in SE(2)",
    )
    parser.add_argument(
        "--sonar",
        dest="sonar",
        action="store_true",
        help="evaluate sonar odometry in SE(2)",
    )

    parser.add_argument(
        "--velocity", default=None, type=str, help="path to prediction files"
    )

    parser.add_argument(
        "--align-timestamps",
        dest="align_timestamps",
        action="store_true",
        help="score runs that skip frames by matching each prediction to the closest ground truth "
        "timestamp (not used for benchmark submissions)",
    )

    parser.set_defaults(radar=False)
    args = parser.parse_args()

    eval_odom(args.pred, args.gt, args.radar, args.sonar, args.align_timestamps)
    if args.velocity is not None:
        eval_odom_vel(args.velocity, args.gt, args.radar, args.sonar, args.align_timestamps)

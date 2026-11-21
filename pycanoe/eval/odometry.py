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


def eval_odom(pred="test/demo/pred/3d", gt="test/demo/gt", radar=False, sonar=False):
    # evaluation mode
    dim = 2 if radar else 3

    # parse sequences
    seq = get_sequences(pred, ".txt")
    T_pred, _, seq_lens_pred = get_sequence_poses(pred, seq)

    # For sequences where there are extractor specific parameters in the name
    # Ex: 'canoe-2025-08-21-19-16_test1.txt' -> 'canoe-2025-08-21-19-16.txt'
    seq_gt = []
    for s in seq:
        curr_seq = s.split('_')[0]
        if ".txt" != curr_seq[-4:]: 
            curr_seq=curr_seq+".txt"
        seq_gt.append(curr_seq)

    # get corresponding groundtruth poses
    T_gt, _, seq_lens_gt, crop = get_sequence_poses_gt(gt, seq_gt, dim, sonar)

    # compute errors
    t_err, r_err, _ = compute_kitti_metrics(
        T_gt, T_pred, seq_lens_gt, seq_lens_pred, seq, pred, dim, crop
    )

    # print out results
    print("Evaluated sequences: ", seq)
    print("Overall error: ", t_err, " %, ", r_err, " deg/m")

    return t_err, r_err

def eval_odom_vel(pred="test/demo/pred/3d", gt="test/demo/gt", radar=False, sonar=False):
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
    vel_gt, _, seq_lens_gt, crop = get_sequence_velocities_gt(gt, seq_gt, dim, sonar)

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

    parser.set_defaults(radar=False)
    args = parser.parse_args()

    eval_odom(args.pred, args.gt, args.radar, args.sonar)
    if args.velocity is not None:
        eval_odom_vel(args.velocity, args.gt, args.radar, args.sonar)

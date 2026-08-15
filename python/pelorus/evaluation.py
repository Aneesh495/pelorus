"""
Trajectory evaluation toolkit: Absolute Trajectory Error (ATE) and Relative Pose Error (RPE).
Implements Horn/Umeyama closed-form rigid alignment and SE(3) error statistics.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import numpy as np


@dataclass
class TrajectoryMetrics:
    rmse: float
    mean: float
    median: float
    std: float
    min_err: float
    max_err: float
    error_values: np.ndarray


def align_umeyama(model: np.ndarray, data: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    """
    Compute rigid transformation [R, t, s] aligning data to model using Umeyama algorithm.
    model, data: arrays of shape (N, 3).
    Returns R (3, 3), t (3,), scale (float).
    """
    n = model.shape[0]
    if n != data.shape[0] or n < 3:
        raise ValueError("Umeyama alignment requires at least 3 corresponding points.")

    mu_m = np.mean(model, axis=0)
    mu_d = np.mean(data, axis=0)

    model_centered = model - mu_m
    data_centered = data - mu_d

    sigma_d_sq = np.sum(data_centered ** 2) / n

    # Cross-covariance matrix H = data^T * model
    H = (data_centered.T @ model_centered) / n

    U, S, Vt = np.linalg.svd(H)

    # Reflection correction
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1.0, 1.0, 1.0 if d >= 0 else -1.0])

    R = Vt.T @ D @ U.T
    scale = 1.0 # Rigid SE(3) alignment without scale dilation
    t = mu_m - scale * (R @ mu_d)

    return R, t, scale


def evaluate_ate(
    estimated_traj: np.ndarray,
    ground_truth_traj: np.ndarray,
    align: bool = True,
) -> TrajectoryMetrics:
    """
    Compute Absolute Trajectory Error (ATE) between estimated and ground truth positions.
    Positions are arrays of shape (N, 3).
    """
    n = min(len(estimated_traj), len(ground_truth_traj))
    est = estimated_traj[:n]
    gt = ground_truth_traj[:n]

    if align:
        R, t, _ = align_umeyama(gt, est)
        aligned_est = (R @ est.T).T + t
    else:
        aligned_est = est

    errors = np.linalg.norm(aligned_est - gt, axis=1)

    return TrajectoryMetrics(
        rmse=float(np.sqrt(np.mean(errors ** 2))),
        mean=float(np.mean(errors)),
        median=float(np.median(errors)),
        std=float(np.std(errors)),
        min_err=float(np.min(errors)),
        max_err=float(np.max(errors)),
        error_values=errors,
    )


def evaluate_rpe(
    estimated_traj: np.ndarray,
    ground_truth_traj: np.ndarray,
    delta_frames: int = 1,
) -> TrajectoryMetrics:
    """
    Compute Relative Pose Error (RPE) drift per delta_frames interval.
    """
    n = min(len(estimated_traj), len(ground_truth_traj))
    if n <= delta_frames:
        raise ValueError("Trajectory shorter than relative delta frames.")

    errors = []
    for i in range(n - delta_frames):
        delta_est = estimated_traj[i + delta_frames] - estimated_traj[i]
        delta_gt = ground_truth_traj[i + delta_frames] - ground_truth_traj[i]
        err = np.linalg.norm(delta_est - delta_gt)
        errors.append(err)

    arr = np.array(errors)
    return TrajectoryMetrics(
        rmse=float(np.sqrt(np.mean(arr ** 2))),
        mean=float(np.mean(arr)),
        median=float(np.median(arr)),
        std=float(np.std(arr)),
        min_err=float(np.min(arr)),
        max_err=float(np.max(arr)),
        error_values=arr,
    )

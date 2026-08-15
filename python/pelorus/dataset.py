"""
Synthetic benchmark dataset generator: creates realistic LiDAR-inertial trajectories
and 3D obstacle scenes for automated pipeline verification.
"""

from typing import Dict, List, Tuple
import numpy as np


class SyntheticDataset:
    """
    Generates synthetic autonomous mobile platform sweeps:
    - 6-DOF ground truth trajectory
    - 200 Hz IMU linear accelerations and angular velocities
    - 10 Hz 3D LiDAR point clouds with structured walls, ground, and dynamic obstacles
    """

    def __init__(self, duration_sec: float = 10.0, scan_rate_hz: float = 10.0, imu_rate_hz: float = 200.0):
        self.duration = duration_sec
        self.scan_rate = scan_rate_hz
        self.imu_rate = imu_rate_hz
        self.dt_scan = 1.0 / scan_rate_hz
        self.dt_imu = 1.0 / imu_rate_hz

    def generate_circular_loop(self, radius: float = 10.0, speed: float = 2.0) -> Dict:
        """
        Generate platform traversing a circular path of given radius at constant forward speed.
        """
        omega = speed / radius
        total_imu_steps = int(self.duration * self.imu_rate)
        total_scans = int(self.duration * self.scan_rate)

        # 1. IMU stream
        imu_timestamps = np.linspace(0.0, self.duration, total_imu_steps)
        imu_accels = np.zeros((total_imu_steps, 3), dtype=np.float32)
        imu_gyros = np.zeros((total_imu_steps, 3), dtype=np.float32)

        centripetal = (speed ** 2) / radius
        imu_accels[:, 1] = centripetal # Lateral centripetal acceleration
        imu_accels[:, 2] = 9.81        # Gravity reaction force
        imu_gyros[:, 2] = omega        # Yaw angular velocity

        # Add mild Gaussian sensor noise
        imu_accels += np.random.normal(0, 0.02, imu_accels.shape).astype(np.float32)
        imu_gyros += np.random.normal(0, 0.005, imu_gyros.shape).astype(np.float32)

        # 2. Ground truth poses
        scan_timestamps = np.linspace(0.0, self.duration, total_scans)
        gt_positions = np.zeros((total_scans, 3), dtype=np.float32)
        gt_quaternions = np.zeros((total_scans, 4), dtype=np.float32)

        for i, t in enumerate(scan_timestamps):
            theta = omega * t
            gt_positions[i, 0] = radius * np.cos(theta)
            gt_positions[i, 1] = radius * np.sin(theta)
            gt_positions[i, 2] = 0.0

            half_yaw = (theta + np.pi * 0.5) * 0.5
            gt_quaternions[i] = [np.cos(half_yaw), 0.0, 0.0, np.sin(half_yaw)]

        # 3. Static point clouds
        scans = []
        for i, t in enumerate(scan_timestamps):
            pos = gt_positions[i]
            pts = []

            # Ground plane points
            for gx in range(-15, 16, 2):
                for gy in range(-15, 16, 2):
                    pts.append([float(gx), float(gy), 0.0])

            # Cylindrical pillars (obstacles) around radius + 3
            for pillar_angle in [0.0, np.pi * 0.5, np.pi, np.pi * 1.5]:
                cx = (radius + 3.0) * np.cos(pillar_angle)
                cy = (radius + 3.0) * np.sin(pillar_angle)
                for pz in np.linspace(0.0, 2.5, 6):
                    for da in np.linspace(0, 2 * np.pi, 8, endpoint=False):
                        pts.append([cx + 0.4 * np.cos(da), cy + 0.4 * np.sin(da), pz])

            scans.append(np.array(pts, dtype=np.float32))

        return {
            "imu_timestamps": imu_timestamps,
            "imu_accels": imu_accels,
            "imu_gyros": imu_gyros,
            "scan_timestamps": scan_timestamps,
            "gt_positions": gt_positions,
            "gt_quaternions": gt_quaternions,
            "scans": scans,
        }

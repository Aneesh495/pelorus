"""
Pelorus Python SDK: Zero-overhead ctypes bindings to native libpelorus.dylib.
"""

import ctypes
import os
import sys
from typing import List, Optional, Tuple
import numpy as np

# Locate shared library
_LIB_DIRS = [
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "lib")),
    os.path.abspath(os.path.join(os.path.dirname(__file__), "lib")),
    "/usr/local/lib",
    "/usr/lib",
]

_LIB_PATH = None
for d in _LIB_DIRS:
    candidate = os.path.join(d, "libpelorus.dylib")
    if os.path.exists(candidate):
        _LIB_PATH = candidate
        break

if _LIB_PATH is None:
    # Try finding anywhere in repo
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    candidate = os.path.join(repo_root, "lib", "libpelorus.dylib")
    if os.path.exists(candidate):
        _LIB_PATH = candidate

try:
    if _LIB_PATH and os.path.exists(_LIB_PATH):
        _lib = ctypes.CDLL(_LIB_PATH)
    else:
        _lib = None
except OSError as e:
    _lib = None


if _lib is not None:
    _lib.pelorus_get_version.restype = ctypes.c_char_p
    _lib.pelorus_get_version.argtypes = []

    _lib.pelorus_engine_create.restype = ctypes.c_void_p
    _lib.pelorus_engine_create.argtypes = [ctypes.c_float]

    _lib.pelorus_engine_destroy.restype = None
    _lib.pelorus_engine_destroy.argtypes = [ctypes.c_void_p]

    _lib.pelorus_engine_push_imu.restype = None
    _lib.pelorus_engine_push_imu.argtypes = [
        ctypes.c_void_p,
        ctypes.c_float, ctypes.c_float, ctypes.c_float,
        ctypes.c_float, ctypes.c_float, ctypes.c_float,
        ctypes.c_double,
    ]

    _lib.pelorus_engine_process_points.restype = ctypes.c_int32
    _lib.pelorus_engine_process_points.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_int32,
        ctypes.c_double,
        ctypes.POINTER(ctypes.c_float),
    ]

    _lib.pelorus_engine_plan_trajectory.restype = ctypes.c_int32
    _lib.pelorus_engine_plan_trajectory.argtypes = [
        ctypes.c_void_p,
        ctypes.c_float, ctypes.c_float, ctypes.c_float,
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_int32,
    ]

    _lib.pelorus_engine_get_obstacle_count.restype = ctypes.c_int32
    _lib.pelorus_engine_get_obstacle_count.argtypes = [ctypes.c_void_p]


def get_version() -> str:
    if _lib is None:
        return "0.8.0-fallback"
    return _lib.pelorus_get_version().decode("utf-8")


class PelorusEngine:
    """
    High-level Python wrapper for Pelorus real-time spatial perception engine.
    """

    def __init__(self, voxel_size: float = 0.1):
        if _lib is None:
            raise RuntimeError(f"Native library libpelorus.dylib not found. Run 'odin build src/c_api -build-mode:shared -out:lib/libpelorus.dylib' first.")
        self._handle = _lib.pelorus_engine_create(ctypes.c_float(voxel_size))
        if not self._handle:
            raise RuntimeError("Failed to allocate PelorusEngine native instance.")

    def __del__(self):
        if hasattr(self, "_handle") and self._handle and _lib is not None:
            _lib.pelorus_engine_destroy(self._handle)
            self._handle = None

    def push_imu(self, accel: Tuple[float, float, float], gyro: Tuple[float, float, float], timestamp: float) -> None:
        """Push single IMU measurement (acceleration m/s^2, angular velocity rad/s)."""
        _lib.pelorus_engine_push_imu(
            self._handle,
            accel[0], accel[1], accel[2],
            gyro[0], gyro[1], gyro[2],
            timestamp,
        )

    def process_point_cloud(self, points: np.ndarray, timestamp: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Ingest LiDAR point cloud array of shape (N, 3), return (translation_3d, quaternion_wxyz).
        """
        points = np.ascontiguousarray(points, dtype=np.float32)
        n_pts = points.shape[0]

        out_pose = np.zeros(7, dtype=np.float32)
        pts_ptr = points.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        pose_ptr = out_pose.ctypes.data_as(ctypes.POINTER(ctypes.c_float))

        res = _lib.pelorus_engine_process_points(self._handle, pts_ptr, n_pts, timestamp, pose_ptr)
        if res != 0:
            raise RuntimeError(f"Engine point processing failed with error code: {res}")

        translation = out_pose[:3]
        quaternion = np.array([out_pose[6], out_pose[3], out_pose[4], out_pose[5]], dtype=np.float32) # [w, x, y, z]
        return translation, quaternion

    def plan_trajectory(self, goal_xyz: Tuple[float, float, float], max_waypoints: int = 20) -> np.ndarray:
        """
        Compute collision-free kinodynamic minimum-jerk trajectory to target 3D goal.
        Returns array of waypoints of shape (M, 3).
        """
        buf = np.zeros(max_waypoints * 3, dtype=np.float32)
        buf_ptr = buf.ctypes.data_as(ctypes.POINTER(ctypes.c_float))

        count = _lib.pelorus_engine_plan_trajectory(
            self._handle,
            goal_xyz[0], goal_xyz[1], goal_xyz[2],
            buf_ptr,
            max_waypoints,
        )

        if count <= 0:
            return np.empty((0, 3), dtype=np.float32)

        return buf[:count * 3].reshape((count, 3))

    def get_obstacle_count(self) -> int:
        """Return the number of dynamic 3D obstacles currently tracked."""
        return int(_lib.pelorus_engine_get_obstacle_count(self._handle))

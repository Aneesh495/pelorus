"""
Pelorus: High-Performance Real-Time Spatial Perception & Kinodynamic Planning Engine.
"""

from .bindings import PelorusEngine, get_version
from .evaluation import TrajectoryMetrics, evaluate_ate, evaluate_rpe, align_umeyama
from .dataset import SyntheticDataset

__version__ = "0.8.0"
__all__ = [
    "PelorusEngine",
    "get_version",
    "TrajectoryMetrics",
    "evaluate_ate",
    "evaluate_rpe",
    "align_umeyama",
    "SyntheticDataset",
]

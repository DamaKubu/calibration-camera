"""Bearing estimation (direction-of-arrival) from tracked centroids.

Core geometry per prompt:
- Undistort centroid only (not full image)
- Pixel -> normalized camera ray via intrinsics
- Camera ray -> world ray via camera rotation
- Output unit bearing vector
- Temporal smoothing + stability metric

Calibration formats supported:
- OpenCV YAML from your repo (camera_matrix/distortion_coefficients or K/D)
- Stereo YAML with R/T (we use R to relate cam2 -> cam1/world)

World frame convention:
- If stereo YAML is used, cam1 is treated as world frame.
- If no rotation is provided, identity is used.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import math
import re

import cv2
import numpy as np


def _parse_opencv_matrix_block(text: str, key: str) -> Optional[np.ndarray]:
    # Matches:
    # key: !!opencv-matrix
    #    rows: r
    #    cols: c
    #    dt: d
    #    data: [ ... ]
    m = re.search(rf"{re.escape(key)}:\s*!!opencv-matrix\s+rows:\s*(\d+)\s+cols:\s*(\d+).*?data:\s*\[(.*?)\]", text, re.S)
    if not m:
        return None
    rows = int(m.group(1))
    cols = int(m.group(2))
    data_str = m.group(3)
    nums = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", data_str)]
    if len(nums) != rows * cols:
        return None
    return np.array(nums, dtype=np.float64).reshape((rows, cols))


def load_intrinsics_from_yaml(path: str) -> Tuple[np.ndarray, np.ndarray, Tuple[int, int]]:
    """Load K, D, (w,h) from an OpenCV-style intrinsic YAML."""
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        text = f.read()

    K = _parse_opencv_matrix_block(text, "camera_matrix")
    if K is None:
        K = _parse_opencv_matrix_block(text, "K")
    if K is None:
        raise ValueError(f"Could not find camera matrix in {path}")

    D = _parse_opencv_matrix_block(text, "distortion_coefficients")
    if D is None:
        D = _parse_opencv_matrix_block(text, "D")
    if D is None:
        D = np.zeros((5, 1), dtype=np.float64)

    # Try to read width/height if present
    w = 0
    h = 0
    mw = re.search(r"image_width:\s*(\d+)", text)
    mh = re.search(r"image_height:\s*(\d+)", text)
    if mw and mh:
        w = int(mw.group(1))
        h = int(mh.group(1))

    return K.astype(np.float64), D.astype(np.float64), (w, h)


def load_stereo_rotation_from_yaml(path: str) -> np.ndarray:
    """Load stereo rotation R (cam1->cam2) from a stereo YAML."""
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        text = f.read()
    R = _parse_opencv_matrix_block(text, "R")
    if R is None or R.shape != (3, 3):
        raise ValueError(f"Could not find 3x3 R in {path}")
    return R.astype(np.float64)


def unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n <= 0:
        return v
    return v / n


def angle_between_unit(u: np.ndarray, v: np.ndarray) -> float:
    c = float(np.clip(np.dot(u, v), -1.0, 1.0))
    return float(math.acos(c))


@dataclass
class BearingParams:
    ema_alpha: float = 0.25
    stable_window: int = 12
    stable_max_mean_deg: float = 0.35


@dataclass
class BearingState:
    bearing_world: Optional[np.ndarray] = None  # (3,)
    recent_angles_deg: list[float] = field(default_factory=list)


class BearingEstimator:
    def __init__(
        self,
        K: np.ndarray,
        D: np.ndarray,
        R_world_from_cam: Optional[np.ndarray] = None,
        params: BearingParams | None = None,
    ):
        self.K = K.astype(np.float64)
        self.D = D.astype(np.float64)
        self.R_wc = (np.eye(3, dtype=np.float64) if R_world_from_cam is None else R_world_from_cam.astype(np.float64))
        self.params = params or BearingParams()
        self._states: Dict[int, BearingState] = {}

    def reset(self) -> None:
        self._states.clear()

    def estimate(self, track_id: int, centroid_xy: Tuple[float, float]) -> Tuple[np.ndarray, bool, float]:
        """Return (bearing_world_unit, is_stable, stability_metric_deg)."""

        x, y = float(centroid_xy[0]), float(centroid_xy[1])

        # cv2.undistortPoints expects shape (N,1,2)
        pts = np.array([[[x, y]]], dtype=np.float64)
        und = cv2.undistortPoints(pts, self.K, self.D, P=None)
        xn = float(und[0, 0, 0])
        yn = float(und[0, 0, 1])

        ray_c = unit(np.array([xn, yn, 1.0], dtype=np.float64))
        ray_w = unit(self.R_wc @ ray_c)

        st = self._states.get(track_id)
        if st is None:
            st = BearingState(bearing_world=ray_w.copy())
            self._states[track_id] = st
            return ray_w, False, 999.0

        prev = st.bearing_world
        assert prev is not None

        # Angular change for stability metric
        ang_deg = math.degrees(angle_between_unit(prev, ray_w))
        st.recent_angles_deg.append(float(ang_deg))
        if len(st.recent_angles_deg) > self.params.stable_window:
            st.recent_angles_deg = st.recent_angles_deg[-self.params.stable_window :]

        # EMA smoothing in vector space (then renormalize)
        a = float(self.params.ema_alpha)
        smoothed = unit((1.0 - a) * prev + a * ray_w)
        st.bearing_world = smoothed

        mean_deg = float(np.mean(st.recent_angles_deg)) if st.recent_angles_deg else 999.0
        is_stable = (len(st.recent_angles_deg) >= self.params.stable_window) and (mean_deg <= self.params.stable_max_mean_deg)
        return smoothed, is_stable, mean_deg


def rotation_world_from_cam2_using_stereo_R(R_cam2_from_cam1: np.ndarray) -> np.ndarray:
    """Given stereo R that maps cam1 vectors into cam2 (X2 = R * X1 + T),
    return rotation that maps cam2 vectors into cam1/world.
    """

    return R_cam2_from_cam1.T.copy()

"""Two-camera fusion by bearing agreement only (no triangulation).

Per prompt:
- Each camera runs independently
- Fusion is done by time alignment (tolerance) + angular agreement
- Output bearings from each camera + confidence

This does NOT try to estimate depth/position.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import math
import numpy as np


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v if n <= 0 else v / n


def angular_error_deg(u: np.ndarray, v: np.ndarray) -> float:
    u = _unit(u)
    v = _unit(v)
    c = float(np.clip(np.dot(u, v), -1.0, 1.0))
    return float(math.degrees(math.acos(c)))


@dataclass
class BearingObs:
    ts_s: float
    track_id: int
    bearing_w: np.ndarray  # (3,)
    per_cam_conf: float


@dataclass
class FusionParams:
    time_tolerance_s: float = 0.12
    max_angle_deg: float = 2.5


@dataclass
class FusedPair:
    ts_s: float
    cam1_track_id: int
    cam2_track_id: int
    bearing_cam1_w: np.ndarray
    bearing_cam2_w: np.ndarray
    angle_deg: float
    confidence: float


class BearingFuser:
    def __init__(self, params: FusionParams | None = None):
        self.params = params or FusionParams()
        self._recent_cam1: List[BearingObs] = []
        self._recent_cam2: List[BearingObs] = []

    def reset(self) -> None:
        self._recent_cam1.clear()
        self._recent_cam2.clear()

    def push_cam1(self, obs: BearingObs) -> None:
        self._recent_cam1.append(obs)
        self._prune(obs.ts_s)

    def push_cam2(self, obs: BearingObs) -> None:
        self._recent_cam2.append(obs)
        self._prune(obs.ts_s)

    def _prune(self, now_s: float) -> None:
        ttl = max(0.5, 3.0 * self.params.time_tolerance_s)
        self._recent_cam1 = [o for o in self._recent_cam1 if now_s - o.ts_s <= ttl]
        self._recent_cam2 = [o for o in self._recent_cam2 if now_s - o.ts_s <= ttl]

    def fuse_latest(self) -> Optional[FusedPair]:
        """Try to fuse the newest observation from whichever camera updated last."""
        if not self._recent_cam1 or not self._recent_cam2:
            return None

        o1 = self._recent_cam1[-1]
        o2 = self._recent_cam2[-1]
        anchor = o1 if o1.ts_s >= o2.ts_s else o2

        if anchor is o1:
            candidates = [o for o in self._recent_cam2 if abs(o.ts_s - o1.ts_s) <= self.params.time_tolerance_s]
            if not candidates:
                return None
            best = min(candidates, key=lambda o: angular_error_deg(o1.bearing_w, o.bearing_w))
            angle = angular_error_deg(o1.bearing_w, best.bearing_w)
            if angle > self.params.max_angle_deg:
                return None
            conf = float(0.5 * (o1.per_cam_conf + best.per_cam_conf) * max(0.0, 1.0 - angle / self.params.max_angle_deg))
            return FusedPair(
                ts_s=max(o1.ts_s, best.ts_s),
                cam1_track_id=o1.track_id,
                cam2_track_id=best.track_id,
                bearing_cam1_w=o1.bearing_w,
                bearing_cam2_w=best.bearing_w,
                angle_deg=angle,
                confidence=conf,
            )

        candidates = [o for o in self._recent_cam1 if abs(o.ts_s - o2.ts_s) <= self.params.time_tolerance_s]
        if not candidates:
            return None
        best = min(candidates, key=lambda o: angular_error_deg(o2.bearing_w, o.bearing_w))
        angle = angular_error_deg(o2.bearing_w, best.bearing_w)
        if angle > self.params.max_angle_deg:
            return None
        conf = float(0.5 * (o2.per_cam_conf + best.per_cam_conf) * max(0.0, 1.0 - angle / self.params.max_angle_deg))
        return FusedPair(
            ts_s=max(o2.ts_s, best.ts_s),
            cam1_track_id=best.track_id,
            cam2_track_id=o2.track_id,
            bearing_cam1_w=best.bearing_w,
            bearing_cam2_w=o2.bearing_w,
            angle_deg=angle,
            confidence=conf,
        )

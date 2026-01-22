"""SORT-style multi-object tracker.

Key requirements implemented:
- Constant-velocity Kalman filter: state (x, y, vx, vy)
- Hungarian assignment on centroid distance
- Track lifecycle: min_hits (age before valid), max_age (miss tolerance)
- Track history: positions, velocities, bbox areas

This is intentionally lightweight and dependency-free (no SciPy).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np


def _hungarian(cost: np.ndarray) -> List[Tuple[int, int]]:
    """Solve assignment with Hungarian algorithm (Munkres), square cost matrix.

    Returns list of (row, col) pairs.

    Notes:
    - Complexity O(n^3); fine for small numbers of blobs.
    - This implementation assumes finite costs.
    """

    cost = np.asarray(cost, dtype=np.float64)
    if cost.ndim != 2 or cost.shape[0] != cost.shape[1]:
        raise ValueError("Hungarian expects square cost matrix")

    n = cost.shape[0]
    u = np.zeros(n + 1)
    v = np.zeros(n + 1)
    p = np.zeros(n + 1, dtype=int)
    way = np.zeros(n + 1, dtype=int)

    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = np.full(n + 1, np.inf)
        used = np.zeros(n + 1, dtype=bool)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = np.inf
            j1 = 0
            for j in range(1, n + 1):
                if used[j]:
                    continue
                cur = cost[i0 - 1, j - 1] - u[i0] - v[j]
                if cur < minv[j]:
                    minv[j] = cur
                    way[j] = j0
                if minv[j] < delta:
                    delta = minv[j]
                    j1 = j
            for j in range(0, n + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break

    assignment = [(p[j] - 1, j - 1) for j in range(1, n + 1)]
    return assignment


@dataclass
class TrackerParams:
    max_age: int = 10
    min_hits: int = 3
    dist_gate_px: float = 80.0
    process_noise_pos: float = 20.0
    process_noise_vel: float = 50.0
    meas_noise_pos: float = 15.0


@dataclass
class Track:
    id: int
    x: np.ndarray  # (4,)
    P: np.ndarray  # (4,4)
    age: int = 1
    hits: int = 0
    time_since_update: int = 0
    last_ts_s: float = 0.0

    bbox_xywh: Tuple[int, int, int, int] = (0, 0, 0, 0)
    area: float = 0.0

    positions_xy: List[Tuple[float, float]] = field(default_factory=list)
    velocities_xy: List[Tuple[float, float]] = field(default_factory=list)
    areas: List[float] = field(default_factory=list)

    def is_confirmed(self, params: TrackerParams) -> bool:
        return self.hits >= params.min_hits

    def centroid_xy(self) -> Tuple[float, float]:
        return float(self.x[0]), float(self.x[1])

    def velocity_xy(self) -> Tuple[float, float]:
        return float(self.x[2]), float(self.x[3])


class SortTracker:
    def __init__(self, params: TrackerParams | None = None):
        self.params = params or TrackerParams()
        self._next_id = 1
        self._tracks: Dict[int, Track] = {}

    @property
    def tracks(self) -> List[Track]:
        return list(self._tracks.values())

    def reset(self) -> None:
        self._next_id = 1
        self._tracks.clear()

    def _kf_predict(self, track: Track, dt: float) -> None:
        F = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

        qpos = float(self.params.process_noise_pos)
        qvel = float(self.params.process_noise_vel)
        Q = np.diag([qpos * qpos, qpos * qpos, qvel * qvel, qvel * qvel]).astype(np.float64)

        track.x = F @ track.x
        track.P = F @ track.P @ F.T + Q

    def _kf_update(self, track: Track, z_xy: np.ndarray) -> None:
        H = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
            ],
            dtype=np.float64,
        )
        r = float(self.params.meas_noise_pos)
        R = np.diag([r * r, r * r]).astype(np.float64)

        y = z_xy - (H @ track.x)
        S = H @ track.P @ H.T + R
        K = track.P @ H.T @ np.linalg.inv(S)
        track.x = track.x + K @ y
        I = np.eye(4)
        track.P = (I - K @ H) @ track.P

    def update(
        self,
        detections_xy: np.ndarray,
        detections_bbox_xywh: List[Tuple[int, int, int, int]],
        detections_area: np.ndarray,
        ts_s: float,
    ) -> List[Track]:
        """Update tracker.

        detections_xy: (N,2) float
        detections_bbox_xywh: list len N
        detections_area: (N,) float
        """

        if detections_xy.ndim != 2 or detections_xy.shape[1] != 2:
            raise ValueError("detections_xy must be (N,2)")

        track_ids = sorted(self._tracks.keys())
        tracks = [self._tracks[tid] for tid in track_ids]

        # Predict
        for trk in tracks:
            dt = max(1e-3, float(ts_s - trk.last_ts_s)) if trk.last_ts_s > 0 else (1.0 / 30.0)
            self._kf_predict(trk, dt)
            trk.age += 1
            trk.time_since_update += 1

        n_trk = len(tracks)
        n_det = int(detections_xy.shape[0])

        if n_trk == 0:
            for i in range(n_det):
                self._start_track(detections_xy[i], detections_bbox_xywh[i], float(detections_area[i]), ts_s)
            return self.tracks

        # Build square cost matrix by padding with dummy rows/cols
        n = max(n_trk, n_det)
        big = 1e6
        cost = np.full((n, n), big, dtype=np.float64)

        for i, trk in enumerate(tracks):
            tx, ty = trk.centroid_xy()
            for j in range(n_det):
                dx, dy = detections_xy[j]
                dist = float(np.hypot(dx - tx, dy - ty))
                if dist <= self.params.dist_gate_px:
                    cost[i, j] = dist

        assignment = _hungarian(cost)

        matched_trk = set()
        matched_det = set()

        for i, j in assignment:
            if i < n_trk and j < n_det:
                if cost[i, j] >= big:
                    continue
                trk = tracks[i]
                z = detections_xy[j].astype(np.float64)
                self._kf_update(trk, z)

                trk.time_since_update = 0
                trk.hits += 1
                trk.last_ts_s = ts_s

                trk.bbox_xywh = detections_bbox_xywh[j]
                trk.area = float(detections_area[j])

                vx, vy = trk.velocity_xy()
                trk.positions_xy.append(trk.centroid_xy())
                trk.velocities_xy.append((vx, vy))
                trk.areas.append(trk.area)

                matched_trk.add(trk.id)
                matched_det.add(j)

        # Start new tracks for unmatched detections
        for j in range(n_det):
            if j in matched_det:
                continue
            self._start_track(detections_xy[j], detections_bbox_xywh[j], float(detections_area[j]), ts_s)

        # Prune old tracks
        dead = []
        for tid, trk in self._tracks.items():
            if trk.time_since_update > self.params.max_age:
                dead.append(tid)
        for tid in dead:
            del self._tracks[tid]

        return self.tracks

    def _start_track(self, det_xy: np.ndarray, bbox_xywh: Tuple[int, int, int, int], area: float, ts_s: float) -> None:
        x = np.array([float(det_xy[0]), float(det_xy[1]), 0.0, 0.0], dtype=np.float64)
        P = np.diag([200.0, 200.0, 500.0, 500.0]).astype(np.float64)
        tid = self._next_id
        self._next_id += 1
        trk = Track(id=tid, x=x, P=P, hits=1, time_since_update=0, last_ts_s=ts_s, bbox_xywh=bbox_xywh, area=area)
        trk.positions_xy.append((float(det_xy[0]), float(det_xy[1])))
        trk.velocities_xy.append((0.0, 0.0))
        trk.areas.append(float(area))
        self._tracks[tid] = trk

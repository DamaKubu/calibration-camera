"""Motion-based candidate generation for small flying objects.

Design goals:
- Luminance-only (expects 8-bit Y image)
- Robust enough for cheap USB cameras
- Simple knobs, predictable behavior

This module does NOT do CNN detection; it produces candidate blobs based on motion.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class MotionBlob:
    bbox_xywh: Tuple[int, int, int, int]
    centroid_xy: Tuple[float, float]
    area: int
    aspect: float
    speed_px_s: float


@dataclass
class MotionParams:
    blur_sigma: float = 1.0
    bg_alpha: float = 0.02
    diff_thresh: int = 20
    min_area: int = 8
    max_area: int = 600
    min_aspect: float = 0.2
    max_aspect: float = 5.0
    morph_open: int = 3
    morph_close: int = 5
    min_blob_speed_px_s: float = 15.0
    max_blob_speed_px_s: float = 2500.0


class MotionDetector:
    """Motion blob detector using running-average background + frame differencing.

    Also estimates blob centroid speed by matching to previous-frame blobs
    (nearest centroid, within a gate). This is only used to reject near-static blobs.
    """

    def __init__(self, params: MotionParams | None = None):
        self.params = params or MotionParams()
        self._bg: Optional[np.ndarray] = None
        self._prev_blobs_xy: Optional[np.ndarray] = None  # shape (N,2)
        self._prev_ts: Optional[float] = None

    def reset(self) -> None:
        self._bg = None
        self._prev_blobs_xy = None
        self._prev_ts = None

    def _preprocess_y(self, y: np.ndarray) -> np.ndarray:
        if y.ndim != 2 or y.dtype != np.uint8:
            raise ValueError("Expected Y plane as uint8 HxW")

        if self.params.blur_sigma > 0:
            # Choose kernel size from sigma; keep odd.
            k = int(max(3, 2 * round(3 * self.params.blur_sigma) + 1))
            y = cv2.GaussianBlur(y, (k, k), self.params.blur_sigma)
        return y

    def detect(self, y: np.ndarray, ts_s: float) -> List[MotionBlob]:
        y = self._preprocess_y(y)

        if self._bg is None:
            self._bg = y.astype(np.float32)
            self._prev_blobs_xy = None
            self._prev_ts = ts_s
            return []

        cv2.accumulateWeighted(y, self._bg, self.params.bg_alpha)
        bg_u8 = cv2.convertScaleAbs(self._bg)

        diff = cv2.absdiff(y, bg_u8)
        _, mask = cv2.threshold(diff, self.params.diff_thresh, 255, cv2.THRESH_BINARY)

        if self.params.morph_open > 0:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.params.morph_open, self.params.morph_open))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        if self.params.morph_close > 0:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.params.morph_close, self.params.morph_close))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        n, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

        dt = 1e-3
        if self._prev_ts is not None:
            dt = max(1e-3, float(ts_s - self._prev_ts))

        blobs: List[MotionBlob] = []
        curr_xy = []

        # Build previous centroid array for speed estimate
        prev_xy = self._prev_blobs_xy

        for i in range(1, n):
            x, y0, w, h, area = stats[i]
            if area < self.params.min_area or area > self.params.max_area:
                continue

            aspect = float(w) / float(max(1, h))
            if aspect < self.params.min_aspect or aspect > self.params.max_aspect:
                continue

            cx, cy = centroids[i]
            curr_xy.append((cx, cy))

            speed = 0.0
            if prev_xy is not None and prev_xy.size > 0:
                d = prev_xy - np.array([[cx, cy]], dtype=np.float32)
                dist = float(np.sqrt(np.min(np.sum(d * d, axis=1))))
                speed = dist / dt

            # Only enforce speed gate when we have a previous frame's blob set.
            # (On warmup / after long gaps, this prevents rejecting everything.)
            if prev_xy is not None and prev_xy.size > 0:
                if speed < self.params.min_blob_speed_px_s or speed > self.params.max_blob_speed_px_s:
                    # Reject almost-static components (e.g., sensor noise) and pathological jumps
                    continue

            blobs.append(
                MotionBlob(
                    bbox_xywh=(int(x), int(y0), int(w), int(h)),
                    centroid_xy=(float(cx), float(cy)),
                    area=int(area),
                    aspect=float(aspect),
                    speed_px_s=float(speed),
                )
            )

        self._prev_blobs_xy = np.array(curr_xy, dtype=np.float32) if curr_xy else None
        self._prev_ts = ts_s
        return blobs

    @staticmethod
    def debug_render(mask: np.ndarray) -> np.ndarray:
        """Render a motion mask for debugging (BGR)."""
        return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

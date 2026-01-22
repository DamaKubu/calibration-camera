"""Track-level drone vs bird discrimination.

Per prompt:
- Use motion-based features (velocity/acceleration/area variance)
- Do NOT rely on per-frame CNN
- Output confidence score in [0,1] and smooth it over time

This is a demo-grade heuristic classifier designed for stability.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np

from .tracking import Track


def _sigmoid(x: float) -> float:
    return float(1.0 / (1.0 + np.exp(-x)))


@dataclass
class ClassifierParams:
    # Windows for computing variances
    window: int = 25

    # Heuristic scaling for variance normalization
    vel_std_ref: float = 120.0
    acc_std_ref: float = 450.0
    area_std_ref: float = 80.0

    # Score smoothing (EMA)
    score_alpha: float = 0.15


class MotionClassifier:
    """Outputs a 'drone confidence' score (1=drone-like, 0=bird-like)."""

    def __init__(self, params: ClassifierParams | None = None):
        self.params = params or ClassifierParams()
        self._score_ema: Dict[int, float] = {}

    def reset(self) -> None:
        self._score_ema.clear()

    def score_track(self, trk: Track) -> Tuple[float, str, dict]:
        w = int(self.params.window)

        v = np.array(trk.velocities_xy[-w:], dtype=np.float64)
        a = np.array(trk.areas[-w:], dtype=np.float64)

        if v.shape[0] < max(5, w // 3):
            s = self._score_ema.get(trk.id, 0.5)
            return float(s), ("drone" if s >= 0.5 else "bird"), {"warmup": True}

        speed = np.linalg.norm(v, axis=1)
        acc = np.diff(v, axis=0)
        acc_mag = np.linalg.norm(acc, axis=1) if acc.shape[0] > 0 else np.zeros((0,), dtype=np.float64)

        vel_std = float(np.std(speed))
        acc_std = float(np.std(acc_mag)) if acc_mag.size > 0 else 0.0
        area_std = float(np.std(a)) if a.size > 0 else 0.0

        # Bird-ish: high acceleration variance (flapping/oscillation), size changes
        # Drone-ish: smoother acceleration, steadier size
        vel_term = 1.0 - min(1.0, vel_std / max(1e-6, self.params.vel_std_ref))
        acc_term = 1.0 - min(1.0, acc_std / max(1e-6, self.params.acc_std_ref))
        area_term = 1.0 - min(1.0, area_std / max(1e-6, self.params.area_std_ref))

        # Mild preference for smoothness: emphasize acceleration and area stability
        raw = 0.20 * vel_term + 0.55 * acc_term + 0.25 * area_term

        # Map to [0,1] with gentle slope; center around 0.5
        score = _sigmoid((raw - 0.5) * 6.0)

        prev = self._score_ema.get(trk.id, score)
        a_ema = float(self.params.score_alpha)
        score_ema = float((1.0 - a_ema) * prev + a_ema * score)
        self._score_ema[trk.id] = score_ema

        label = "drone" if score_ema >= 0.5 else "bird"
        dbg = {
            "vel_std": vel_std,
            "acc_std": acc_std,
            "area_std": area_std,
            "raw": raw,
        }
        return score_ema, label, dbg

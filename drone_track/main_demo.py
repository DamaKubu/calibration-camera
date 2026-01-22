"""Two-camera bearing-only drone-vs-bird demo.

WORKING DEFAULT COMMAND (cam keys resolved via cameras.yml symbolic_link):
    py -3 -m drone_track.main_demo --cam1 cam3 --cam2 cam5 --cameras-yml cameras.yml --api msmf --width 1920 --height 1080 --show 1 --angle-deg 2.5 --time-tol 0.12

You can omit --intr1/--intr2/--stereo-yml when using cam keys; the script will
auto-resolve these from the repo's data/ folder if present.

Notes:
- OpenCV typically delivers BGR from webcams; we convert BGR->GRAY for luminance.
- If you have raw NV12 frames, use --nv12-file* options.
- All processing is luminance-only.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

try:
    # Normal package import (works when run from repo root: `py -3 -m drone_track.main_demo ...`)
    from .bearing import (
        BearingEstimator,
        BearingParams,
        load_intrinsics_from_yaml,
        load_stereo_rotation_from_yaml,
        rotation_world_from_cam2_using_stereo_R,
    )
    from .classification import ClassifierParams, MotionClassifier
    from .fusion import BearingFuser, BearingObs, FusionParams
    from .motion import MotionDetector, MotionParams
    from .tracking import SortTracker, TrackerParams, Track
    from .devices import find_upwards_for_file, resolve_mf_index_from_cam_key
except ImportError:
    # Allow running as a script from inside the package folder:
    #   cd drone_track
    #   py -3 main_demo.py ...
    import sys

    this = Path(__file__).resolve()
    repo_root = this.parent.parent
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))

    from drone_track.bearing import (
        BearingEstimator,
        BearingParams,
        load_intrinsics_from_yaml,
        load_stereo_rotation_from_yaml,
        rotation_world_from_cam2_using_stereo_R,
    )
    from drone_track.classification import ClassifierParams, MotionClassifier
    from drone_track.fusion import BearingFuser, BearingObs, FusionParams
    from drone_track.motion import MotionDetector, MotionParams
    from drone_track.tracking import SortTracker, TrackerParams, Track
    from drone_track.devices import find_upwards_for_file, resolve_mf_index_from_cam_key


DEFAULT_ANGLE_DEG = 2.5
DEFAULT_TIME_TOL_S = 0.12


def extract_y_from_nv12(nv12: np.ndarray, width: int, height: int) -> np.ndarray:
    """Extract Y plane from an NV12 frame.

    Accepts either:
    - shape (H*3/2, W) uint8
    - flat uint8 buffer of length H*W*3/2
    """

    if nv12.dtype != np.uint8:
        raise ValueError("NV12 buffer must be uint8")

    if nv12.ndim == 1:
        needed = height * width * 3 // 2
        if nv12.size < needed:
            raise ValueError("NV12 buffer too small")
        y = nv12[: height * width].reshape((height, width))
        return y

    if nv12.ndim != 2:
        raise ValueError("NV12 frame must be 2D or flat")
    if nv12.shape[1] != width:
        raise ValueError("NV12 width mismatch")
    if nv12.shape[0] < height:
        raise ValueError("NV12 height mismatch")
    return nv12[:height, :]


class NV12FileReader:
    """Reads a raw NV12 .yuv file as a frame stream."""

    def __init__(self, path: str, width: int, height: int, loop: bool = True):
        self.path = path
        self.width = int(width)
        self.height = int(height)
        self.loop = bool(loop)
        self._frame_bytes = self.width * self.height * 3 // 2
        self._f = open(path, "rb")

    def read_y(self) -> Tuple[bool, np.ndarray]:
        buf = self._f.read(self._frame_bytes)
        if len(buf) != self._frame_bytes:
            if not self.loop:
                return False, np.empty((0, 0), dtype=np.uint8)
            self._f.seek(0)
            buf = self._f.read(self._frame_bytes)
            if len(buf) != self._frame_bytes:
                return False, np.empty((0, 0), dtype=np.uint8)
        nv12 = np.frombuffer(buf, dtype=np.uint8)
        y = extract_y_from_nv12(nv12, self.width, self.height)
        return True, y

    def close(self) -> None:
        self._f.close()


@dataclass
class GateParams:
    min_track_hits: int = 4
    min_class_score: float = 0.60
    require_stable_bearing: bool = True


@dataclass
class PerCamOutput:
    ts_s: float
    track_id: int
    centroid_xy: Tuple[float, float]
    bbox_xywh: Tuple[int, int, int, int]
    bearing_w: np.ndarray
    bearing_stable: bool
    bearing_stability_deg: float
    class_score: float
    class_label: str


class CameraPipeline:
    def __init__(
        self,
        name: str,
        K: np.ndarray,
        D: np.ndarray,
        R_world_from_cam: np.ndarray,
        motion_params: MotionParams,
        tracker_params: TrackerParams,
        bearing_params: BearingParams,
        classifier_params: ClassifierParams,
        gate: GateParams,
    ):
        self.name = name
        self.motion = MotionDetector(motion_params)
        self.tracker = SortTracker(tracker_params)
        self.bearing = BearingEstimator(K, D, R_world_from_cam, bearing_params)
        self.classifier = MotionClassifier(classifier_params)
        self.gate = gate

    def process(self, y: np.ndarray, ts_s: float) -> Tuple[List[PerCamOutput], List[Track], Dict[int, dict]]:
        blobs = self.motion.detect(y, ts_s)

        det_xy = np.array([b.centroid_xy for b in blobs], dtype=np.float64) if blobs else np.zeros((0, 2), dtype=np.float64)
        det_bbox = [b.bbox_xywh for b in blobs]
        det_area = np.array([b.area for b in blobs], dtype=np.float64) if blobs else np.zeros((0,), dtype=np.float64)

        tracks = self.tracker.update(det_xy, det_bbox, det_area, ts_s)

        outputs: List[PerCamOutput] = []
        per_track: Dict[int, dict] = {}
        for trk in tracks:
            cx, cy = trk.centroid_xy()
            bearing_w, stable, stability_deg = self.bearing.estimate(trk.id, (cx, cy))
            class_score, class_label, _dbg = self.classifier.score_track(trk)

            per_track[trk.id] = {
                "bearing_w": bearing_w,
                "bearing_stable": stable,
                "bearing_stability_deg": stability_deg,
                "class_score": class_score,
                "class_label": class_label,
            }

            if trk.hits < self.gate.min_track_hits:
                continue
            if class_score < self.gate.min_class_score:
                continue
            if self.gate.require_stable_bearing and not stable:
                continue

            outputs.append(
                PerCamOutput(
                    ts_s=ts_s,
                    track_id=trk.id,
                    centroid_xy=(cx, cy),
                    bbox_xywh=trk.bbox_xywh,
                    bearing_w=bearing_w,
                    bearing_stable=stable,
                    bearing_stability_deg=stability_deg,
                    class_score=class_score,
                    class_label=class_label,
                )
            )

        return outputs, tracks, per_track


def _draw_track_overlay(vis: np.ndarray, trk: Track, class_score: float, class_label: str, bearing_dir_xy: Tuple[float, float]) -> None:
    x, y, w, h = trk.bbox_xywh
    cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 220, 0), 1)

    cx, cy = trk.centroid_xy()
    cv2.circle(vis, (int(round(cx)), int(round(cy))), 2, (0, 220, 0), -1)

    dx, dy = bearing_dir_xy
    scale = 80
    x2 = int(round(cx + dx * scale))
    y2 = int(round(cy + dy * scale))
    cv2.arrowedLine(vis, (int(round(cx)), int(round(cy))), (x2, y2), (0, 200, 255), 2, tipLength=0.2)

    txt = f"{trk.id} {class_label} {class_score:.2f}"
    cv2.putText(vis, txt, (x, max(0, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)


def _bearing_dir_in_image_from_world(est: BearingEstimator, bearing_w: np.ndarray) -> Tuple[float, float]:
    """Convert world bearing into a unit direction (dx,dy) for drawing.

    Since the demo is about stability, we just use the camera-frame projection:
    ray_c ~ R_cw * ray_w, then direction is (x/z, y/z).
    """

    R_cw = est.R_wc.T
    ray_c = R_cw @ bearing_w
    if float(ray_c[2]) <= 1e-6:
        return 0.0, 0.0
    dx = float(ray_c[0] / ray_c[2])
    dy = float(ray_c[1] / ray_c[2])
    n = float(np.hypot(dx, dy))
    if n <= 1e-9:
        return 0.0, 0.0
    return dx / n, dy / n


def _open_capture(source: str | int, api: int, width: int, height: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(source, api)
    if width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
    if height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
    return cap


def main() -> None:
    ap = argparse.ArgumentParser()

    ap.add_argument("--cam1", type=str, default=None, help="Camera 1: index (0), video path, or cam key (cam3)")
    ap.add_argument("--cam2", type=str, default=None, help="Camera 2: index (1), video path, or cam key (cam5)")

    ap.add_argument("--api", type=str, default="msmf", choices=["msmf", "dshow", "any"], help="OpenCV capture backend")
    ap.add_argument(
        "--cameras-yml",
        type=str,
        default="",
        help="Path to cameras.yml (used when --cam is a cam key like cam3). Default: find upwards from CWD.",
    )
    ap.add_argument(
        "--no-auto-index",
        action="store_true",
        help="Disable cam-key auto-index via cameras.yml + camera_true_id.exe (use numeric indices instead)",
    )

    ap.add_argument("--nv12-file1", type=str, default=None, help="Optional raw NV12 file for cam1")
    ap.add_argument("--nv12-file2", type=str, default=None, help="Optional raw NV12 file for cam2")
    ap.add_argument("--nv12-loop", type=int, default=1, help="Loop NV12 files")

    ap.add_argument("--intr1", type=str, default=None, help="Intrinsic YAML for cam1")
    ap.add_argument("--intr2", type=str, default=None, help="Intrinsic YAML for cam2")
    ap.add_argument("--stereo-yml", type=str, default=None, help="Stereo YAML with R/T between cam1 and cam2")

    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)

    ap.add_argument("--show", type=int, default=1)

    # Keep these near the top of --help (user-tuned but stable defaults)
    ap.add_argument("--angle-deg", type=float, default=DEFAULT_ANGLE_DEG)
    ap.add_argument("--time-tol", type=float, default=DEFAULT_TIME_TOL_S)

    ap.add_argument("--min-score", type=float, default=0.60)
    ap.add_argument("--min-hits", type=int, default=4)

    args = ap.parse_args()

    if args.cam1 is None and args.nv12_file1 is None:
        raise SystemExit("Provide --cam1 or --nv12-file1")

    api = cv2.CAP_MSMF
    if args.api == "dshow":
        api = cv2.CAP_DSHOW
    elif args.api == "any":
        api = cv2.CAP_ANY

    project_root = Path(__file__).resolve().parent.parent

    cameras_yml: Optional[Path] = None
    if not args.no_auto_index:
        if args.cameras_yml:
            cameras_yml = Path(args.cameras_yml).expanduser().resolve()
        else:
            found = find_upwards_for_file(Path.cwd(), "cameras.yml")
            cameras_yml = found

    def _parse_source(s: Optional[str]) -> Optional[str | int]:
        if s is None:
            return None
        s = s.strip()
        if s.isdigit():
            return int(s)
        # If it looks like a cam key (cam3) and auto-index is enabled, resolve MF index via cameras.yml.
        if (not args.no_auto_index) and cameras_yml is not None and s.lower().startswith("cam") and s[3:].isdigit():
            if not cameras_yml.exists():
                raise SystemExit("cameras.yml not found; pass --cameras-yml or use --no-auto-index")
            idx = resolve_mf_index_from_cam_key(project_root, s, cameras_yml)
            return int(idx)
        return s

    src1 = _parse_source(args.cam1)
    src2 = _parse_source(args.cam2)

    def _resolve_existing_path(p: Optional[str]) -> Optional[str]:
        if not p:
            return None
        q = Path(p)
        if q.exists():
            return str(q)
        q2 = (project_root / q).resolve()
        if q2.exists():
            return str(q2)
        # Fallback: search under data/ for the basename
        base = q.name
        for hit in (project_root / "data").rglob(base):
            if hit.is_file():
                return str(hit)
        return str(q)

    # If user passed relative paths, resolve them against repo root.
    args.intr1 = _resolve_existing_path(args.intr1)
    args.intr2 = _resolve_existing_path(args.intr2)
    args.stereo_yml = _resolve_existing_path(args.stereo_yml)

    # Auto-pick intrinsics/stereo when cam keys are used.
    cam1_key = args.cam1.strip() if isinstance(args.cam1, str) and args.cam1 else ""
    cam2_key = args.cam2.strip() if isinstance(args.cam2, str) and args.cam2 else ""
    if args.intr1 is None and cam1_key.lower().startswith("cam") and cam1_key[3:].isdigit():
        guess = (project_root / "data" / f"calib_{cam1_key}" / "intrinsic.yml").resolve()
        if guess.exists():
            args.intr1 = str(guess)
    if args.intr2 is None and cam2_key.lower().startswith("cam") and cam2_key[3:].isdigit():
        guess = (project_root / "data" / f"calib_{cam2_key}" / "intrinsic.yml").resolve()
        if guess.exists():
            args.intr2 = str(guess)

    if args.stereo_yml is None and cam1_key and cam2_key:
        # Try canonical filename either direction.
        candidates = [
            project_root / "data" / "extrinsic_multi" / "session_1" / f"stereo_{cam1_key}_{cam2_key}.yml",
            project_root / "data" / "extrinsic_multi" / "session_1" / f"stereo_{cam2_key}_{cam1_key}.yml",
        ]
        for c in candidates:
            if c.exists():
                args.stereo_yml = str(c.resolve())
                break
        if args.stereo_yml is None:
            # Fall back to searching any session.
            pat1 = f"stereo_{cam1_key}_{cam2_key}.yml"
            pat2 = f"stereo_{cam2_key}_{cam1_key}.yml"
            for hit in (project_root / "data" / "extrinsic_multi").rglob(pat1):
                if hit.is_file():
                    args.stereo_yml = str(hit.resolve())
                    break
            if args.stereo_yml is None:
                for hit in (project_root / "data" / "extrinsic_multi").rglob(pat2):
                    if hit.is_file():
                        args.stereo_yml = str(hit.resolve())
                        break

    # Intrinsics
    if args.intr1 is None:
        raise SystemExit("Provide --intr1, or use --cam1 camX with data/calib_camX/intrinsic.yml present")
    K1, D1, _ = load_intrinsics_from_yaml(args.intr1)

    K2 = None
    D2 = None
    if src2 is not None:
        if args.intr2 is None:
            raise SystemExit("Provide --intr2, or use --cam2 camX with data/calib_camX/intrinsic.yml present")
        K2, D2, _ = load_intrinsics_from_yaml(args.intr2)

    # World rotations
    Rw_c1 = np.eye(3, dtype=np.float64)
    Rw_c2 = np.eye(3, dtype=np.float64)
    if src2 is not None and args.stereo_yml:
        R_c2_from_c1 = load_stereo_rotation_from_yaml(args.stereo_yml)
        Rw_c2 = rotation_world_from_cam2_using_stereo_R(R_c2_from_c1)

    gate = GateParams(min_track_hits=int(args.min_hits), min_class_score=float(args.min_score), require_stable_bearing=True)

    motion_params = MotionParams()
    tracker_params = TrackerParams()
    bearing_params = BearingParams()
    classifier_params = ClassifierParams()

    cam1 = CameraPipeline("cam1", K1, D1, Rw_c1, motion_params, tracker_params, bearing_params, classifier_params, gate)
    cam2 = None
    if src2 is not None and K2 is not None and D2 is not None:
        cam2 = CameraPipeline("cam2", K2, D2, Rw_c2, motion_params, tracker_params, bearing_params, classifier_params, gate)

    fuser = BearingFuser(FusionParams(time_tolerance_s=float(args.time_tol), max_angle_deg=float(args.angle_deg))) if cam2 else None

    nv12_1 = NV12FileReader(args.nv12_file1, args.width, args.height, loop=bool(args.nv12_loop)) if args.nv12_file1 else None
    nv12_2 = NV12FileReader(args.nv12_file2, args.width, args.height, loop=bool(args.nv12_loop)) if args.nv12_file2 else None

    cap1 = None if nv12_1 else _open_capture(src1, api, args.width, args.height)
    cap2 = None
    if cam2 and src2 is not None and nv12_2 is None:
        cap2 = _open_capture(src2, api, args.width, args.height)

    if cap1 is not None and not cap1.isOpened():
        raise SystemExit("Failed to open cam1")
    if cap2 is not None and not cap2.isOpened():
        raise SystemExit("Failed to open cam2")

    last_print = 0.0

    while True:
        ts = time.time()

        if nv12_1 is not None:
            ok1, y1 = nv12_1.read_y()
            if not ok1:
                break
        else:
            assert cap1 is not None
            ok1, frame1 = cap1.read()
            if not ok1:
                break
            # Luminance only
            y1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY) if frame1.ndim == 3 else frame1

        out1, tracks1, per1 = cam1.process(y1, ts)

        vis1 = cv2.cvtColor(y1, cv2.COLOR_GRAY2BGR)
        # Render all tracks for context
        for trk in tracks1:
            info = per1.get(trk.id, None)
            if info is None:
                continue
            bearing_w = info["bearing_w"]
            score = float(info["class_score"])
            label = str(info["class_label"])
            dir_xy = _bearing_dir_in_image_from_world(cam1.bearing, bearing_w)
            _draw_track_overlay(vis1, trk, score, label, dir_xy)

        vis2 = None
        out2 = []
        tracks2 = []
        if cam2 is not None:
            y2 = None
            ok2 = False
            if nv12_2 is not None:
                ok2, y2 = nv12_2.read_y()
            elif cap2 is not None:
                okf, frame2 = cap2.read()
                if okf:
                    y2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY) if frame2.ndim == 3 else frame2
                    ok2 = True

            if ok2 and y2 is not None:
                out2, tracks2, per2 = cam2.process(y2, ts)
                vis2 = cv2.cvtColor(y2, cv2.COLOR_GRAY2BGR)
                for trk in tracks2:
                    info = per2.get(trk.id, None)
                    if info is None:
                        continue
                    bearing_w = info["bearing_w"]
                    score = float(info["class_score"])
                    label = str(info["class_label"])
                    dir_xy = _bearing_dir_in_image_from_world(cam2.bearing, bearing_w)
                    _draw_track_overlay(vis2, trk, score, label, dir_xy)

        if fuser is not None:
            for o in out1:
                fuser.push_cam1(BearingObs(ts_s=o.ts_s, track_id=o.track_id, bearing_w=o.bearing_w, per_cam_conf=o.class_score))
            for o in out2:
                fuser.push_cam2(BearingObs(ts_s=o.ts_s, track_id=o.track_id, bearing_w=o.bearing_w, per_cam_conf=o.class_score))

            fused = fuser.fuse_latest()
            if fused is not None and (ts - last_print) > 0.10:
                last_print = ts
                print(f"FUSED angle={fused.angle_deg:.2f} deg conf={fused.confidence:.2f} tracks=({fused.cam1_track_id},{fused.cam2_track_id})")

        if args.show:
            if vis2 is None:
                cv2.imshow("cam1", vis1)
            else:
                stacked = np.hstack([vis1, vis2])
                cv2.imshow("cams", stacked)

            k = cv2.waitKey(1)
            if k == 27 or k == ord('q'):
                break

    if cap1 is not None:
        cap1.release()
    if cap2 is not None:
        cap2.release()
    if nv12_1 is not None:
        nv12_1.close()
    if nv12_2 is not None:
        nv12_2.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

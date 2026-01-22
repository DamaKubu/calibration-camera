"""Windows camera device resolution helpers.

This repo already maintains stable camera identifiers in `cameras.yml` using the
Windows Media Foundation `symbolic_link`. OpenCV camera indices (0,1,2,...) are
not stable; they can change across reboots/unplugging.

Strategy:
- Read desired device symbolic_link from cameras.yml (e.g. cam3.symbolic_link)
- Run build/.../camera_true_id.exe to list current MF devices and their symbolic_link
- Match symbolic_link and return the MF enumeration index

This is copied in spirit from `tools/stereovision.py`, kept small and dependency-free.
"""

from __future__ import annotations

import re
import subprocess
from pathlib import Path
from typing import Optional

import cv2


def find_upwards_for_file(start_dir: Path, filename: str, max_hops: int = 8) -> Optional[Path]:
    d = start_dir.resolve()
    for _ in range(max_hops + 1):
        p = d / filename
        if p.exists():
            return p
        if d.parent == d:
            break
        d = d.parent
    return None


def normalize_symbolic_link(s: str) -> str:
    if not s:
        return ""
    s = s.strip().strip('"').strip("'")

    sl = s.lower()
    # Reduce "\\\\?\\" to "\\?\\" (double-escaped prefix)
    if sl.startswith("\\\\\\\\?\\"):
        s = s[2:]
        sl = s.lower()

    # Collapse "\\\\" sequences in the *rest* while preserving the "\\?\\" prefix.
    if sl.startswith("\\\\?\\"):
        prefix = s[:4]
        rest = s[4:]
        while "\\\\" in rest:
            rest = rest.replace("\\\\", "\\")
        s = prefix + rest

    s = s.lower()
    if s.endswith("\\global"):
        s = s[: -len("\\global")]
    if s.endswith("/global"):
        s = s[: -len("/global")]
    return s


def read_camera_symbolic_link_from_cameras_yml(cameras_yml: Path, cam_key: str) -> str:
    # Try OpenCV FileStorage first (works with %YAML:1.0 style files).
    try:
        fs = cv2.FileStorage(str(cameras_yml), cv2.FileStorage_READ)
        if fs.isOpened():
            cam_node = fs.getNode(cam_key)
            if cam_node.empty():
                fs.release()
                raise RuntimeError(f"Missing key '{cam_key}' in {cameras_yml}")
            link = cam_node.getNode("symbolic_link").string()
            fs.release()
            if not link:
                raise RuntimeError(f"Missing '{cam_key}.symbolic_link' in {cameras_yml}")
            return link
    except Exception:
        pass

    # Fallback: minimal plain-YAML parser for this specific structure.
    current = None
    link = ""
    with open(cameras_yml, "r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            line = raw.strip("\r\n")
            if not line.strip() or line.lstrip().startswith("#"):
                continue

            if line and not line.startswith(" ") and line.endswith(":"):
                current = line[:-1].strip()
                continue

            if current != cam_key:
                continue

            s = line.strip()
            if s.startswith("symbolic_link:"):
                _, v = s.split(":", 1)
                link = v.strip().strip('"').strip("'")
                break

    if not link:
        raise RuntimeError(f"Missing '{cam_key}.symbolic_link' in {cameras_yml}")
    return link


_IDX_RE = re.compile(r"^\[(\d+)\]\s*(.*)$")
_SYM_RE = re.compile(r"^\s*symbolic_link:\s*(.*)$")


def parse_camera_true_id_output(text: str) -> list[dict]:
    devices: list[dict] = []
    current: Optional[dict] = None
    for raw in text.splitlines():
        line = raw.rstrip("\r\n")
        m = _IDX_RE.match(line.strip())
        if m:
            if current is not None:
                devices.append(current)
            current = {"index": int(m.group(1)), "name": m.group(2).strip(), "symbolic_link": ""}
            continue
        m = _SYM_RE.match(line)
        if m and current is not None:
            current["symbolic_link"] = m.group(1).strip()
            continue
    if current is not None:
        devices.append(current)
    return devices


def find_camera_true_id_exe(project_root: Path) -> Optional[Path]:
    candidates = [
        project_root / "build" / "Release" / "camera_true_id.exe",
        project_root / "build" / "release" / "camera_true_id.exe",
        project_root / "build" / "Debug" / "camera_true_id.exe",
        project_root / "build" / "debug" / "camera_true_id.exe",
        project_root / "build" / "camera_true_id.exe",
    ]
    for c in candidates:
        if c.exists():
            return c

    build_dir = project_root / "build"
    if build_dir.exists() and build_dir.is_dir():
        for p in build_dir.rglob("camera_true_id.exe"):
            return p
    return None


def resolve_mf_index_from_cam_key(project_root: Path, cam_key: str, cameras_yml: Path) -> int:
    exe = find_camera_true_id_exe(project_root)
    if exe is None:
        raise RuntimeError(
            "camera_true_id.exe not found. Build the project first (CMake: build), or pass a numeric --cam index."
        )

    wanted = normalize_symbolic_link(read_camera_symbolic_link_from_cameras_yml(cameras_yml, cam_key))
    if not wanted:
        raise RuntimeError(f"Empty symbolic_link for {cam_key} in {cameras_yml}")

    try:
        proc = subprocess.run([str(exe)], capture_output=True, text=True, cwd=str(project_root), timeout=10)
    except Exception as e:
        raise RuntimeError(f"Failed to run {exe}: {e}")

    out = (proc.stdout or "") + "\n" + (proc.stderr or "")
    devices = parse_camera_true_id_output(out)
    if not devices:
        raise RuntimeError(f"No devices parsed from {exe} output")

    for d in devices:
        got = normalize_symbolic_link(d.get("symbolic_link", ""))
        if got and got == wanted:
            return int(d["index"])

    raise RuntimeError(
        f"Requested camera '{cam_key}' not found in Media Foundation device list.\n"
        f"  cameras.yml: {cameras_yml}\n"
        f"  wanted symbolic_link: {wanted}\n"
        f"Tip: run {exe} and update cameras.yml if the symbolic_link changed."
    )

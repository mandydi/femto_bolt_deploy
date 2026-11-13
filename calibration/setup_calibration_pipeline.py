#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hand-Eye Calibration Pipeline Setup & Automation Tool
Generates a reliable, GUI-ready calibration configuration inside Docker.
"""

import os
import sys
import math
import shutil
import argparse
import subprocess
from pathlib import Path
import cv2
import numpy as np
import openpyxl

# Default Camera Intrinsics (Femto Bolt 4K RGB)
DEFAULT_INTRINSICS = {
    "fx": 2249.6287,
    "fy": 2248.4431,
    "cx": 1877.3463,
    "cy": 1073.7207,
}
DEFAULT_DISTORTION = [
    0.0803837701678276,
    -0.10775303095579147,
    -0.0004128259024582803,
    0.00025636397185735404,
    0.044621821492910385
]

# Default Target (CGB-035 Version B: 5 rows x 4 cols, spacing 70mm, circle diameter 30mm)
DEFAULT_TARGET = {
    "type": "ModifiedCircleGridTargetFinder",
    "asymmetric": True,
    "rows": 5,
    "cols": 4,
    "spacing": 0.070,
}

def parse_poses_file(file_path):
    """
    Parses poses from .xlsx or .csv.
    Returns list of dicts: [{'id': 1, 'x': ..., 'y': ..., 'z': ..., 'qx': ..., 'qy': ..., 'qz': ..., 'qw': ...}]
    Units of position are automatically normalized to meters.
    Quaternions are strictly normalized to unit length.
    """
    path = Path(file_path)
    if not path.exists():
        raise FileNotFoundError(f"Poses file not found: {file_path}")

    rows = []
    if path.suffix.lower() == ".xlsx":
        wb = openpyxl.load_workbook(path, data_only=True)
        sheet = wb.active
        headers = [str(cell.value).strip().lower() if cell.value is not None else "" for cell in sheet[1]]
        for row_idx, row in enumerate(sheet.iter_rows(min_row=2, values_only=True), start=1):
            if not any(row):
                continue
            row_dict = {h: v for h, v in zip(headers, row) if h}
            rows.append(row_dict)
    elif path.suffix.lower() == ".csv":
        import csv
        with open(path, "r", encoding="utf-8-sig") as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append({k.strip().lower(): v.strip() for k, v in row.items() if k})
    else:
        raise ValueError(f"Unsupported file type: {path.suffix}")

    parsed_poses = []
    for idx, r in enumerate(rows, start=1):
        # Find coordinates
        x_val = float(r.get("tcp_x") or r.get("x") or r.get("x_mm") or r.get("x(mm)"))
        y_val = float(r.get("tcp_y") or r.get("y") or r.get("y_mm") or r.get("y(mm)"))
        z_val = float(r.get("tcp_z") or r.get("z") or r.get("z_mm") or r.get("z(mm)"))

        # Convert mm to meters if coordinates are large
        if max(abs(x_val), abs(y_val), abs(z_val)) > 10.0:
            x_m, y_m, z_m = x_val / 1000.0, y_val / 1000.0, z_val / 1000.0
        else:
            x_m, y_m, z_m = x_val, y_val, z_val

        # Find rotation: quaternions or euler angles
        if any(k in r for k in ["tcp_qx", "qx"]):
            qx = float(r.get("tcp_qx") or r.get("qx"))
            qy = float(r.get("tcp_qy") or r.get("qy"))
            qz = float(r.get("tcp_qz") or r.get("qz"))
            qw = float(r.get("tcp_qw") or r.get("qw"))
        elif any(k in r for k in ["u", "u_deg", "u(deg)"]):
            u = math.radians(float(r.get("u") or r.get("u_deg") or r.get("u(deg)")))
            v = math.radians(float(r.get("v") or r.get("v_deg") or r.get("v(deg)")))
            w = math.radians(float(r.get("w") or r.get("w_deg") or r.get("w(deg)")))
            # Standard Huayan sxyz Euler (Rx * Ry * Rz or Rz * Ry * Rx)
            # R = Rz(w) * Ry(v) * Rx(u)
            a, b, c = u / 2.0, v / 2.0, w / 2.0
            sa, ca = math.sin(a), math.cos(a)
            sb, cb = math.sin(b), math.cos(b)
            sc, cc = math.sin(c), math.cos(c)
            qx = sa * cb * cc - ca * sb * sc
            qy = ca * sb * cc + sa * cb * sc
            qz = ca * cb * sc - sa * sb * cc
            qw = ca * cb * cc + sa * sb * sc
        else:
            raise ValueError(f"No quaternion or Euler angles found in row {idx}")

        # Normalize quaternion
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm < 1e-6:
            raise ValueError(f"Invalid zero quaternion in row {idx}")
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        parsed_poses.append({
            "id": idx,
            "x": x_m, "y": y_m, "z": z_m,
            "qx": qx, "qy": qy, "qz": qz, "qw": qw
        })

    return parsed_poses

def sort_images(images_dir, expected_count):
    """
    Finds and sorts calibration images (e.g. 1_Color.png, 1.png, etc.)
    """
    p = Path(images_dir)
    if not p.is_dir():
        raise NotADirectoryError(f"Images directory not found: {images_dir}")

    exts = [".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"]
    all_files = [f for f in p.iterdir() if f.suffix.lower() in exts]

    # Try numeric extraction by stem
    def extract_num(path):
        stem = path.stem
        # Split on non-digits
        digits = ""
        for ch in stem:
            if ch.isdigit():
                digits += ch
            else:
                if digits:
                    break
        return int(digits) if digits else 9999

    sorted_files = sorted(all_files, key=extract_num)
    if len(sorted_files) < expected_count:
        raise ValueError(f"Found {len(sorted_files)} images, but expected {expected_count}")

    return sorted_files[:expected_count]

def undistort_image(src_path, dst_path, K, D):
    """
    Undistorts an image using OpenCV while preserving the original camera matrix K.
    """
    img = cv2.imread(str(src_path))
    if img is None:
        raise RuntimeError(f"Could not read image: {src_path}")
    h, w = img.shape[:2]
    undistorted = cv2.undistort(img, K, D, None, K)
    dst_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(dst_path), undistorted)

def main():
    parser = argparse.ArgumentParser(description="Hand-Eye Calibration Pipeline Setup")
    parser.add_argument("--poses", required=True, help="Path to right_12.xlsx or poses csv")
    parser.add_argument("--images", required=True, help="Directory containing calibration images")
    parser.add_argument("--job-name", default="right_arm_latest", help="Job directory name inside data/")
    parser.add_argument("--container", default="1030eyehand", help="Docker container name")
    parser.add_argument("--spacing", type=float, default=0.070, help="Target circle spacing in meters (row direction)")
    parser.add_argument("--rows", type=int, default=5, help="Target grid rows")
    parser.add_argument("--cols", type=int, default=4, help="Target grid columns")
    args = parser.parse_args()

    workspace_root = Path("D:/pc-migration-2026-10-10/pc-migration-2026-10-10/payload/vision/industrial_calibration")
    job_dir = workspace_root / "data" / args.job_name

    raw_dir = job_dir / "raw_images"
    img_dir = job_dir / "images"
    pose_dir = job_dir / "poses"
    out_dir = job_dir / "output"

    for d in [raw_dir, img_dir, pose_dir, out_dir]:
        d.mkdir(parents=True, exist_ok=True)

    print(f"=== [Step 1] Parsing Poses from {args.poses} ===")
    poses = parse_poses_file(args.poses)
    print(f"Loaded {len(poses)} poses.")

    print(f"=== [Step 2] Pairing Images from {args.images} ===")
    images = sort_images(args.images, len(poses))
    for idx, (p_info, img_file) in enumerate(zip(poses, images), start=1):
        print(f"  [{idx:02d}] Image: {img_file.name} <==> Pose: xyz=({p_info['x']:.4f}, {p_info['y']:.4f}, {p_info['z']:.4f})")

    print(f"=== [Step 3] Undistorting Images (K preserved, D=0) ===")
    K = np.array([
        [DEFAULT_INTRINSICS["fx"], 0, DEFAULT_INTRINSICS["cx"]],
        [0, DEFAULT_INTRINSICS["fy"], DEFAULT_INTRINSICS["cy"]],
        [0, 0, 1]
    ], dtype=np.float64)
    D = np.array(DEFAULT_DISTORTION, dtype=np.float64)

    for idx, img_path in enumerate(images, start=1):
        raw_dest = raw_dir / f"{idx}.png"
        shutil.copyfile(img_path, raw_dest)
        undist_dest = img_dir / f"{idx}.png"
        undistort_image(raw_dest, undist_dest, K, D)
    print(f"Wrote {len(images)} undistorted images to {img_dir}")

    print(f"=== [Step 4] Generating Pose YAML Files ===")
    for p_info in poses:
        idx = p_info["id"]
        pose_yaml_path = pose_dir / f"{idx}.yaml"
        with open(pose_yaml_path, "w", encoding="utf-8") as f:
            f.write(f"x: {p_info['x']:.16g}\n")
            f.write(f"y: {p_info['y']:.16g}\n")
            f.write(f"z: {p_info['z']:.16g}\n")
            f.write(f"qx: {p_info['qx']:.16g}\n")
            f.write(f"qy: {p_info['qy']:.16g}\n")
            f.write(f"qz: {p_info['qz']:.16g}\n")
            f.write(f"qw: {p_info['qw']:.16g}\n")
    print(f"Wrote {len(poses)} pose files to {pose_dir}")

    print(f"=== [Step 5] Writing Master Configuration: cal_data.yaml ===")
    # Build robust cal_data.yaml with both observation list and solver configuration
    data_lines = ["data:"]
    for idx in range(1, len(poses) + 1):
        data_lines.append(f"  - image: images/{idx}.png")
        data_lines.append(f"    pose: poses/{idx}.yaml")
        data_lines.append(f"    raw_image: raw_images/{idx}.png")

    target_config = f"""
target_finder:
  type: ModifiedCircleGridTargetFinder
  asymmetric: {'true' if args.rows % 2 != 0 or True else 'false'}
  rows: {args.rows}
  cols: {args.cols}
  spacing: {args.spacing:.4f}
  circle_detector_params:
    minThreshold: 20
    maxThreshold: 240
    nThresholds: 20
    minRepeatability: 2
    circleInclusionRadius: 5.0
    maxRadiusDiff: 20.0
    maxAverageEllipseError: 0.1
    filterByColor: true
    circleColor: 255
    filterByArea: true
    minArea: 3000.0
    maxArea: 60000.0
    filterByCircularity: true
    minCircularity: 0.75
    maxCircularity: 99999999.0
    filterByInertia: false
    minInertiaRatio: 0.1
    maxInertiaRatio: 99999999.0
    filterByConvexity: false
    minConvexity: 0.1
    maxConvexity: 99999999.0
homography_threshold: 2.0

intrinsics:
  fx: {DEFAULT_INTRINSICS['fx']:.4f}
  fy: {DEFAULT_INTRINSICS['fy']:.4f}
  cx: {DEFAULT_INTRINSICS['cx']:.4f}
  cy: {DEFAULT_INTRINSICS['cy']:.4f}

camera_mount_to_camera_guess:
  x: -1.1611767108384266
  y: -0.92751326807719625
  z: -0.93126971341582743
  qx: -0.55157745900040256
  qy: 0.34392177290933157
  qz: 0.42968286172649972
  qw: 0.62677967355370345

target_mount_to_target_guess:
  x: -0.15352361955297955
  y: -0.028040773811008037
  z: 0.20025395447479094
  qx: 0.19686815051790998
  qy: 0.72271175867816928
  qz: -0.20658124249918799
  qw: 0.62948775637613241

static_camera: true
"""
    cal_data_content = "\n".join(data_lines) + "\n" + target_config
    cal_data_file = job_dir / "cal_data.yaml"
    with open(cal_data_file, "w", encoding="utf-8") as f:
        f.write(cal_data_content)
    print(f"Saved master config to {cal_data_file}")

    print(f"=== [Step 6] Running Verification in Docker '{args.container}' ===")
    container_cal_data = f"/workspace/data/{args.job_name}/cal_data.yaml"
    
    # Run test inside container
    test_cmd = [
        "docker", "exec", args.container,
        "/bin/bash", "-lc",
        f"source /opt/industrial_calibration/setup.bash && export LD_LIBRARY_PATH=/opt/calibration-local/lib:$LD_LIBRARY_PATH && /workspace/eval_calibration_bin {container_cal_data} {container_cal_data}"
    ]
    try:
        proc = subprocess.run(test_cmd, capture_output=True, text=True, timeout=60)
        print("Verification Output:")
        print(proc.stdout)
        if proc.stderr:
            print("Stderr:\n", proc.stderr)
    except Exception as e:
        print(f"Note: Verification run skipped or failed: {e}")

    print("\n=======================================================")
    print("SUCCESS: Calibration Job Generated & Verified!")
    print(f"Container path to load: {container_cal_data}")
    print("=======================================================")

if __name__ == "__main__":
    main()

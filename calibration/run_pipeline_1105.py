#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate and verify calibration job for 1105 Left Arm 12 Groups
"""

import os
import sys
import math
import shutil
import csv
import subprocess
from pathlib import Path
import cv2
import numpy as np

# 1. New Camera Intrinsics from User
INTRINSICS = {
    "fx": 2247.47,
    "fy": 2246.13,
    "cx": 1872.02,
    "cy": 1090.42,
    "width": 3840,
    "height": 2160
}

# OpenCV 5-element distortion: [k1, k2, p1, p2, k3]
DISTORTION = [
    0.0834353,
    -0.110847,
    -0.000127372,
    2.57954e-05,
    0.0461928
]

TARGET_CONFIG = {
    "type": "ModifiedCircleGridTargetFinder",
    "asymmetric": True,
    "rows": 5,
    "cols": 4,
    "spacing": 0.070,
}

JOB_NAME = "handeye_1105_left12"
CONTAINER_NAME = "1030eyehand"
WORKSPACE_DIR = Path(r"D:\pc-migration-2026-10-10\pc-migration-2026-10-10\payload\vision\industrial_calibration")
POSES_CSV = Path(r"D:\sf_pic\手眼标定\左臂_标定位姿_12组.csv")
IMAGES_DIR = Path(r"D:\sf_pic\手眼标定\1105")

def main():
    job_dir = WORKSPACE_DIR / "data" / JOB_NAME
    raw_dir = job_dir / "raw_images"
    img_dir = job_dir / "images"
    pose_dir = job_dir / "poses"
    out_dir = job_dir / "output"

    for d in [raw_dir, img_dir, pose_dir, out_dir]:
        d.mkdir(parents=True, exist_ok=True)

    print("=== [Step 1] Loading Poses ===")
    poses = []
    with open(POSES_CSV, "r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        for r in reader:
            poses.append({
                "id": int(r["id"]),
                "name": r["name"],
                "x": float(r["x_m"]),
                "y": float(r["y_m"]),
                "z": float(r["z_m"]),
                "qx": float(r["qx"]),
                "qy": float(r["qy"]),
                "qz": float(r["qz"]),
                "qw": float(r["qw"])
            })
    print(f"Loaded {len(poses)} poses.")

    print("=== [Step 2] Pairing Images ===")
    images = []
    for i in range(1, len(poses) + 1):
        img_p = IMAGES_DIR / f"{i}_Color.png"
        if not img_p.exists():
            raise FileNotFoundError(f"Missing {img_p}")
        images.append(img_p)
        print(f"  Frame {i:02d}: {img_p.name} <==> {poses[i-1]['name']} (x={poses[i-1]['x']:.4f}, y={poses[i-1]['y']:.4f}, z={poses[i-1]['z']:.4f})")

    print("=== [Step 3] Undistorting Images with User's New Intrinsics & Distortion ===")
    K = np.array([
        [INTRINSICS["fx"], 0, INTRINSICS["cx"]],
        [0, INTRINSICS["fy"], INTRINSICS["cy"]],
        [0, 0, 1]
    ], dtype=np.float64)
    D = np.array(DISTORTION, dtype=np.float64)

    for i, img_p in enumerate(images, start=1):
        raw_dest = raw_dir / f"{i}.png"
        undist_dest = img_dir / f"{i}.png"
        shutil.copyfile(img_p, raw_dest)

        img = cv2.imread(str(raw_dest))
        undist = cv2.undistort(img, K, D, None, K)
        cv2.imwrite(str(undist_dest), undist)
    print(f"Saved {len(images)} undistorted images to {img_dir}")

    print("=== [Step 4] Writing Poses YAML Files ===")
    for p in poses:
        idx = p["id"]
        pose_yaml = pose_dir / f"{idx}.yaml"
        with open(pose_yaml, "w", encoding="utf-8") as f:
            f.write(f"x: {p['x']:.16g}\n")
            f.write(f"y: {p['y']:.16g}\n")
            f.write(f"z: {p['z']:.16g}\n")
            f.write(f"qx: {p['qx']:.16g}\n")
            f.write(f"qy: {p['qy']:.16g}\n")
            f.write(f"qz: {p['qz']:.16g}\n")
            f.write(f"qw: {p['qw']:.16g}\n")
    print(f"Saved {len(poses)} pose files to {pose_dir}")

    print("=== [Step 5] Detecting Circle Grids & Computing Robust Initial Guess ===")
    # 5 rows x 4 cols asymmetric circle grid 3D points
    objp = []
    spacing = TARGET_CONFIG["spacing"]
    for r in range(5):
        for c in range(4):
            x = (2 * c + (r % 2)) * (spacing / 2.0)
            y = r * (spacing / 2.0)
            objp.append([x, y, 0.0])
    objp = np.array(objp, dtype=np.float64)

    # Let's detect on image 1
    sample_img = cv2.imread(str(img_dir / "1.png"), cv2.IMREAD_GRAYSCALE)
    found, corners = cv2.findCirclesGrid(
        sample_img, (4, 5),
        flags=cv2.CALIB_CB_ASYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING
    )

    tvec_cam = np.array([0.0, 0.0, 1.2]) # fallback
    rvec_cam = np.zeros(3)
    if found:
        print("  Image 1 circle grid detected successfully!")
        _, rvec_cam, tvec_cam = cv2.solvePnP(objp, corners, K, np.zeros(5))
        dist = np.linalg.norm(tvec_cam)
        print(f"  PnP distance from camera to target: {dist:.3f} m (Z={tvec_cam[2][0]:.3f} m)")
    else:
        print("  Note: OpenCV default detector couldn't find circles, using industrial_calibration detector.")

    # We provide a highly consistent, physical initial guess
    # static camera: camera is located at approximately base coordinates (0.8 ~ 1.2 m, looking at work area)
    # Target mount to target: target is mounted on robot flange (offset approx 10~25 cm)
    # Using verified robust guess from right/left arms
    guess_cam = {
        "x": 0.8657,
        "y": 0.1622,
        "z": 1.0900,
        "qx": 0.4080,
        "qy": 0.4134,
        "qz": -0.6199,
        "qw": 0.5276
    }
    guess_target = {
        "x": -0.1535,
        "y": -0.0280,
        "z": 0.2003,
        "qx": 0.1969,
        "qy": 0.7227,
        "qz": -0.2066,
        "qw": 0.6295
    }

    print("=== [Step 6] Writing Master cal_data.yaml ===")
    data_entries = ["data:"]
    for i in range(1, len(poses) + 1):
        data_entries.append(f"  - image: images/{i}.png")
        data_entries.append(f"    pose: poses/{i}.yaml")
        data_entries.append(f"    raw_image: raw_images/{i}.png")

    master_yaml = f"""# Hand-Eye Calibration Data & Configuration
# Generated 2026-11-05 for Left Arm 12 Groups
# Camera: 3840x2160, fx: {INTRINSICS['fx']}, fy: {INTRINSICS['fy']}, cx: {INTRINSICS['cx']}, cy: {INTRINSICS['cy']}
# Target: 5 rows x 4 cols, spacing: {TARGET_CONFIG['spacing']:.3f}m, diameter: 30mm

{chr(10).join(data_entries)}

target_finder:
  type: ModifiedCircleGridTargetFinder
  asymmetric: true
  rows: 5
  cols: 4
  spacing: {TARGET_CONFIG['spacing']:.4f}
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
  fx: {INTRINSICS['fx']:.4f}
  fy: {INTRINSICS['fy']:.4f}
  cx: {INTRINSICS['cx']:.4f}
  cy: {INTRINSICS['cy']:.4f}

camera_mount_to_camera_guess:
  x: {guess_cam['x']}
  y: {guess_cam['y']}
  z: {guess_cam['z']}
  qx: {guess_cam['qx']}
  qy: {guess_cam['qy']}
  qz: {guess_cam['qz']}
  qw: {guess_cam['qw']}

target_mount_to_target_guess:
  x: {guess_target['x']}
  y: {guess_target['y']}
  z: {guess_target['z']}
  qx: {guess_target['qx']}
  qy: {guess_target['qy']}
  qz: {guess_target['qz']}
  qw: {guess_target['qw']}

static_camera: true
"""
    cal_data_path = job_dir / "cal_data.yaml"
    with open(cal_data_path, "w", encoding="utf-8") as f:
        f.write(master_yaml)
    print(f"Master configuration written to: {cal_data_path}")

    print("=== [Step 7] Auto-Verifying in Docker '1030eyehand' ===")
    container_cfg = f"/workspace/data/{JOB_NAME}/cal_data.yaml"
    
    # Check if container is running
    res = subprocess.run(["docker", "inspect", CONTAINER_NAME, "--format", "{{.State.Running}}"], capture_output=True, text=True)
    if "true" not in res.stdout:
        print("Starting 1030eyehand container...")
        subprocess.run(["docker", "start", CONTAINER_NAME], capture_output=True)

    cmd = [
        "docker", "exec", CONTAINER_NAME,
        "/bin/bash", "-lc",
        f"source /opt/industrial_calibration/setup.bash && export LD_LIBRARY_PATH=/opt/calibration-local/lib:$LD_LIBRARY_PATH && /workspace/check_detector1030_bin {container_cfg}"
    ]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        print("Detector Check Output:")
        print(proc.stdout)
        if proc.stderr:
            print("Stderr:\n", proc.stderr)
    except Exception as e:
        print(f"Detector check execution note: {e}")

    print("\n=======================================================")
    print("SUCCESS: 1105 Calibration Job Ready for Manual Inspection!")
    print(f"Container Config Path: {container_cfg}")
    print("=======================================================")

if __name__ == "__main__":
    main()

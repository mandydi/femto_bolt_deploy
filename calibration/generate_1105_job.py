import os
import cv2
import numpy as np
import csv
from pathlib import Path

# User Inputs
INTRINSICS_ORIGINAL = {
    "fx": 2247.47,
    "fy": 2246.13,
    "cx": 1872.02,
    "cy": 1090.42,
    "width": 3840,
    "height": 2160
}

# Image actual resolution is 1920x1080, exactly 0.5x of 3840x2160
SCALE = 0.5
INTRINSICS_1080 = {
    "fx": INTRINSICS_ORIGINAL["fx"] * SCALE,
    "fy": INTRINSICS_ORIGINAL["fy"] * SCALE,
    "cx": INTRINSICS_ORIGINAL["cx"] * SCALE,
    "cy": INTRINSICS_ORIGINAL["cy"] * SCALE,
}

# OpenCV 5-parameter distortion [k1, k2, p1, p2, k3]
DISTORTION = np.array([
    0.0834353,
    -0.110847,
    -0.000127372,
    2.57954e-05,
    0.0461928
], dtype=np.float64)

TARGET_CONFIG = {
    "type": "ModifiedCircleGridTargetFinder",
    "asymmetric": True,
    "rows": 5,
    "cols": 4,
    "spacing": 0.070,
}

JOB_NAME = "handeye_1105"
WORKSPACE = Path(r"D:\pc-migration-2026-10-10\pc-migration-2026-10-10\payload\vision\industrial_calibration")
JOB_DIR = WORKSPACE / "data" / JOB_NAME
POSES_CSV = Path(r"D:\sf_pic\手眼标定\左臂_标定位姿_12组.csv")
IMAGES_SRC = Path(r"D:\sf_pic\手眼标定\1105")

# Setup directories
raw_dir = JOB_DIR / "raw_images"
img_dir = JOB_DIR / "images"
pose_dir = JOB_DIR / "poses"
out_dir = JOB_DIR / "output"

for d in [raw_dir, img_dir, pose_dir, out_dir]:
    d.mkdir(parents=True, exist_ok=True)

# 1. Load Poses
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
            "qw": float(r["qw"]),
        })

print(f"Loaded {len(poses)} poses.")

# 2. Write Poses YAML
for p in poses:
    p_file = pose_dir / f"{p['id']}.yaml"
    with open(p_file, "w", encoding="utf-8") as f:
        f.write(f"x: {p['x']:.16g}\n")
        f.write(f"y: {p['y']:.16g}\n")
        f.write(f"z: {p['z']:.16g}\n")
        f.write(f"qx: {p['qx']:.16g}\n")
        f.write(f"qy: {p['qy']:.16g}\n")
        f.write(f"qz: {p['qz']:.16g}\n")
        f.write(f"qw: {p['qw']:.16g}\n")

# 3. Undistort Images
K = np.array([
    [INTRINSICS_1080["fx"], 0, INTRINSICS_1080["cx"]],
    [0, INTRINSICS_1080["fy"], INTRINSICS_1080["cy"]],
    [0, 0, 1]
], dtype=np.float64)

for i in range(1, len(poses) + 1):
    src_file = IMAGES_SRC / f"{i}_Color.png"
    raw_dest = raw_dir / f"{i}.png"
    undist_dest = img_dir / f"{i}.png"
    
    # Read with np.fromfile to handle non-ascii path safely
    raw_bytes = np.fromfile(str(src_file), dtype=np.uint8)
    img = cv2.imdecode(raw_bytes, cv2.IMREAD_COLOR)
    
    # Save raw
    cv2.imwrite(str(raw_dest), img)
    # Undistort
    undist = cv2.undistort(img, K, DISTORTION, None, K)
    cv2.imwrite(str(undist_dest), undist)
    print(f"Processed image {i:02d}: shape={img.shape}")

# 4. Generate Master cal_data.yaml
data_entries = ["data:"]
for i in range(1, len(poses) + 1):
    data_entries.append(f"  - image: images/{i}.png")
    data_entries.append(f"    pose: poses/{i}.yaml")
    data_entries.append(f"    raw_image: raw_images/{i}.png")

master_yaml = f"""# Hand-Eye Calibration Data Configuration (1105 Left Arm 12 Groups)
# Camera: 1920x1080 (Scaled 0.5x from 4K), fx: {INTRINSICS_1080['fx']}, fy: {INTRINSICS_1080['fy']}, cx: {INTRINSICS_1080['cx']}, cy: {INTRINSICS_1080['cy']}
# Target: CGB-035 Version B (5 rows x 4 cols, spacing: 0.070m, diameter: 30mm)

{chr(10).join(data_entries)}

target_finder:
  type: ModifiedCircleGridTargetFinder
  asymmetric: true
  rows: 5
  cols: 4
  spacing: 0.070
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
    minArea: 300.0
    maxArea: 25000.0
    filterByCircularity: true
    minCircularity: 0.50
    maxCircularity: 99999999.0
    filterByInertia: false
    minInertiaRatio: 0.1
    maxInertiaRatio: 99999999.0
    filterByConvexity: false
    minConvexity: 0.1
    maxConvexity: 99999999.0
homography_threshold: 2.0

intrinsics:
  fx: {INTRINSICS_1080['fx']:.4f}
  fy: {INTRINSICS_1080['fy']:.4f}
  cx: {INTRINSICS_1080['cx']:.4f}
  cy: {INTRINSICS_1080['cy']:.4f}

camera_mount_to_camera_guess:
  x: 0.865716
  y: 0.162192
  z: 1.08997
  qx: 0.407986
  qy: 0.413434
  qz: -0.619875
  qw: 0.527613

target_mount_to_target_guess:
  x: -0.153524
  y: -0.028041
  z: 0.200254
  qx: 0.196868
  qy: 0.722712
  qz: -0.206581
  qw: 0.629488

static_camera: true
"""

cal_data_file = JOB_DIR / "cal_data.yaml"
with open(cal_data_file, "w", encoding="utf-8") as f:
    f.write(master_yaml)

print(f"\nSuccessfully generated master config: {cal_data_file}")
print("Container internal path: /workspace/data/handeye_1105/cal_data.yaml")

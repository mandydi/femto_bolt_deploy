#!/usr/bin/env python3
"""Print camera translation relative to fixed ArUco marker reference positions."""

import argparse
import json
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage


class LocationNode(Node):
    def __init__(self, reference, frames, hz, watch):
        super().__init__('aruco_location')
        self.reference = {int(k): np.asarray(v, dtype=np.float64)
                          for k, v in reference['markers'].items()}
        self.marker_size_m = float(reference['marker_size_m'])
        self.image_size = tuple(reference['image_size'])
        self.frame_id = reference['frame_id']
        self.required_frames = frames
        self.period_s = 1.0 / hz
        self.watch = watch
        self.samples = deque(maxlen=frames)
        self.camera_info = None
        self.last_sample_s = 0.0
        self.complete = False
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        )
        half = self.marker_size_m / 2
        self.object_points = np.asarray(
            [[-half, half, 0], [half, half, 0],
             [half, -half, 0], [-half, -half, 0]], dtype=np.float32
        )
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE, depth=1)
        self.create_subscription(CameraInfo, '/vision/top_camera/rgb/camera_info',
                                 self.on_info, qos)
        self.create_subscription(CompressedImage,
                                 '/vision/top_camera/color/image_raw/compressed',
                                 self.on_image, qos)

    def on_info(self, msg):
        self.camera_info = msg

    def on_image(self, msg):
        now = time.monotonic()
        if self.camera_info is None or now - self.last_sample_s < self.period_s:
            return
        self.last_sample_s = now
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return
        height, width = image.shape[:2]
        if (width, height) != self.image_size or (self.camera_info.width, self.camera_info.height) != self.image_size:
            self.get_logger().error('Image/CameraInfo resolution differs from reference')
            return
        if self.camera_info.header.frame_id != self.frame_id:
            self.get_logger().error('Camera frame differs from reference')
            return
        k = np.asarray(self.camera_info.k, dtype=np.float64).reshape(3, 3)
        d = np.asarray(self.camera_info.d, dtype=np.float64)
        corners, ids, _ = self.detector.detectMarkers(image)
        per_marker = []
        used_ids = []
        if ids is not None:
            for points, marker_id in zip(corners, ids.ravel()):
                marker_id = int(marker_id)
                if marker_id not in self.reference:
                    continue
                ok, rvec, tvec = cv2.solvePnP(
                    self.object_points, points.reshape(4, 2), k, d,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
                if not ok or not np.isfinite(tvec).all() or tvec[2, 0] <= 0:
                    continue
                projected, _ = cv2.projectPoints(self.object_points, rvec, tvec, k, d)
                error = np.linalg.norm(projected.reshape(4, 2) - points.reshape(4, 2), axis=1).mean()
                if error > 1.0:
                    continue
                per_marker.append((self.reference[marker_id] - tvec.ravel()) * 1000)
                used_ids.append(marker_id)
        if not per_marker:
            return
        offset = np.median(np.stack(per_marker), axis=0)
        self.samples.append((offset, used_ids))
        if len(self.samples) < self.required_frames:
            return
        filtered = np.median(np.stack([item[0] for item in self.samples]), axis=0)
        all_ids = sorted(set(i for _, marker_ids in self.samples for i in marker_ids))
        result = {
            'frame_id': self.frame_id,
            'offset_mm': [round(float(x), 2) for x in filtered],
            'marker_ids': all_ids,
            'sample_frames': len(self.samples),
            'stamp_sec': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'meaning': 'reference_marker_xyz_minus_current_marker_xyz',
        }
        print(json.dumps(result, ensure_ascii=False), flush=True)
        if not self.watch:
            self.complete = True


def main():
    parser = argparse.ArgumentParser(description='Report ArUco camera position offset in mm')
    parser.add_argument('--reference', required=True)
    parser.add_argument('--frames', type=int, default=5)
    parser.add_argument('--hz', type=float, default=2.0)
    parser.add_argument('--timeout', type=float, default=20.0)
    parser.add_argument('--watch', action='store_true')
    args = parser.parse_args()
    if args.frames < 1 or args.hz <= 0 or args.timeout <= 0:
        parser.error('frames, hz, and timeout must be positive')
    reference = json.loads(Path(args.reference).read_text(encoding='utf-8'))
    rclpy.init()
    node = LocationNode(reference, args.frames, args.hz, args.watch)
    started = time.monotonic()
    try:
        while rclpy.ok() and not node.complete:
            rclpy.spin_once(node, timeout_sec=0.2)
            if not args.watch and time.monotonic() - started > args.timeout:
                print('No usable ArUco offset before timeout', file=sys.stderr)
                return 1
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

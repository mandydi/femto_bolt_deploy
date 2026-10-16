#!/usr/bin/env python3
"""Report 2 cm ArUco marker poses in the RGB optical camera frame."""

import argparse
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage


class ArucoPoseNode(Node):
    def __init__(self, marker_size_m: float, max_hz: float):
        super().__init__('aruco_pose_live')
        self.marker_size_m = marker_size_m
        self.period_s = 1.0 / max_hz
        self.last_process_s = 0.0
        self.info = None
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(self.dictionary)
        h = marker_size_m / 2.0
        self.object_points = np.array(
            [[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]],
            dtype=np.float32,
        )
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.create_subscription(CameraInfo, '/vision/top_camera/rgb/camera_info', self.on_info, qos)
        self.create_subscription(
            CompressedImage,
            '/vision/top_camera/color/image_raw/compressed',
            self.on_image,
            qos,
        )

    def on_info(self, msg: CameraInfo):
        self.info = msg

    def on_image(self, msg: CompressedImage):
        now = time.monotonic()
        if self.info is None or now - self.last_process_s < self.period_s:
            return
        self.last_process_s = now
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return
        if (image.shape[1], image.shape[0]) != (self.info.width, self.info.height):
            self.get_logger().error('Image size differs from CameraInfo; skipping pose')
            return
        camera_matrix = np.asarray(self.info.k, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(self.info.d, dtype=np.float64)
        corners, ids, _ = self.detector.detectMarkers(image)
        poses = []
        if ids is not None:
            for points, marker_id in zip(corners, ids.ravel()):
                ok, rvec, tvec = cv2.solvePnP(
                    self.object_points,
                    points.reshape(4, 2),
                    camera_matrix,
                    distortion,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
                if not ok or not np.isfinite(tvec).all() or tvec[2, 0] <= 0:
                    continue
                projected, _ = cv2.projectPoints(
                    self.object_points, rvec, tvec, camera_matrix, distortion
                )
                error_px = np.linalg.norm(
                    projected.reshape(4, 2) - points.reshape(4, 2), axis=1
                ).mean()
                poses.append({
                    'id': int(marker_id),
                    'translation_m': [round(float(v), 5) for v in tvec.ravel()],
                    'rotation_vector_rad': [round(float(v), 5) for v in rvec.ravel()],
                    'reprojection_error_px': round(float(error_px), 3),
                })
        print(json.dumps({
            'stamp_sec': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'frame_id': self.info.header.frame_id,
            'image_size': [image.shape[1], image.shape[0]],
            'marker_size_m': self.marker_size_m,
            'poses': poses,
        }, ensure_ascii=False), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--marker-size-m', type=float, default=0.02)
    parser.add_argument('--max-hz', type=float, default=2.0)
    args = parser.parse_args()
    if args.marker_size_m <= 0 or args.max_hz <= 0:
        parser.error('marker-size-m and max-hz must be positive')
    rclpy.init()
    node = ArucoPoseNode(args.marker_size_m, args.max_hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

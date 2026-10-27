#!/usr/bin/env python3
"""Capture one RGB frame and the matching stream's CameraInfo for onsite setup."""
import argparse
import json
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--info', required=True)
    parser.add_argument('--timeout', type=float, default=20)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('top_camera_frame_capture')
    bridge = CvBridge()
    data = {'image': None, 'info': None}

    def on_image(msg):
        if data['image'] is None:
            data['image'] = msg

    def on_info(msg):
        if data['info'] is None:
            data['info'] = msg

    node.create_subscription(Image, '/vision/top_camera/rgb', on_image, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/vision/top_camera/rgb/camera_info', on_info,
                             qos_profile_sensor_data)
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline and (data['image'] is None or data['info'] is None):
            rclpy.spin_once(node, timeout_sec=0.2)
        if data['image'] is None or data['info'] is None:
            raise RuntimeError(f"RGB received={data['image'] is not None}; "
                               f"CameraInfo received={data['info'] is not None} before timeout")
        msg, info = data['image'], data['info']
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if not cv2.imwrite(args.output, frame, [cv2.IMWRITE_JPEG_QUALITY, 90]):
            raise RuntimeError('Could not write RGB snapshot')
        details = dict(width=msg.width, height=msg.height, encoding=msg.encoding,
                       frame_id=msg.header.frame_id, stamp_sec=msg.header.stamp.sec,
                       camera_info_width=info.width, camera_info_height=info.height,
                       distortion_model=info.distortion_model, d=list(info.d),
                       k=list(info.k), p=list(info.p))
        with open(args.info, 'w', encoding='utf-8') as stream:
            json.dump(details, stream, indent=2)
        print(json.dumps(details, ensure_ascii=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Save one timestamp-matched 4K RGB, aligned depth and point-cloud group."""
import os
import sys
import time
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9


def point_rows(msg):
    fields = {field.name: field for field in msg.fields}
    if any(fields[name].datatype != 7 for name in ('x', 'y', 'z')):
        raise ValueError('PointCloud2 x/y/z must be FLOAT32')
    if msg.row_step != msg.width * msg.point_step:
        raise ValueError('PointCloud2 row padding is not supported by this saver')
    endian = '>' if msg.is_bigendian else '<'
    dtype = np.dtype({'names': ['x', 'y', 'z'],
                      'formats': [endian + 'f4'] * 3,
                      'offsets': [fields[name].offset for name in ('x', 'y', 'z')],
                      'itemsize': msg.point_step})
    return np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)


def write_ply(msg, path):
    rows = point_rows(msg)
    def valid(chunk):
        xyz = np.column_stack((chunk['x'], chunk['y'], chunk['z']))
        keep = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0)
        return xyz[keep].astype('<f4', copy=False)
    step = 100_000
    count = sum(len(valid(rows[i:i + step])) for i in range(0, len(rows), step))
    with open(path, 'wb') as stream:
        header = (f'ply\nformat binary_little_endian 1.0\nelement vertex {count}\n'
                  'property float x\nproperty float y\nproperty float z\nend_header\n')
        stream.write(header.encode('ascii'))
        for i in range(0, len(rows), step):
            stream.write(valid(rows[i:i + step]).tobytes())
    return count


def main():
    if len(sys.argv) != 3 or not (sys.argv[2].isdigit() and len(sys.argv[2]) == 6):
        raise SystemExit('Usage: save_dataset_single.py OUTPUT_DIR SIX_DIGIT_INDEX')
    root, index = Path(sys.argv[1]), sys.argv[2]
    for folder in ('rgb', 'depth', 'point'):
        (root / folder).mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('femto_single_frame_saver')
    matched = [None]
    topics = ((Image, '/vision/top_camera/rgb'),
              (Image, '/vision/top_camera/depth'),
              (PointCloud2, '/vision/top_camera/points'))
    subs = [message_filters.Subscriber(node, typ, topic, qos_profile=10)
            for typ, topic in topics]
    sync = message_filters.ApproximateTimeSynchronizer(subs, 30, 0.5)
    sync.registerCallback(lambda rgb, depth, points: matched.__setitem__(0, (rgb, depth, points)))
    deadline = time.monotonic() + 30
    try:
        while matched[0] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if matched[0] is None:
            raise RuntimeError('No RGB/depth/point-cloud group within 500 ms in 30 seconds')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    rgb_msg, depth_msg, points_msg = matched[0]
    stamps = [stamp(msg) for msg in matched[0]]
    if max(stamps) - min(stamps) > 0.5:
        raise RuntimeError('Captured frames are not timestamp-matched')
    bridge = CvBridge()
    rgb = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
    depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
    if rgb.shape[:2] != (2160, 3840) or depth.shape[:2] != (2160, 3840) or depth.dtype != np.uint16:
        raise ValueError(f'Unexpected RGB/depth shape or dtype: {rgb.shape}, {depth.shape}, {depth.dtype}')
    destinations = [root / 'rgb' / f'{index}.png',
                    root / 'depth' / f'{index}.png',
                    root / 'point' / f'{index}.ply']
    temporary = [path.with_name(path.stem + '.tmp' + path.suffix) for path in destinations]
    try:
        if not cv2.imwrite(str(temporary[0]), rgb):
            raise RuntimeError('RGB PNG write failed')
        if not cv2.imwrite(str(temporary[1]), depth):
            raise RuntimeError('Depth PNG write failed')
        count = write_ply(points_msg, temporary[2])
        if count == 0:
            raise RuntimeError('Point cloud has no valid points')
        for src, dest in zip(temporary, destinations):
            os.replace(src, dest)
            if 'CAPTURE_UID' in os.environ:
                os.chown(dest, int(os.environ['CAPTURE_UID']), int(os.environ['CAPTURE_GID']))
    finally:
        for path in temporary:
            path.unlink(missing_ok=True)
    print(f'SAVED {index}: RGB {rgb.shape[1]}x{rgb.shape[0]}, '
          f'depth {depth.shape[1]}x{depth.shape[0]} {depth.dtype}, '
          f'cloud {count} points; timestamp spread {max(stamps)-min(stamps):.3f}s', flush=True)


if __name__ == '__main__':
    main()

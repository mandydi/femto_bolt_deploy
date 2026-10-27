#!/usr/bin/env python3
"""Short read-only diagnostic of the three vision topic timestamps."""
import time

import message_filters
import rclpy
from sensor_msgs.msg import Image, PointCloud2


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9


rclpy.init()
node = rclpy.create_node('vision_sync_probe')
counts = [0, 0, 0]
last = [None, None, None]
sync_counts = {0.08: 0, 0.5: 0, 1.0: 0}
topics = [('/vision/top_camera/rgb', Image),
          ('/vision/top_camera/depth', Image),
          ('/vision/top_camera/points', PointCloud2)]
subs = [message_filters.Subscriber(node, typ, topic, qos_profile=10)
        for topic, typ in topics]
for i, sub in enumerate(subs):
    def note(msg, idx=i):
        counts[idx] += 1
        last[idx] = stamp(msg)
    sub.registerCallback(note)
for slop in sync_counts:
    sync = message_filters.ApproximateTimeSynchronizer(subs, 30, slop)
    def matched(a, b, c, window=slop):
        sync_counts[window] += 1
    sync.registerCallback(matched)
deadline = time.monotonic() + 15
try:
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    print('counts=', counts, 'last_stamps=', last, 'sync=', sync_counts, flush=True)
    if all(x is not None for x in last):
        print('last_stamp_spread_sec=', round(max(last) - min(last), 3), flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import time

rclpy.init()
node = Node("perf_test")

def cb(msg):
    t0 = time.time()
    print("PointCloud received: {}x{}, point_step={}".format(msg.width, msg.height, msg.point_step))
    
    if msg.point_step == 12:
        pts = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3)
    else:
        pad = msg.point_step - 12
        dtype = np.dtype([("xyz", np.float32, (3,)), ("pad", np.uint8, (pad,))])
        arr = np.frombuffer(msg.data, dtype=dtype)
        pts = arr["xyz"]
    
    t1 = time.time()
    mask = ~np.isnan(pts).any(axis=1) & ~(pts == 0).all(axis=1)
    valid_pts = pts[mask]
    t2 = time.time()
    print("Numpy parse: {:.1f}ms, filter: {:.1f}ms, valid count: {}".format((t1-t0)*1000, (t2-t1)*1000, len(valid_pts)))
    
    t3 = time.time()
    ply_path = "/tmp/test_pc.ply"
    header = "ply\nformat ascii 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nend_header\n".format(len(valid_pts))
    with open(ply_path, "wb") as f:
        f.write(header.encode('ascii'))
        # Fast write using savetxt or format
        np.savetxt(f, valid_pts, fmt="%.4f", delimiter=" ")
    t4 = time.time()
    print("Write file time: {:.1f}ms".format((t4-t3)*1000))
    
    node.destroy_node()
    rclpy.shutdown()

sub = node.create_subscription(PointCloud2, "/vision/top_camera/points", cb, 1)
try:
    rclpy.spin(node)
except Exception:
    pass

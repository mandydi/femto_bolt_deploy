import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct
import sys
import os

class DatasetSaver(Node):
    def __init__(self, out_dir):
        super().__init__('femto_dataset_saver')
        self.out_dir = out_dir
        self.bridge = CvBridge()
        self.got_rgb = False
        self.got_depth = False
        self.got_points = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.sub_rgb = self.create_subscription(
            CompressedImage,
            '/vision/top_camera/color/image_raw/compressed',
            self.cb_rgb,
            qos
        )
        self.sub_depth = self.create_subscription(
            Image,
            '/vision/top_camera/depth',
            self.cb_depth,
            qos
        )
        self.sub_points = self.create_subscription(
            PointCloud2,
            '/vision/top_camera/points',
            self.cb_points,
            qos
        )

    def cb_rgb(self, msg):
        if self.got_rgb:
            return
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imwrite(os.path.join(self.out_dir, 'rgb.png'), img)
        self.got_rgb = True
        print("[OK] RGB saved: " + str(img.shape))
        self.check_done()

    def cb_depth(self, msg):
        if self.got_depth:
            return
        depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # Save raw 16-bit depth PNG
        cv2.imwrite(os.path.join(self.out_dir, 'depth.png'), depth_img)
        # Also save normalized visualization for easy view
        depth_norm = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
        cv2.imwrite(os.path.join(self.out_dir, 'depth_vis.png'), depth_color)
        np.save(os.path.join(self.out_dir, 'depth.npy'), depth_img)
        self.got_depth = True
        print("[OK] Depth saved: " + str(depth_img.shape))
        self.check_done()

    def cb_points(self, msg):
        if self.got_points:
            return
        point_step = msg.point_step
        data = msg.data
        points = []
        for i in range(0, len(data), point_step):
            x, y, z = struct.unpack_from('fff', data, i)
            if not (np.isnan(x) or np.isnan(y) or np.isnan(z) or (x == 0 and y == 0 and z == 0)):
                points.append((x, y, z))
        
        ply_path = os.path.join(self.out_dir, 'pointcloud.ply')
        with open(ply_path, 'w') as f:
            f.write("ply\nformat ascii 1.0\n")
            f.write("element vertex " + str(len(points)) + "\n")
            f.write("property float x\nproperty float y\nproperty float z\n")
            f.write("end_header\n")
            for p in points:
                f.write("{:.4f} {:.4f} {:.4f}\n".format(p[0], p[1], p[2]))
        
        self.got_points = True
        print("[OK] PointCloud saved: " + str(len(points)) + " points -> pointcloud.ply")
        self.check_done()

    def check_done(self):
        if self.got_rgb and self.got_depth and self.got_points:
            print("[SUCCESS] All 3 modalities (RGB, Depth, PointCloud) captured!")
            sys.exit(0)

if __name__ == '__main__':
    out_dir = '/tmp/captured_frame'
    os.makedirs(out_dir, exist_ok=True)
    rclpy.init()
    node = DatasetSaver(out_dir)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    rclpy.shutdown()

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
import time

class ContinuousDatasetSaver(Node):
    def __init__(self, out_dir, interval=2.0):
        super().__init__('femto_continuous_dataset_saver')
        self.out_dir = out_dir
        self.interval = float(interval)
        self.bridge = CvBridge()

        self.rgb_dir = os.path.join(out_dir, 'rgb')
        self.depth_dir = os.path.join(out_dir, 'depth')
        self.point_dir = os.path.join(out_dir, 'point')

        os.makedirs(self.rgb_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)
        os.makedirs(self.point_dir, exist_ok=True)

        # Detect existing frame count to continue numbering seamlessly
        existing_frames = [
            int(f.split('.')[0]) for f in os.listdir(self.rgb_dir)
            if f.split('.')[0].isdigit()
        ]
        self.frame_idx = max(existing_frames) if existing_frames else 0

        self.latest_rgb = None
        self.latest_depth = None
        self.latest_points = None

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

        self.get_logger().info(
            f"Continuous capture initialized. Saving to: {self.out_dir} every {self.interval}s (starting from frame #{self.frame_idx + 1:06d})"
        )

        # Timer to trigger periodic capture every interval seconds
        self.timer = self.create_timer(self.interval, self.timer_capture_tick)

    def cb_rgb(self, msg):
        self.latest_rgb = msg

    def cb_depth(self, msg):
        self.latest_depth = msg

    def cb_points(self, msg):
        self.latest_points = msg

    def timer_capture_tick(self):
        if self.latest_rgb is None or self.latest_depth is None or self.latest_points is None:
            self.get_logger().warn("Waiting for all 3 topics (RGB, Depth, PointCloud) to arrive...")
            return

        self.frame_idx += 1
        name_base = f"{self.frame_idx:06d}"
        t_start = time.time()

        # 1. Save 4K RGB
        try:
            np_arr = np.frombuffer(self.latest_rgb.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            rgb_path = os.path.join(self.rgb_dir, f"{name_base}.png")
            cv2.imwrite(rgb_path, img)
            try: os.chmod(rgb_path, 0o666)
            except Exception: pass
            rgb_shape = img.shape
        except Exception as e:
            self.get_logger().error(f"Failed to save RGB: {e}")
            return

        # 2. Save 16-bit D2C Depth
        try:
            depth_img = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding='passthrough')
            depth_path = os.path.join(self.depth_dir, f"{name_base}.png")
            cv2.imwrite(depth_path, depth_img)
            try: os.chmod(depth_path, 0o666)
            except Exception: pass
            depth_shape = depth_img.shape
        except Exception as e:
            self.get_logger().error(f"Failed to save Depth: {e}")
            return

        # 3. Save 3D PointCloud (ASCII PLY)
        try:
            msg = self.latest_points
            point_step = msg.point_step
            data = msg.data
            points = []
            for i in range(0, len(data), point_step):
                x, y, z = struct.unpack_from('fff', data, i)
                if not (np.isnan(x) or np.isnan(y) or np.isnan(z) or (x == 0 and y == 0 and z == 0)):
                    points.append((x, y, z))

            ply_path = os.path.join(self.point_dir, f"{name_base}.ply")
            with open(ply_path, 'w') as f:
                f.write(f"ply\nformat ascii 1.0\nelement vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n")
                for p in points:
                    f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
            try: os.chmod(ply_path, 0o666)
            except Exception: pass
        except Exception as e:
            self.get_logger().error(f"Failed to save PointCloud: {e}")
            return

        elapsed = time.time() - t_start
        print(
            f"[FRAME {name_base}] Captured: RGB {rgb_shape[1]}x{rgb_shape[0]} | "
            f"Depth {depth_shape[1]}x{depth_shape[0]} (D2C) | "
            f"Points: {len(points):,} pts (cost: {elapsed:.2f}s)",
            flush=True
        )

def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else '/home/caseim/yolo26_packet/captured_frame'
    interval = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

    rclpy.init()
    node = ContinuousDatasetSaver(out_dir, interval)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nCapture stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

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

class SingleFrameDatasetSaver(Node):
    def __init__(self, out_dir):
        super().__init__('femto_single_dataset_saver')
        self.out_dir = out_dir
        self.bridge = CvBridge()

        self.rgb_dir = os.path.join(out_dir, 'rgb')
        self.depth_dir = os.path.join(out_dir, 'depth')
        self.point_dir = os.path.join(out_dir, 'point')

        os.makedirs(self.rgb_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)
        os.makedirs(self.point_dir, exist_ok=True)

        # Automatically determine next sequence number from existing files
        existing_frames = []
        for d in [self.rgb_dir, self.depth_dir, self.point_dir]:
            if os.path.exists(d):
                for f in os.listdir(d):
                    stem = f.split('.')[0]
                    if stem.isdigit():
                        existing_frames.append(int(stem))

        self.frame_idx = (max(existing_frames) + 1) if existing_frames else 1
        self.name_base = f"{self.frame_idx:06d}"

        self.latest_rgb = None
        self.latest_depth = None
        self.latest_points = None
        self.done = False

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
            f"Waiting for topics to capture single frame #{self.name_base}..."
        )

        # Check loop timer
        self.timer = self.create_timer(0.2, self.check_and_save)

    def cb_rgb(self, msg):
        self.latest_rgb = msg

    def cb_depth(self, msg):
        self.latest_depth = msg

    def cb_points(self, msg):
        self.latest_points = msg

    def check_and_save(self):
        if self.done:
            return

        if self.latest_rgb is None or self.latest_depth is None or self.latest_points is None:
            return

        self.done = True
        t_start = time.time()
        print(f"\n[INFO] Starting single frame save for #{self.name_base}...", flush=True)

        # 1. Save 4K RGB
        try:
            np_arr = np.frombuffer(self.latest_rgb.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            rgb_path = os.path.join(self.rgb_dir, f"{self.name_base}.png")
            cv2.imwrite(rgb_path, img)
            try: os.chmod(rgb_path, 0o666)
            except Exception: pass
            rgb_shape = img.shape
        except Exception as e:
            self.get_logger().error(f"Failed to save RGB: {e}")
            self.done = False
            return

        # 2. Save 16-bit D2C Depth
        try:
            depth_img = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding='passthrough')
            depth_path = os.path.join(self.depth_dir, f"{self.name_base}.png")
            cv2.imwrite(depth_path, depth_img)
            try: os.chmod(depth_path, 0o666)
            except Exception: pass
            depth_shape = depth_img.shape
        except Exception as e:
            self.get_logger().error(f"Failed to save Depth: {e}")
            self.done = False
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

            ply_path = os.path.join(self.point_dir, f"{self.name_base}.ply")
            with open(ply_path, 'w') as f:
                f.write(f"ply\nformat ascii 1.0\nelement vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n")
                for p in points:
                    f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
            try: os.chmod(ply_path, 0o666)
            except Exception: pass
        except Exception as e:
            self.get_logger().error(f"Failed to save PointCloud: {e}")
            self.done = False
            return

        elapsed = time.time() - t_start
        print(f"=========================================================", flush=True)
        print(f"[SUCCESS] Single frame #{self.name_base} saved successfully! (Time cost: {elapsed:.2f}s)", flush=True)
        print(f"  - RGB:   rgb/{self.name_base}.png ({rgb_shape[1]}x{rgb_shape[0]})", flush=True)
        print(f"  - Depth: depth/{self.name_base}.png ({depth_shape[1]}x{depth_shape[0]} 16-bit D2C)", flush=True)
        print(f"  - Point: point/{self.name_base}.ply ({len(points):,} points)", flush=True)
        print(f"=========================================================", flush=True)

        # Trigger exit
        sys.exit(0)

def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else '/home/caseim/yolo26_packet/captured_frame'
    rclpy.init()
    node = SingleFrameDatasetSaver(out_dir)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()

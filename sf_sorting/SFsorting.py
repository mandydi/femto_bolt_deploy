import json
import os
import sys
import time
import argparse
import cv2
import numpy as np
import open3d as o3d

_PLY_CACHE = {}

def clear_ply_cache():
    """清空内存中的点云缓存"""
    global _PLY_CACHE
    _PLY_CACHE.clear()

def numpy_voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """
    纯NumPy实现体素降采样，每个体素保留质心点
    :param points: N×3 点云坐标
    :param voxel_size: 体素边长（单位与点云一致）
    :return: 降采样后的点云 M×3
    """
    voxel_idx = np.floor(points / voxel_size).astype(np.int32)
    unique_voxels, inverse_idx = np.unique(voxel_idx, axis=0, return_inverse=True)

    sum_x = np.bincount(inverse_idx, weights=points[:, 0])
    sum_y = np.bincount(inverse_idx, weights=points[:, 1])
    sum_z = np.bincount(inverse_idx, weights=points[:, 2])
    count = np.bincount(inverse_idx)

    return np.column_stack([sum_x, sum_y, sum_z]) / count[:, None]


# ===================== 三级加载机制（内存→磁盘npy→原始PLY） =====================
def load_downsampled_ply(ply_path: str, voxel_mm: float) -> np.ndarray:
    """
    三级加载降采样点云：
    1. 内存缓存命中 → 直接返回
    2. 磁盘npy缓存命中 → 加载到内存并返回
    3. 都不命中 → 读取原始PLY + 体素降采样 + 双缓存持久化
    :return: 降采样后的点云 N×3 float32
    """
    cache_key = (ply_path, voxel_mm)

    if cache_key in _PLY_CACHE:
        return _PLY_CACHE[cache_key]
    npy_path = f"{ply_path}.voxel{voxel_mm:.1f}.npy"
    if os.path.exists(npy_path):
        pts_all = np.load(npy_path).astype(np.float32)
        _PLY_CACHE[cache_key] = pts_all
        return pts_all
    pcd_raw = o3d.io.read_point_cloud(ply_path)
    pts_raw = np.asarray(pcd_raw.points, dtype=np.float32)
    pts_all = numpy_voxel_downsample(pts_raw, voxel_mm)
    np.save(npy_path, pts_all)
    _PLY_CACHE[cache_key] = pts_all
    return pts_all


def polygon_to_mask(polygon_points, img_shape):
    """
    polygon_points: [[x0,y0],[x1,y1],...] list
    img_shape: (H,W)
    返回二值mask uint8，0/255
    """
    mask = np.zeros(img_shape[:2], dtype=np.uint8)
    pts = np.array(polygon_points, dtype=np.int32).reshape((-1, 1, 2))
    cv2.fillPoly(mask, [pts], 255)
    return mask


def get_polygon_centroid_inside(poly_points):
    """
    计算多边形质心，保证质心在多边形内部
    poly_points: list[[x,y],...]
    return (cx, cy) float
    """
    from shapely.geometry import Polygon
    poly = Polygon(poly_points)
    centroid = poly.centroid
    if not poly.contains(centroid):
        pt = poly.representative_point()
        return (pt.x, pt.y)
    return (centroid.x, centroid.y)


def undistort_pixel_point(u, v, camera_matrix, dist_coeffs):
    """单个畸变像素去畸变，返回归一化平面坐标 (x_norm, y_norm)"""
    pts = np.array([[[u, v]]], dtype=np.float64)
    undist_pts = cv2.undistortPoints(pts, camera_matrix, dist_coeffs)
    xn, yn = undist_pts[0, 0]
    return xn, yn


def pixel2world_with_distortion(u, v, depth_mm, camera_matrix, dist_coeffs):
    """畸变像素(u,v)+深度 → 相机坐标系3D点mm"""
    if depth_mm <= 0:
        return None
    xn, yn = undistort_pixel_point(u, v, camera_matrix, dist_coeffs)
    Z = depth_mm
    X = xn * Z
    Y = yn * Z
    return (X, Y, Z)


def fit_plane_svd(points_xyz: np.ndarray, dist_thresh_mm: float = 1.0, ransac_iter: int = 800):
    """
    基于Open3D RANSAC拟合平面，返回(a,b,c,d)，法向量单位化
    points_xyz: N×3 numpy array (mm)
    return (a,b,c,d) or None
    """
    n_points = points_xyz.shape[0]
    if n_points < 4:
        print(f"点数量不足，只有{n_points}个点，无法拟合平面")
        return None
    pcd_roi = o3d.geometry.PointCloud()
    pcd_roi.points = o3d.utility.Vector3dVector(points_xyz)
    plane_model, inlier_idx = pcd_roi.segment_plane(
        distance_threshold=dist_thresh_mm,
        ransac_n=3,
        num_iterations=ransac_iter
    )
    a, b, c, d = plane_model
    norm_len = np.linalg.norm([a, b, c])
    if norm_len < 1e-9:
        return None
    a /= norm_len
    b /= norm_len
    c /= norm_len
    return (float(a), float(b), float(c), float(d))


def calc_plane_xy_angles(plane_coeff):
    """
    计算平面方程在x、y方向的坡度倾斜角（相对于xy水平面）
    :param plane_coeff: 平面方程系数 (a, b, c, d)，满足 ax + by + cz + d = 0
    :return: (theta_x_deg, theta_y_deg)  x方向夹角、y方向夹角，单位：度
    """
    a, b, c, d = plane_coeff

    if abs(c) < 1e-9:
        return 90.0, 90.0

    tan_theta_x = abs(a / c)
    theta_x_deg = np.degrees(np.arctan(tan_theta_x))

    tan_theta_y = abs(b / c)
    theta_y_deg = np.degrees(np.arctan(tan_theta_y))

    return theta_x_deg, theta_y_deg


def get_mask_outline_polygon(mask: np.ndarray):
    """从二值mask提取最外层外包轮廓，返回像素坐标轮廓点(N,2)"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        return None
    max_cnt = max(contours, key=cv2.contourArea)
    outline_px = max_cnt.reshape(-1, 2)
    return outline_px


def get_prism_inlier_from_ply(
        mask_1_2_mini: np.ndarray,
        ply_path: str,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        z_offset_range: float = 150.0,
        voxel_mm: float = 3.0,
        max_roi_pts: int = 3000
):
    """
    构造Z向多棱柱，筛选PLY点云落在棱柱内部的点
    :return prism_inlier_points(N×3), outline_px(N,2), z_low, z_high；失败返回None,None,None,None
    """
    outline_px = get_mask_outline_polygon(mask_1_2_mini)
    if outline_px is None or len(outline_px) < 3:
        print("mask_1_2_mini提取不到有效轮廓")
        return None, None, None, None

    if isinstance(ply_path, str):
        pts_all = load_downsampled_ply(ply_path, voxel_mm)
    elif isinstance(ply_path, np.ndarray):
        if voxel_mm > 0 and ply_path.shape[0] > 0:
            pts_all = numpy_voxel_downsample(ply_path.astype(np.float32), voxel_mm)
        else:
            pts_all = ply_path.astype(np.float32)
    else:
        print(f"不支持的点云格式: {type(ply_path)}")
        return None, outline_px, None, None

    if pts_all.shape[0] < 10:
        print("ply点云点数太少")
        return None, outline_px, None, None

    X_all, Y_all, Z_all = pts_all[:, 0], pts_all[:, 1], pts_all[:, 2]
    valid_z_mask = Z_all > 1e-3
    if not np.any(valid_z_mask):
        return None, outline_px, None, None

    Xv = X_all[valid_z_mask]
    Yv = Y_all[valid_z_mask]
    Zv = Z_all[valid_z_mask]

    xn = Xv / Zv
    yn = Yv / Zv
    pts_norm = np.column_stack([xn, yn, np.ones_like(xn)])
    dummy_rvec = np.zeros(3)
    dummy_tvec = np.zeros(3)
    proj_pixels, _ = cv2.projectPoints(pts_norm, dummy_rvec, dummy_tvec, camera_matrix, dist_coeffs)
    proj_uv = proj_pixels[:, 0, :]

    # 向量化批量判断点是否在多边形内
    proj_u = np.clip(np.round(proj_uv[:, 0]).astype(np.int32), 0, mask_1_2_mini.shape[1] - 1)
    proj_v = np.clip(np.round(proj_uv[:, 1]).astype(np.int32), 0, mask_1_2_mini.shape[0] - 1)
    inside = mask_1_2_mini[proj_v, proj_u] > 0

    pts_candidate = np.column_stack([Xv, Yv, Zv])[inside]
    if pts_candidate.shape[0] < 4:
        print("棱柱候选区域内ply点云过少")
        return None, outline_px, None, None

    z_mean = float(np.mean(pts_candidate[:, 2]))
    z_low = z_mean - z_offset_range
    z_high = z_mean + z_offset_range
    z_prism_mask = (pts_candidate[:, 2] >= z_low) & (pts_candidate[:, 2] <= z_high)
    prism_inlier_points = pts_candidate[z_prism_mask]

    if len(prism_inlier_points) > max_roi_pts:
        idx_sel = np.random.choice(len(prism_inlier_points), max_roi_pts, replace=False)
        prism_inlier_points = prism_inlier_points[idx_sel]

    return prism_inlier_points, outline_px, z_low, z_high


def get_shrunk_mask_prism_fit_plane(
        poly_mask: np.ndarray,
        centroid_px: tuple,
        ply_path: str,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        z_offset_range: float = 150.0,
        save_mini_mask_path: str = None,
        voxel_mm: float = 3.0,
        max_roi_pts: int = 3000
):
    """
    缩小mask构造棱柱并拟合平面
    return (mask_1_2_mini, plane_coeff, prism_points, outline_px, z_low, z_high)
    """
    H, W = poly_mask.shape[:2]
    u0, v0 = centroid_px
    ys, xs = np.where(poly_mask > 0)
    if len(xs) == 0:
        print("原始mask没有有效像素")
        return np.zeros_like(poly_mask), None, None, None, None, None

    xmin, xmax = xs.min(), xs.max()
    ymin, ymax = ys.min(), ys.max()
    orig_w = xmax - xmin
    orig_h = ymax - ymin
    new_w = int(round(orig_w / 2.0))
    new_h = int(round(orig_h / 2.0))
    half_w = new_w // 2
    half_h = new_h // 2

    roi_x1 = np.clip(u0 - half_w, 0, W - 1)
    roi_x2 = np.clip(u0 + half_w, 0, W - 1)
    roi_y1 = np.clip(v0 - half_h, 0, H - 1)
    roi_y2 = np.clip(v0 + half_h, 0, H - 1)

    mask_1_2_mini = np.zeros_like(poly_mask)
    mask_1_2_mini[roi_y1:roi_y2 + 1, roi_x1:roi_x2 + 1] = poly_mask[roi_y1:roi_y2 + 1, roi_x1:roi_x2 + 1]

    # 仅当指定路径时才保存mini mask
    if save_mini_mask_path is not None:
        cv2.imwrite(save_mini_mask_path, mask_1_2_mini)

    prism_points, outline_px, z_low, z_high = get_prism_inlier_from_ply(
        mask_1_2_mini=mask_1_2_mini,
        ply_path=ply_path,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        z_offset_range=z_offset_range,
        voxel_mm = voxel_mm,
        max_roi_pts = max_roi_pts
    )
    if prism_points is None or prism_points.shape[0] < 4:
        return mask_1_2_mini, None, prism_points, outline_px, z_low, z_high

    plane_coeff = fit_plane_svd(prism_points)
    return mask_1_2_mini, plane_coeff, prism_points, outline_px, z_low, z_high


def process_one_sample(
        png_path: str,
        depth_path: str,
        ply_path: str,
        fx: float, fy: float, cx: float, cy: float,
        dist_coeffs: np.ndarray,
        mask_path: str,
        shapes_list: list,
        z_prism_offset: float = 150.0,
        voxel_mm: float = 3.0,
        max_roi_pts: int = 3000
):
    """
    单样本平面拟合主函数
    :param png_path: 彩色图路径
    :param depth_path: 深度图路径
    :param ply_path: 点云PLY路径
    :param fx, fy, cx, cy: 相机内参
    :param dist_coeffs: 畸变系数数组
    :param mask4_path: 本地mask4图片路径（二值图）
    :param shapes_list: 标注列表，格式为 [{'label': 'xxx', 'points': [[x,y],...]}, ...]
    :param z_prism_offset: Z向棱柱半厚度，单位mm
    :param voxel_mm: 点云体素降采样尺寸，单位mm
    :param max_roi_pts: 送入RANSAC的最大点数
    :return: (min_xyz, plane_coeff, prism_points, theta_x_deg, theta_y_deg)
             失败对应位置返回None
    """
    # 1. 读取彩色图
    if isinstance(png_path, str):
        img = cv2.imread(png_path)
        if img is None:
            raise FileNotFoundError(f"彩色图读取失败 {png_path}")
    else:
        img = png_path
    H, W = img.shape[:2]

    # 2. 读取本地mask4（替代原JSON生成label=4逻辑）
    if isinstance(mask_path, str):
        mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
        if mask is None:
            raise FileNotFoundError(f"mask4图片读取失败 {mask_path}")
    else:
        mask = mask_path.copy()
    # 确保二值化
    mask = (mask > 0).astype(np.uint8) * 255

    # 3. 相机内参矩阵
    camera_matrix = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ], dtype=np.float64)

    # 4. 处理标注列表：carton/soft_bag/foam 对应 mask_1_2
    mask_1_2 = np.zeros((H, W), dtype=np.uint8)
    poly_12_list = []
    # 定义属于1/2类的标签集合
    LABEL_12_SET = {'carton', 'soft_bag', 'foam'}

    for shape in shapes_list:
        label = shape.get('label', '')
        pts = shape.get('points', [])
        if label in LABEL_12_SET and len(pts) >= 3:
            m = polygon_to_mask(pts, (H, W))
            mask_1_2 = cv2.bitwise_or(mask_1_2, m)
            poly_12_list.append({"points": pts, "label": label})

    # 5. 交集判断
    intersect_mask = cv2.bitwise_and(mask_1_2, mask)
    if not np.any(intersect_mask > 0):
        print("mask_1_2 与 mask 无交集")
        return None, None, None, None, None

    # 6. 读取深度图
    if isinstance(depth_path, str):
        depth_raw = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        if depth_raw is None:
            raise FileNotFoundError(f"深度图读取失败 {depth_path}")
        depth_np = depth_raw.astype(np.float32)
    else:
        depth_np = depth_path.astype(np.float32)

    # 7. 遍历1/2类多边形，找Z最小的有效点
    valid_records = []
    for item in poly_12_list:
        poly_pts = item["points"]
        poly_mask = polygon_to_mask(poly_pts, (H, W))
        inter = cv2.bitwise_and(poly_mask, mask)
        if not np.any(inter > 0):
            continue

        cx_px, cy_px = get_polygon_centroid_inside(poly_pts)
        cx_px = np.clip(cx_px, 0, W - 1)
        cy_px = np.clip(cy_px, 0, H - 1)
        u = int(round(cx_px))
        v = int(round(cy_px))

        d_val = depth_np[v, u]
        xyz = pixel2world_with_distortion(u, v, d_val, camera_matrix, dist_coeffs)
        if xyz is not None:
            valid_records.append({"xyz": xyz, "u": u, "v": v, "poly_mask": poly_mask})

    if len(valid_records) == 0:
        print("相交的多边形没有有效深度")
        return None, None, None, None, None

    # 8. 取Z最小的多边形做平面拟合
    valid_records.sort(key=lambda r: r["xyz"][2])
    best_rec = valid_records[0]
    min_xyz = best_rec["xyz"]
    best_poly_mask = best_rec["poly_mask"]
    best_u, best_v = best_rec["u"], best_rec["v"]

    mask_1_2_mini, plane_coeff, prism_points, outline_px, z_low, z_high = get_shrunk_mask_prism_fit_plane(
        poly_mask=best_poly_mask,
        centroid_px=(best_u, best_v),
        ply_path=ply_path,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        z_offset_range=z_prism_offset,
        voxel_mm=voxel_mm,
        max_roi_pts=max_roi_pts
    )

    # 9. 计算平面夹角
    theta_x_deg, theta_y_deg = None, None
    if plane_coeff is not None:
        a, b, c, d = plane_coeff
        print("\n=====拟合平面结果=====")
        print(f"棱柱内PLY点数: {prism_points.shape[0]}")
        print(f"Z棱柱区间: z_low={z_low:.2f} mm , z_high={z_high:.2f} mm")
        print(f"平面方程: {a:.4f}*x + {b:.4f}*y + {c:.4f}*z + {d:.4f} = 0")
        theta_x_deg, theta_y_deg = calc_plane_xy_angles(plane_coeff)
        print(f"x方向倾斜角: {theta_x_deg:.4f} °")
        print(f"y方向倾斜角: {theta_y_deg:.4f} °")
    return min_xyz, plane_coeff, prism_points, theta_x_deg, theta_y_deg


def extract_pointcloud2_xyz_mm(cloud_msg):
    """
    高效解析 ROS2 sensor_msgs/PointCloud2 消息中的 (x, y, z)，并转换为毫米(mm)
    """
    num_points = cloud_msg.width * cloud_msg.height
    if num_points == 0 or len(cloud_msg.data) == 0:
        return np.empty((0, 3), dtype=np.float32)
    raw_bytes = np.frombuffer(cloud_msg.data, dtype=np.uint8)
    step = cloud_msg.point_step
    raw_reshaped = raw_bytes[:num_points * step].reshape((num_points, step))
    # 点云 x, y, z 位于前 12 字节（float32，单位米）
    pts_m = np.ascontiguousarray(raw_reshaped[:, :12]).view(np.float32).reshape((num_points, 3))
    pts_mm = pts_m * 1000.0
    valid_mask = np.isfinite(pts_mm).all(axis=1) & (pts_mm[:, 2] > 10.0)
    return pts_mm[valid_mask]


class SFSortingROS2Node:
    """
    ROS 2 实时订阅节点：
    订阅 /vision/top_camera/rgb, /vision/top_camera/depth, /vision/top_camera/points
    通过时间同步器 ApproximateTimeSynchronizer 触发 YOLO 目标检测与平面拟合分拣
    """
    def __init__(self, mask_path="mask.png", fx=2249.6287, fy=2248.4431, cx=1877.3463, cy=1073.7207, dist_coeffs=None, loop=False, z_offset=120.0):
        import rclpy
        from sensor_msgs.msg import Image, PointCloud2
        import message_filters
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

        self.mask_path = mask_path
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.array([
            0.080384, -0.107753, -0.000413, 0.000256, 0.044622, 0.0, 0.0, 0.0
        ], dtype=np.float64)
        self.loop = loop
        self.z_offset = z_offset
        self.processing = False
        self.latest_result = None

        self.node = rclpy.create_node('sf_sorting_subscriber_node')
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, depth=5)

        self.sub_rgb = message_filters.Subscriber(self.node, Image, '/vision/top_camera/rgb', qos_profile=qos)
        self.sub_depth = message_filters.Subscriber(self.node, Image, '/vision/top_camera/depth', qos_profile=qos)
        self.sub_points = message_filters.Subscriber(self.node, PointCloud2, '/vision/top_camera/points', qos_profile=qos)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth, self.sub_points],
            queue_size=10,
            slop=0.08
        )
        self.sync.registerCallback(self.topic_callback)
        print("已成功订阅 /vision/top_camera 实时话题 [rgb, depth, points]，等待时间戳同步接收...", flush=True)

    def topic_callback(self, rgb_msg, depth_msg, points_msg):
        if self.processing:
            return
        self.processing = True
        t0 = time.time()
        try:
            print(f"\n[ROS2 实时同步帧] 时间戳: {rgb_msg.header.stamp.sec}.{rgb_msg.header.stamp.nanosec:09d}")
            print(f"  RGB: {rgb_msg.width}x{rgb_msg.height} ({rgb_msg.encoding})")
            print(f"  Depth: {depth_msg.width}x{depth_msg.height} ({depth_msg.encoding})")
            print(f"  Points: {points_msg.width * points_msg.height} 点")

            # 1. 转换彩色图 (rgb8 -> BGR)
            if rgb_msg.encoding == 'rgb8':
                rgb_arr = np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape((rgb_msg.height, rgb_msg.width, 3))
                bgr_img = cv2.cvtColor(rgb_arr, cv2.COLOR_RGB2BGR)
            elif rgb_msg.encoding == 'bgr8':
                bgr_img = np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape((rgb_msg.height, rgb_msg.width, 3)).copy()
            else:
                bgr_img = np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape((rgb_msg.height, rgb_msg.width, -1))

            # 2. 转换深度图 (16UC1 -> mm float32)
            depth_np = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape((depth_msg.height, depth_msg.width)).astype(np.float32)

            # 3. 转换点云 (米 -> 毫米 mm)
            pts_mm = extract_pointcloud2_xyz_mm(points_msg)
            print(f"  有效点云点数 (>10mm): {len(pts_mm)}")

            # 4. YOLO 目标检测
            from predict_yolo import yoloDetect
            yolo_list = yoloDetect(bgr_img)
            print(f"  YOLO 检测完成，共识别 {len(yolo_list)} 个包裹目标")

            # 5. 执行平面拟合与抓取点定位
            clear_ply_cache()
            min_xyz, plane_coeff, prism_points, theta_x, theta_y = process_one_sample(
                png_path=bgr_img,
                depth_path=depth_np,
                ply_path=pts_mm,
                fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy,
                dist_coeffs=self.dist_coeffs,
                mask_path=self.mask_path,
                shapes_list=yolo_list,
                z_prism_offset=self.z_offset
            )

            if min_xyz is not None:
                print(f"\nz最小三维坐标 x={min_xyz[0]:.2f}, y={min_xyz[1]:.2f}, z={min_xyz[2]:.2f} mm")
            else:
                print("\n>>> 当前未检测到有效包裹目标或未拟合出有效抓取平面")

            self.latest_result = (min_xyz, plane_coeff, prism_points, theta_x, theta_y)
            print(f"单帧全流程耗时: {time.time()-t0:.2f}s")

            if not self.loop:
                raise SystemExit
        finally:
            self.processing = False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="顺丰分拣目标检测与平面抓取点计算 (支持实时ROS2话题与离线文件)")
    parser.add_argument("--mode", choices=["topic", "file"], default="topic", help="运行模式: topic (实时ROS2话题) 或 file (离线文件)")
    parser.add_argument("--loop", action="store_true", help="topic 模式下是否持续循环处理 (默认处理单帧后退出)")
    parser.add_argument("--dir", type=str, default="1788945487498660", help="file 模式下的测试样本目录")
    parser.add_argument("--png", type=str, default=None, help="file 模式下的彩色图路径")
    parser.add_argument("--depth", type=str, default=None, help="file 模式下的深度图路径")
    parser.add_argument("--ply", type=str, default=None, help="file 模式下的点云PLY路径")
    parser.add_argument("--mask", type=str, default="mask.png", help="掩码图片路径")
    parser.add_argument("--z_offset", type=float, default=120.0, help="棱柱拟合Z方向偏移半厚度 (mm)")

    args = parser.parse_args()

    # 相机标定参数 (Femto Bolt 4K 标定参数)
    fx = 2249.6287
    fy = 2248.4431
    cx = 1877.3463
    cy = 1073.7207
    dist_coeffs = np.array([
        0.080384,   # k1
        -0.107753,  # k2
        -0.000413,  # p1
        0.000256,   # p2
        0.044622,   # k3
        0.000000,   # k4
        0.000000,   # k5
        0.000000    # k6
    ], dtype=np.float64)

    if args.mode == "topic":
        import rclpy
        rclpy.init()
        runner = SFSortingROS2Node(
            mask_path=args.mask,
            fx=fx, fy=fy, cx=cx, cy=cy,
            dist_coeffs=dist_coeffs,
            loop=args.loop,
            z_offset=args.z_offset
        )
        try:
            rclpy.spin(runner.node)
        except SystemExit:
            pass
        finally:
            runner.node.destroy_node()
            rclpy.shutdown()

    elif args.mode == "file":
        sample_dir = args.dir
        png_file = args.png or os.path.join(sample_dir, f"color_{os.path.basename(sample_dir)}.png")
        depth_file = args.depth or os.path.join(sample_dir, f"depth_{os.path.basename(sample_dir)}.png")
        ply_file = args.ply or os.path.join(sample_dir, f"RGBDPoints_{os.path.basename(sample_dir)}.ply")

        print(f"=== 离线文件模式 ===")
        print(f"Color: {png_file}")
        print(f"Depth: {depth_file}")
        print(f"PLY:   {ply_file}")
        print(f"Mask:  {args.mask}")

        from predict_yolo import yoloDetect
        yolo_list = yoloDetect(png_file)
        print(f"YOLO 检测完成，共识别 {len(yolo_list)} 个目标")

        clear_ply_cache()
        min_xyz, plane_coeff, prism_points, theta_x, theta_y = process_one_sample(
            png_path=png_file,
            depth_path=depth_file,
            ply_path=ply_file,
            fx=fx, fy=fy, cx=cx, cy=cy,
            dist_coeffs=dist_coeffs,
            mask_path=args.mask,
            shapes_list=yolo_list,
            z_prism_offset=args.z_offset
        )

        if min_xyz is not None:
            print(f"\nz最小三维坐标 x={min_xyz[0]:.2f}, y={min_xyz[1]:.2f}, z={min_xyz[2]:.2f} mm")
        else:
            print("无结果")
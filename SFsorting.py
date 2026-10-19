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

    npy_path = f"{ply_path}.voxel{voxel_mm:.4f}.npy"
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


def get_quadrilateral_from_polygon(poly_points):
    """
    对多边形进行自适应逼近，得到4个顶点的近似内接四边形
    顶点均落在原多边形边界上，适用于类矩形物体
    :param poly_points: list[[x,y],...] 多边形像素坐标
    :return: (4,2) numpy数组 四个顶点像素坐标（顺时针排序），失败返回None
    """
    if len(poly_points) < 4:
        return None

    pts = np.array(poly_points, dtype=np.int32).reshape((-1, 1, 2))
    perimeter = cv2.arcLength(pts, True)

    # 自适应epsilon，迭代逼近到4个顶点
    epsilon = 0.01 * perimeter
    approx = None
    for _ in range(20):
        approx = cv2.approxPolyDP(pts, epsilon, True)
        if len(approx) == 4:
            break
        elif len(approx) > 4:
            epsilon *= 1.2
        else:
            epsilon *= 0.8

    if len(approx) != 4:
        print(f"警告：无法逼近为4点四边形，当前顶点数{len(approx)}，返回原始前4个顶点")
        if len(poly_points) >= 4:
            quad = np.array(poly_points[:4], dtype=np.float32)
        else:
            return None
    else:
        quad = approx.reshape(4, 2).astype(np.float32)

    # 按质心极角顺时针排序四个顶点
    cx, cy = np.mean(quad[:, 0]), np.mean(quad[:, 1])
    angles = np.arctan2(quad[:, 1] - cy, quad[:, 0] - cx)
    sort_idx = np.argsort(angles)
    quad = quad[sort_idx]
    return quad


def undistort_pixel_point(u, v, camera_matrix, dist_coeffs):
    """单个畸变像素去畸变，返回归一化平面坐标 (x_norm, y_norm)"""
    pts = np.array([[[u, v]]], dtype=np.float64)
    undist_pts = cv2.undistortPoints(pts, camera_matrix, dist_coeffs)
    xn, yn = undist_pts[0, 0]
    return xn, yn


def pixel2world_with_distortion(u, v, depth_mm, camera_matrix, dist_coeffs):
    """畸变像素(u,v)+深度(mm) → 相机坐标系3D点(mm)"""
    if depth_mm <= 0:
        return None
    xn, yn = undistort_pixel_point(u, v, camera_matrix, dist_coeffs)
    Z = depth_mm
    X = xn * Z
    Y = yn * Z
    return (X, Y, Z)


def pixel2world_by_plane(u, v, camera_matrix, dist_coeffs, plane_coeff):
    """
    根据像素坐标 + 拟合平面方程，计算三维坐标（比单像素深度更鲁棒）
    :param u: 像素横坐标
    :param v: 像素纵坐标
    :param camera_matrix: 相机内参矩阵
    :param dist_coeffs: 畸变系数
    :param plane_coeff: 平面方程(a,b,c,d)，满足 ax + by + cz + d = 0
    :return: (X,Y,Z) 三维坐标，失败返回None
    """
    a, b, c, d = plane_coeff
    xn, yn = undistort_pixel_point(u, v, camera_matrix, dist_coeffs)

    # 代入平面方程求解深度Z：a*xn*Z + b*yn*Z + c*Z + d = 0
    denom = a * xn + b * yn + c
    if abs(denom) < 1e-9:
        return None
    Z = -d / denom
    if Z <= 0:
        return None

    X = xn * Z
    Y = yn * Z
    return (float(X), float(Y), float(Z))


def fit_plane_svd(points_xyz: np.ndarray, dist_thresh_mm: float = 0.005, ransac_iter: int = 800):
    """
    基于Open3D RANSAC拟合平面，返回(a,b,c,d)，法向量单位化
    points_xyz: N×3 numpy array (单位同点云)
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
    return float(theta_x_deg), float(theta_y_deg)


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
        ply_source,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        z_offset_range: float = 0.15,
        voxel_mm: float = 0.005,
        max_roi_pts: int = 3000
):
    """
    构造Z向多棱柱，筛选PLY点云落在棱柱内部的点
    :param ply_source: 文件路径字符串 或 已提取好的 N×3 点云数组 (单位: 米)
    :return prism_inlier_points(N×3), outline_px(N,2), z_low, z_high；失败返回None,None,None,None
    """
    outline_px = get_mask_outline_polygon(mask_1_2_mini)
    if outline_px is None or len(outline_px) < 3:
        print("mask_1_2_mini提取不到有效轮廓")
        return None, None, None, None

    if isinstance(ply_source, np.ndarray):
        pts_all = ply_source
        if voxel_mm > 0.0 and len(pts_all) > 0:
            pts_all = numpy_voxel_downsample(pts_all, voxel_mm)
    else:
        pts_all = load_downsampled_ply(ply_source, voxel_mm)

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
        ply_source,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        z_offset_range: float = 0.15,
        save_mini_mask_path: str = None,
        voxel_mm: float = 0.005,
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

    if save_mini_mask_path is not None:
        cv2.imwrite(save_mini_mask_path, mask_1_2_mini)

    prism_points, outline_px, z_low, z_high = get_prism_inlier_from_ply(
        mask_1_2_mini=mask_1_2_mini,
        ply_source=ply_source,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        z_offset_range=z_offset_range,
        voxel_mm=voxel_mm,
        max_roi_pts=max_roi_pts
    )

    if prism_points is None or prism_points.shape[0] < 4:
        return mask_1_2_mini, None, prism_points, outline_px, z_low, z_high

    plane_coeff = fit_plane_svd(prism_points)
    return mask_1_2_mini, plane_coeff, prism_points, outline_px, z_low, z_high


def process_one_sample(
        png_path,
        depth_path,
        ply_path,
        fx, fy, cx, cy,
        dist_coeffs,
        mask_path='mask1.png',
        shapes_list=None,
        z_prism_offset=0.15,
        voxel_mm=0.005,
        max_roi_pts=3000
):
    """
    单样本核心处理：
    :return (best_label, min_xyz, quad_3d, plane_info)
    """
    # 1. 读取彩色图
    if isinstance(png_path, np.ndarray):
        img = png_path
    else:
        img = cv2.imread(png_path)
    if img is None:
        raise FileNotFoundError(f"彩色图读取失败 {png_path}")
    H, W = img.shape[:2]

    # 2. 读取本地mask (优先 mask1.png)
    if isinstance(mask_path, np.ndarray):
        mask = mask_path
    else:
        if not os.path.exists(mask_path):
            script_dir = os.path.dirname(os.path.abspath(__file__))
            cand = os.path.join(script_dir, mask_path)
            if os.path.exists(cand):
                mask_path = cand
        mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise FileNotFoundError(f"mask图片读取失败 {mask_path}")
    mask = (mask > 0).astype(np.uint8) * 255  # 确保二值化

    # 3. 相机内参矩阵
    camera_matrix = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ], dtype=np.float64)

    # 4. 处理标注列表：carton/soft_bag/foam 对应 mask_1_2
    mask_1_2 = np.zeros((H, W), dtype=np.uint8)
    poly_12_list = []
    LABEL_12_SET = {'carton', 'soft_bag', 'foam'}
    for shape in (shapes_list or []):
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
        return None, None, None, None

    # 6. 读取深度图
    if isinstance(depth_path, np.ndarray):
        depth_np = depth_path.astype(np.float32)
    else:
        depth_raw = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        if depth_raw is None:
            raise FileNotFoundError(f"深度图读取失败 {depth_path}")
        depth_np = depth_raw.astype(np.float32)

    # 7. 遍历1/2类多边形，找Z最小的有效点，同时保留标签和多边形顶点
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
        # 如果中心单像素没有有效深度，取其周围 15x15 局部窗口内有效深度的中值
        if d_val <= 0:
            patch = depth_np[max(0, v - 7):min(H, v + 8), max(0, u - 7):min(W, u + 8)]
            valid_patch = patch[patch > 100]
            if len(valid_patch) > 0:
                d_val = float(np.median(valid_patch))
            else:
                pts_in_poly = depth_np[(poly_mask > 0) & (depth_np > 100)]
                if len(pts_in_poly) > 0:
                    d_val = float(np.median(pts_in_poly))

        xyz = pixel2world_with_distortion(u, v, d_val, camera_matrix, dist_coeffs)

        if xyz is not None:
            valid_records.append({
                "xyz": xyz,
                "u": u,
                "v": v,
                "poly_mask": poly_mask,
                "label": item["label"],
                "poly_points": item["points"]
            })

    if len(valid_records) == 0:
        print("相交的多边形没有有效深度")
        return None, None, None, None

    # 8. 取Z最小的多边形做平面拟合
    valid_records.sort(key=lambda r: r["xyz"][2])
    best_rec = valid_records[0]
    min_xyz = best_rec["xyz"]
    best_label = best_rec["label"]
    best_poly_mask = best_rec["poly_mask"]
    best_u, best_v = best_rec["u"], best_rec["v"]

    mask_1_2_mini, plane_coeff, prism_points, outline_px, z_low, z_high = get_shrunk_mask_prism_fit_plane(
        poly_mask=best_poly_mask,
        centroid_px=(best_u, best_v),
        ply_source=ply_path,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        z_offset_range=z_prism_offset,
        voxel_mm=voxel_mm,
        max_roi_pts=max_roi_pts
    )

    # 9. 计算平面夹角 + 四边形顶点三维坐标
    plane_info = None
    quad_3d = None
    if plane_coeff is not None:
        theta_x_deg, theta_y_deg = calc_plane_xy_angles(plane_coeff)
        plane_info = {
            "plane_equation": plane_coeff,  # (a, b, c, d)
            "theta_x_deg": theta_x_deg,
            "theta_y_deg": theta_y_deg
        }

        # 计算内接四边形三维坐标
        quad_px = get_quadrilateral_from_polygon(best_rec["poly_points"])
        if quad_px is not None:
            quad_3d_pts = []
            for (u, v) in quad_px:
                xyz = pixel2world_by_plane(u, v, camera_matrix, dist_coeffs, plane_coeff)
                if xyz is None:
                    d_val = depth_np[int(np.clip(round(v), 0, H - 1)), int(np.clip(round(u), 0, W - 1))]
                    xyz = pixel2world_with_distortion(u, v, d_val, camera_matrix, dist_coeffs)
                quad_3d_pts.append(xyz)
            quad_3d_arr = np.array(quad_3d_pts, dtype=np.float32)
            # 如果计算出的深度为米单位(<50)，转为毫米(mm)
            if np.mean(quad_3d_arr[:, 2]) < 50.0:
                quad_3d = quad_3d_arr * 1000.0
            else:
                quad_3d = quad_3d_arr

    return best_label, min_xyz, quad_3d, plane_info


def extract_pointcloud2_xyz_m(cloud_msg, stride=24):
    """
    高效解析 ROS2 sensor_msgs/PointCloud2 消息中的 (x, y, z)，单位：米 (m)
    stride: 步长采样 (默认24)，将463万点快速缩减为~19万点，避免耗时的全量内存拷贝与无效反投影计算
    """
    num_points = cloud_msg.width * cloud_msg.height
    if num_points == 0 or len(cloud_msg.data) == 0:
        return np.empty((0, 3), dtype=np.float32)
    step = cloud_msg.point_step
    raw_bytes = np.frombuffer(cloud_msg.data, dtype=np.uint8)
    raw_reshaped = raw_bytes[:num_points * step].reshape((num_points, step))
    sampled_rows = raw_reshaped[::stride, :12] if stride > 1 else raw_reshaped[:, :12]
    pts_m = np.ascontiguousarray(sampled_rows).view(np.float32).reshape((-1, 3))
    valid_mask = np.isfinite(pts_m).all(axis=1) & (pts_m[:, 2] > 0.01)
    return pts_m[valid_mask]


class SFSortingROS2Node:
    """
    ROS 2 实时订阅节点：
    订阅 /vision/top_camera/rgb, /vision/top_camera/depth, /vision/top_camera/points
    通过 ApproximateTimeSynchronizer 触发 YOLO 目标检测与四边形/平面拟合分拣
    """
    def __init__(self, mask_path="mask1.png", fx=2249.6287, fy=2248.4431, cx=1877.3463, cy=1073.7207, dist_coeffs=None, loop=False, z_offset=0.15):
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
        # 内存预加载 mask
        self.mask_arr = None
        if os.path.exists(self.mask_path):
            self.mask_arr = cv2.imread(self.mask_path, cv2.IMREAD_GRAYSCALE)
            if self.mask_arr is not None:
                self.mask_arr = (self.mask_arr > 0).astype(np.uint8) * 255

        # 内存预热 YOLO 模型
        try:
            from predict_yolo import get_model
            model = get_model()
            dummy_img = np.zeros((640, 640, 3), dtype=np.uint8)
            _ = model.predict(source=dummy_img, device="cpu", imgsz=1280, verbose=False)
            print("[*] YOLO 模型预热完成!", flush=True)
        except Exception as e:
            print(f"[WARN] YOLO 预热跳过: {e}", flush=True)

        self.sub_rgb = message_filters.Subscriber(self.node, Image, '/vision/top_camera/rgb', qos_profile=10)
        self.sub_depth = message_filters.Subscriber(self.node, Image, '/vision/top_camera/depth', qos_profile=10)
        self.sub_points = message_filters.Subscriber(self.node, PointCloud2, '/vision/top_camera/points', qos_profile=10)

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

            # 3. 转换点云 (米 m, 步长采样)
            pts_m = extract_pointcloud2_xyz_m(points_msg, stride=36)
            print(f"  有效点云点数 (>0.01m): {len(pts_m)}")

            # 4. YOLO 目标检测
            from predict_yolo import yoloDetect
            yolo_list = yoloDetect(bgr_img)
            print(f"  YOLO 检测完成，共识别 {len(yolo_list)} 个包裹目标")

            # 5. 执行平面拟合与抓取点定位
            clear_ply_cache()
            best_label, min_xyz, quad_3d, plane_info = process_one_sample(
                png_path=bgr_img,
                depth_path=depth_np,
                ply_path=pts_m,
                fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy,
                dist_coeffs=self.dist_coeffs,
                mask_path=self.mask_arr if self.mask_arr is not None else self.mask_path,
                shapes_list=yolo_list,
                z_prism_offset=self.z_offset,
                voxel_mm=0.0
            )

            if min_xyz is not None:
                print(f"\n[+] 最优目标类别: {best_label}")
                print(f"    z最小三维坐标: x={min_xyz[0]:.2f}, y={min_xyz[1]:.2f}, z={min_xyz[2]:.2f} mm")
                if quad_3d is not None:
                    print(f"    四边形3D顶点 (mm):\n{quad_3d}")
                if plane_info is not None:
                    p = plane_info["plane_equation"]
                    print(f"    平面方程: {p[0]:.4f}*x + {p[1]:.4f}*y + {p[2]:.4f}*z + {p[3]:.4f} = 0")
                    print(f"    倾斜角: theta_x={plane_info['theta_x_deg']:.4f}°, theta_y={plane_info['theta_y_deg']:.4f}°")
            else:
                print("\n>>> 当前未检测到有效包裹目标或未拟合出有效抓取平面")

            self.latest_result = (best_label, min_xyz, quad_3d, plane_info)
            print(f"单帧全流程耗时: {time.time()-t0:.2f}s")

            if not self.loop:
                raise SystemExit
        finally:
            self.processing = False


class SFSortingServerNode:
    """
    ROS 2 服务端节点：
    提供服务 /sf_sorting/get_result (std_srvs/srv/Trigger)
    提供话题 /sf_sorting/result (std_msgs/msg/String)
    在同一局域网下，任何工控机设置相同 ROS_DOMAIN_ID 均可直接调用该服务获取包裹坐标与拟合结果
    """
    def __init__(self, mask_path="mask1.png", fx=2249.6287, fy=2248.4431, cx=1877.3463, cy=1073.7207, dist_coeffs=None, z_offset=0.15):
        import rclpy
        from sensor_msgs.msg import Image, PointCloud2
        from std_srvs.srv import Trigger
        from std_msgs.msg import String
        import message_filters
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rclpy.callback_groups import ReentrantCallbackGroup

        self.mask_path = mask_path
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.array([
            0.080384, -0.107753, -0.000413, 0.000256, 0.044622, 0.0, 0.0, 0.0
        ], dtype=np.float64)
        self.z_offset = z_offset
        self.latest_frame = None
        self.latest_frame_time = 0.0

        self.cb_group = ReentrantCallbackGroup()
        self.node = rclpy.create_node('sf_sorting_server_node')

        # 内存预加载 mask
        self.mask_arr = None
        if os.path.exists(self.mask_path):
            self.mask_arr = cv2.imread(self.mask_path, cv2.IMREAD_GRAYSCALE)
            if self.mask_arr is not None:
                self.mask_arr = (self.mask_arr > 0).astype(np.uint8) * 255

        # 内存预热 YOLO 模型
        try:
            from predict_yolo import get_model
            model = get_model()
            dummy_img = np.zeros((640, 640, 3), dtype=np.uint8)
            _ = model.predict(source=dummy_img, device="cpu", imgsz=1280, verbose=False)
            print("[*] YOLO 模型预热完成!", flush=True)
        except Exception as e:
            print(f"[WARN] YOLO 预热跳过: {e}", flush=True)

        # 订阅相机实时话题 (采用标准 depth=10 避免 ROS2 message_filters QoS 丢包)
        self.sub_rgb = message_filters.Subscriber(self.node, Image, '/vision/top_camera/rgb', qos_profile=10, callback_group=self.cb_group)
        self.sub_depth = message_filters.Subscriber(self.node, Image, '/vision/top_camera/depth', qos_profile=10, callback_group=self.cb_group)
        self.sub_points = message_filters.Subscriber(self.node, PointCloud2, '/vision/top_camera/points', qos_profile=10, callback_group=self.cb_group)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth, self.sub_points],
            queue_size=10,
            slop=0.08
        )
        self.sync.registerCallback(self.frame_callback)

        # 创建服务与发布者
        self.srv = self.node.create_service(Trigger, '/sf_sorting/get_result', self.handle_get_result, callback_group=self.cb_group)
        self.pub_result = self.node.create_publisher(String, '/sf_sorting/result', 10)

        domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
        print("="*60, flush=True)
        print(f"[*] 顺丰分拣 ROS 2 Service Server 启动成功!", flush=True)
        print(f"[*] ROS_DOMAIN_ID: {domain_id}", flush=True)
        print(f"[*] 蒙版路径: {self.mask_path}", flush=True)
        print(f"[*] 服务接口: /sf_sorting/get_result (std_srvs/srv/Trigger)", flush=True)
        print(f"[*] 广播话题: /sf_sorting/result (std_msgs/msg/String JSON)", flush=True)
        print(f"[*] 等待客户端请求...", flush=True)
        print("="*60, flush=True)

    def frame_callback(self, rgb_msg, depth_msg, points_msg):
        self.latest_frame = (rgb_msg, depth_msg, points_msg)
        self.latest_frame_time = time.time()

    def handle_get_result(self, request, response):
        t0 = time.time()
        print(f"\n[Service 请求] 收到 /sf_sorting/get_result 调用请求...", flush=True)

        # 获取最新对齐帧 (由于后台持续以 3.3Hz 实时刷新，内存中已有最新帧时直接处理，零等待)
        start_wait = time.time()
        while self.latest_frame is None:
            time.sleep(0.01)
            if time.time() - start_wait > 2.0:
                break
        t_wait = time.time() - start_wait

        if self.latest_frame is None:
            err_msg = "获取相机画面超时，未收到 /vision/top_camera 话题帧"
            print(f"[ERROR] {err_msg}", flush=True)
            res_dict = {
                "success": False,
                "timestamp": time.time(),
                "best_label": None,
                "x": None,
                "y": None,
                "z": None,
                "quad_3d": None,
                "theta_x_deg": None,
                "theta_y_deg": None
            }
            response.success = False
            response.message = json.dumps(res_dict, ensure_ascii=False)
            return response

        rgb_msg, depth_msg, points_msg = self.latest_frame

        try:
            t1 = time.time()
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

            # 3. 转换点云 (米 m, 步长采样直接提取)
            pts_m = extract_pointcloud2_xyz_m(points_msg, stride=36)
            t_parse = time.time() - t1

            # 4. YOLO 目标检测 (内存直接推理)
            t2 = time.time()
            from predict_yolo import yoloDetect
            yolo_list = yoloDetect(bgr_img)
            t_yolo = time.time() - t2
            print(f"  YOLO 检测完成，共识别 {len(yolo_list)} 个包裹目标 (耗时 {t_yolo*1000:.1f}ms)", flush=True)

            # 5. 执行平面拟合与抓取点定位
            t3 = time.time()
            clear_ply_cache()
            best_label, min_xyz, quad_3d, plane_info = process_one_sample(
                png_path=bgr_img,
                depth_path=depth_np,
                ply_path=pts_m,
                fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy,
                dist_coeffs=self.dist_coeffs,
                mask_path=self.mask_arr if self.mask_arr is not None else self.mask_path,
                shapes_list=yolo_list,
                z_prism_offset=self.z_offset,
                voxel_mm=0.0
            )
            t_fit = time.time() - t3

            total_cost = time.time() - t0
            print(f"  [响应耗时明细] 等待帧: {t_wait*1000:.1f}ms | 转换: {t_parse*1000:.1f}ms | YOLO: {t_yolo*1000:.1f}ms | 3D拟合: {t_fit*1000:.1f}ms | 总响应: {total_cost:.3f}s", flush=True)
            if min_xyz is not None:
                print(f"  --> 计算成功! 目标类别: {best_label}", flush=True)
                print(f"      z最小坐标: x={min_xyz[0]:.2f}, y={min_xyz[1]:.2f}, z={min_xyz[2]:.2f} mm", flush=True)
                if quad_3d is not None:
                    print(f"      四边形3D坐标 (mm):\n{quad_3d}", flush=True)
                if plane_info is not None:
                    pe = plane_info["plane_equation"]
                    print(f"      平面方程: {pe[0]:.4f}*x + {pe[1]:.4f}*y + {pe[2]:.4f}*z + {pe[3]:.4f} = 0", flush=True)
                    print(f"      倾斜角: theta_x={plane_info['theta_x_deg']:.4f}°, theta_y={plane_info['theta_y_deg']:.4f}°", flush=True)

                res_dict = {
                    "success": True,
                    "timestamp": time.time(),
                    "best_label": best_label,
                    "x": round(float(min_xyz[0]), 2),
                    "y": round(float(min_xyz[1]), 2),
                    "z": round(float(min_xyz[2]), 2),
                    "quad_3d": [[round(float(c), 2) for c in pt] for pt in quad_3d] if quad_3d is not None else None,
                    "theta_x_deg": round(float(plane_info["theta_x_deg"]), 4) if plane_info else None,
                    "theta_y_deg": round(float(plane_info["theta_y_deg"]), 4) if plane_info else None
                }
                response.success = True
            else:
                print("  --> 未检测到有效包裹目标或未拟合出有效抓取平面", flush=True)
                res_dict = {
                    "success": False,
                    "timestamp": time.time(),
                    "best_label": None,
                    "x": None,
                    "y": None,
                    "z": None,
                    "quad_3d": None,
                    "theta_x_deg": None,
                    "theta_y_deg": None
                }
                response.success = False

            json_str = json.dumps(res_dict, ensure_ascii=False)
            response.message = json_str

            # 发布到话题
            from std_msgs.msg import String
            msg_str = String()
            msg_str.data = json_str
            self.pub_result.publish(msg_str)

            print(f"[Service 响应完毕] 耗时 {total_cost:.2f}s, success={response.success}\n", flush=True)
            return response
        except Exception as e:
            err_msg = f"处理异常: {str(e)}"
            print(f"[ERROR] {err_msg}", flush=True)
            response.success = False
            response.message = json.dumps({"success": False, "error": err_msg}, ensure_ascii=False)
            return response


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="顺丰分拣目标检测与平面抓取点计算 (支持实时ROS2话题与离线文件)")
    parser.add_argument("--mode", choices=["topic", "service", "file"], default="topic", help="运行模式: service (ROS2服务模式), topic (实时ROS2话题模式) 或 file (离线文件)")
    parser.add_argument("--loop", action="store_true", help="topic 模式下是否持续循环处理 (默认处理单帧后退出)")
    parser.add_argument("--png", type=str, default=None, help="file 模式下的彩色图路径")
    parser.add_argument("--depth", type=str, default=None, help="file 模式下的深度图路径")
    parser.add_argument("--ply", type=str, default=None, help="file 模式下的点云PLY路径")
    parser.add_argument("--mask", type=str, default="mask1.png", help="掩码图片路径 (默认: mask1.png)")
    parser.add_argument("--z_offset", type=float, default=0.15, help="棱柱拟合Z方向偏移半厚度 (米 m, 默认: 0.15)")

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

    if args.mode == "service":
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        rclpy.init()
        server_node = SFSortingServerNode(
            mask_path=args.mask,
            fx=fx, fy=fy, cx=cx, cy=cy,
            dist_coeffs=dist_coeffs,
            z_offset=args.z_offset
        )
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(server_node.node)
        try:
            executor.spin()
        except (KeyboardInterrupt, SystemExit):
            print("\n正在停止分拣服务...")
        finally:
            server_node.node.destroy_node()
            rclpy.shutdown()

    elif args.mode == "topic":
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
        # 默认使用 1015 样本 000005
        default_dir = "/home/caseim/yolo26_packet/captured_frame"
        png_file = args.png or os.path.join(default_dir, "rgb", "000005.png")
        depth_file = args.depth or os.path.join(default_dir, "depth", "000005.png")
        ply_file = args.ply or os.path.join(default_dir, "point", "000005.ply")

        # 如果在本地测试，检查是否存在备用路径
        if not os.path.exists(png_file) and os.path.exists("1015data/captured_8001/rgb/000005.png"):
            png_file = "1015data/captured_8001/rgb/000005.png"
            depth_file = "1015data/captured_8001/depth/000005.png"
            ply_file = "1015data/captured_8001/point/000005.ply"

        print(f"=== 离线文件模式 ===")
        print(f"Color: {png_file}")
        print(f"Depth: {depth_file}")
        print(f"PLY:   {ply_file}")
        print(f"Mask:  {args.mask}")

        t_f0 = time.time()
        bgr_img = cv2.imread(png_file)
        depth_np = cv2.imread(depth_file, cv2.IMREAD_UNCHANGED).astype(np.float32)

        from predict_yolo import yoloDetect
        t_y0 = time.time()
        yolo_list = yoloDetect(bgr_img)
        t_yolo = time.time() - t_y0
        print(f"YOLO 检测完成，共识别 {len(yolo_list)} 个目标 (耗时 {t_yolo*1000:.1f}ms)")

        t_c0 = time.time()
        clear_ply_cache()
        best_label, min_xyz, quad_3d, plane_info = process_one_sample(
            png_path=bgr_img,
            depth_path=depth_np,
            ply_path=ply_file,
            fx=fx, fy=fy, cx=cx, cy=cy,
            dist_coeffs=dist_coeffs,
            mask_path=args.mask,
            shapes_list=yolo_list,
            z_prism_offset=args.z_offset,
            voxel_mm=0.005
        )
        t_fit = time.time() - t_c0
        print(f"=== 处理耗时明细 ===")
        print(f"YOLO 检测耗时: {t_yolo*1000:.1f} ms")
        print(f"3D平面拟合耗时: {t_fit*1000:.1f} ms")
        print(f"全流程总耗时:   {(time.time()-t_f0)*1000:.1f} ms")

        print("\n===== 拟合结果 =====")
        print(f"最优目标类别: {best_label}")
        if min_xyz is not None:
            print(f"z最小三维坐标: x={min_xyz[0]:.2f}, y={min_xyz[1]:.2f}, z={min_xyz[2]:.2f} mm")
        if quad_3d is not None:
            print(f"四边形3D顶点 (mm):\n{quad_3d}")
        if plane_info is not None:
            pe = plane_info["plane_equation"]
            print(f"平面方程: {pe[0]:.4f}*x + {pe[1]:.4f}*y + {pe[2]:.4f}*z + {pe[3]:.4f} = 0")
            print(f"倾斜角: theta_x={plane_info['theta_x_deg']:.4f}°, theta_y={plane_info['theta_y_deg']:.4f}°")
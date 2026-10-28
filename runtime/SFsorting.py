import json
import os
import time
from deployment_config import load_calibration

os.environ.setdefault("OMP_NUM_THREADS", "2")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "2")

import cv2
import numpy as np
import open3d as o3d

try:
    from numba import njit
    _HAS_NUMBA = True
except ImportError:
    _HAS_NUMBA = False


def barcode_wechat_detect(img):
    try:
        detector = cv2.wechat_qrcode.WeChatQRCode()
    except Exception as e:
        print(f"初始化 WeChatQRCode 失败: {e}")
        return []
    if img is None:
        print("图片读取失败")
        return []
    scale = 1.2
    img_resize = cv2.resize(img, None, fx=scale, fy=scale)
    res, points = detector.detectAndDecode(img_resize)
    output = []
    for text, pts in zip(res, points):
        if len(text) > 0:
            output.append({"data": text})
            print(f"识别结果：{text}")
    return output


@njit(cache=True, nogil=True)
def _project_xyz_kernel(points, camera, distortion):
    result = np.empty((len(points), 1, 2), dtype=np.float64)
    k1, k2, p1, p2, k3, k4, k5, k6 = distortion
    for i in range(len(points)):
        z = points[i, 2]
        invz = 1.0 / z if z != 0 else 1.0
        x, y = points[i, 0] * invz, points[i, 1] * invz
        r2 = x*x + y*y
        r4, r6 = r2*r2, r2*r2*r2
        radial = (1 + k1*r2 + k2*r4 + k3*r6) / (1 + k4*r2 + k5*r4 + k6*r6)
        xd = x*radial + 2*p1*x*y + p2*(r2 + 2*x*x)
        yd = y*radial + p1*(r2 + 2*y*y) + 2*p2*x*y
        result[i, 0, 0] = xd*camera[0, 0] + camera[0, 2]
        result[i, 0, 1] = yd*camera[1, 1] + camera[1, 2]
    return result, None

def project_xyz_fast(points, camera, distortion):
    result, _ = _project_xyz_kernel(points, camera, distortion)
    return result.astype(points.dtype), None

@njit(cache=True, nogil=True)
def valid_points_mm(points):
    out = np.empty((len(points), 3), dtype=np.float32)
    count = 0
    for i in range(len(points)):
        x, y, z = points[i, 0], points[i, 1], points[i, 2]
        if np.isfinite(x) and np.isfinite(y) and np.isfinite(z) and z > 0:
            out[count, 0] = x * np.float32(1000)
            out[count, 1] = y * np.float32(1000)
            out[count, 2] = z * np.float32(1000)
            count += 1
    return out[:count]

@njit(cache=True, nogil=True)
def select_projected(points, pixels, mask, z_low, z_high):
    out = np.empty_like(points)
    count = 0
    h, w = mask.shape
    for i in range(len(points)):
        z = points[i, 2]
        if z <= 1e-3 or z < z_low or z > z_high:
            continue
        u = min(w - 1, max(0, int(np.rint(pixels[i, 0, 0]))))
        v = min(h - 1, max(0, int(np.rint(pixels[i, 0, 1]))))
        if mask[v, u] > 0:
            out[count, 0] = points[i, 0]
            out[count, 1] = points[i, 1]
            out[count, 2] = z
            count += 1
    return out[:count]

@njit(cache=True, nogil=True)
def voxel_fast(points, size):
    # Same floor-grid and float64 sums as numpy_voxel_downsample; retain all points.
    n = len(points)
    low = np.full(3, np.int64(9223372036854775807), dtype=np.int64)
    high = np.full(3, np.int64(-9223372036854775807), dtype=np.int64)
    for i in range(n):
        for j in range(3):
            v = np.int64(np.floor(np.float32(points[i, j] / size)))
            low[j] = min(low[j], v)
            high[j] = max(high[j], v)
    dims = high - low + 1
    lookup = {np.int64(-1): np.int64(-1)}
    lookup.clear()
    keys = np.empty(n, dtype=np.int64)
    sums = np.zeros((n, 3), dtype=np.float64)
    counts = np.zeros(n, dtype=np.int64)
    used = 0
    for i in range(n):
        x = np.int64(np.floor(np.float32(points[i, 0] / size))) - low[0]
        y = np.int64(np.floor(np.float32(points[i, 1] / size))) - low[1]
        z = np.int64(np.floor(np.float32(points[i, 2] / size))) - low[2]
        key = (x*dims[1] + y)*dims[2] + z
        if key in lookup:
            idx = lookup[key]
        else:
            idx = used
            lookup[key] = idx
            keys[idx] = key
            used += 1
        for j in range(3):
            sums[idx, j] += points[i, j]
        counts[idx] += 1
    order = np.argsort(keys[:used])
    result = np.empty((used, 3), dtype=np.float64)
    for i in range(used):
        idx = order[i]
        for j in range(3):
            result[i, j] = sums[idx, j] / counts[idx]
    return result


class PreparedCloud:
    def __init__(self, points, camera, distortion):
        started = time.perf_counter()
        self.raw = valid_points_mm(points)
        self.down = voxel_fast(self.raw, 5)
        def uv(pts):
            norm = np.column_stack([pts[:, 0] / pts[:, 2], pts[:, 1] / pts[:, 2], np.ones_like(pts[:, 2])])
            return project_xyz_fast(norm, camera, distortion)[0]
        self.raw_uv = uv(self.raw)
        self.down_uv = uv(self.down)
        self.prepare_ms = round((time.perf_counter() - started) * 1000, 1)

_MASK_CACHE = {}
_LAST_TIMING = {}
_PLY_CACHE = {}


def clear_ply_cache():
    global _PLY_CACHE
    _PLY_CACHE.clear()


def numpy_voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    voxel_idx = np.floor(points / voxel_size).astype(np.int64)
    vmin = voxel_idx.min(axis=0)
    vk = voxel_idx - vmin
    dims = vk.max(axis=0) + 1
    lin = (vk[:, 0] * dims[1] + vk[:, 1]) * dims[2] + vk[:, 2]
    _, inverse_idx = np.unique(lin, return_inverse=True)
    sum_x = np.bincount(inverse_idx, weights=points[:, 0])
    sum_y = np.bincount(inverse_idx, weights=points[:, 1])
    sum_z = np.bincount(inverse_idx, weights=points[:, 2])
    count = np.bincount(inverse_idx)
    return np.column_stack([sum_x, sum_y, sum_z]) / count[:, None]


def load_downsampled_ply(ply_path: str, voxel_mm: float):
    if isinstance(ply_path, PreparedCloud):
        return ply_path.raw, ply_path.down
    pts_raw = valid_points_mm(ply_path) if isinstance(ply_path, np.ndarray) else np.asarray(o3d.io.read_point_cloud(ply_path).points, dtype=np.float32) * 1000
    pts_all = voxel_fast(pts_raw, voxel_mm)
    return pts_raw, pts_all


def polygon_to_mask(polygon_points, img_shape):
    mask = np.zeros(img_shape[:2], dtype=np.uint8)
    pts = np.array(polygon_points, dtype=np.int32).reshape((-1, 1, 2))
    cv2.fillPoly(mask, [pts], 255)
    return mask


def get_polygon_centroid_inside(poly_points):
    from shapely.geometry import Polygon
    poly = Polygon(poly_points)
    centroid = poly.centroid
    if not poly.contains(centroid):
        pt = poly.representative_point()
        return (pt.x, pt.y)
    return (centroid.x, centroid.y)


def undistort_pixel_point(u, v, camera_matrix, dist_coeffs):
    pts = np.array([[[u, v]]], dtype=np.float64)
    undist_pts = cv2.undistortPoints(pts, camera_matrix, dist_coeffs)
    xn, yn = undist_pts[0, 0]
    return xn, yn


def pixel2world_with_distortion(u, v, depth_mm, camera_matrix, dist_coeffs):
    if depth_mm <= 0:
        return None
    xn, yn = undistort_pixel_point(u, v, camera_matrix, dist_coeffs)
    Z = depth_mm
    X = xn * Z
    Y = yn * Z
    return (X, Y, Z)


def fit_plane_svd(points_xyz: np.ndarray, dist_thresh_mm: float = 1.0, ransac_iter: int = 800):
    n_points = points_xyz.shape[0]
    if n_points < 4:
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


def get_mask_outline_polygon(mask: np.ndarray):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        return None
    max_cnt = max(contours, key=cv2.contourArea)
    return max_cnt.reshape(-1, 2)


def get_prism_inlier_from_ply(
        mask_1_2_mini: np.ndarray,
        ply_path: str,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        z_offset_range: float = 150.0,
        voxel_mm: float = 3.0,
        max_roi_pts: int = 3000
):
    outline_px = get_mask_outline_polygon(mask_1_2_mini)
    if outline_px is None or len(outline_px) < 3:
        return None, None, None, None, None

    pts_raw, pts_all = load_downsampled_ply(ply_path, voxel_mm)
    if pts_all.shape[0] < 10:
        return pts_raw, None, outline_px, None, None

    if isinstance(ply_path, PreparedCloud):
        pts_candidate = select_projected(pts_all, ply_path.down_uv, mask_1_2_mini, -np.inf, np.inf)
    else:
        X_all, Y_all, Z_all = pts_all[:, 0], pts_all[:, 1], pts_all[:, 2]
        valid_z_mask = Z_all > 1e-3
        if not np.any(valid_z_mask):
            return pts_raw, None, outline_px, None, None

        Xv = X_all[valid_z_mask]
        Yv = Y_all[valid_z_mask]
        Zv = Z_all[valid_z_mask]
        if isinstance(ply_path, PreparedCloud):
            proj_pixels = ply_path.down_uv[valid_z_mask]
        else:
            pts_norm = np.column_stack([Xv / Zv, Yv / Zv, np.ones_like(Zv)])
            proj_pixels, _ = project_xyz_fast(pts_norm, camera_matrix, dist_coeffs)
        proj_uv = proj_pixels[:, 0, :]

        proj_u = np.clip(np.round(proj_uv[:, 0]).astype(np.int32), 0, mask_1_2_mini.shape[1] - 1)
        proj_v = np.clip(np.round(proj_uv[:, 1]).astype(np.int32), 0, mask_1_2_mini.shape[0] - 1)
        inside = mask_1_2_mini[proj_v, proj_u] > 0
        pts_candidate = np.column_stack([Xv, Yv, Zv])[inside]

    if pts_candidate.shape[0] < 4:
        return pts_raw, None, outline_px, None, None

    z_mean = float(np.mean(pts_candidate[:, 2]))
    z_low = z_mean - z_offset_range
    z_high = z_mean + z_offset_range
    z_prism_mask = (pts_candidate[:, 2] >= z_low) & (pts_candidate[:, 2] <= z_high)
    prism_inlier_points = pts_candidate[z_prism_mask]

    if len(prism_inlier_points) > max_roi_pts:
        idx_sel = np.random.choice(len(prism_inlier_points), max_roi_pts, replace=False)
        prism_inlier_points = prism_inlier_points[idx_sel]

    return pts_raw, prism_inlier_points, outline_px, z_low, z_high


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
    H, W = poly_mask.shape[:2]
    u0, v0 = centroid_px
    # cv2.boundingRect 代替 np.where(poly_mask > 0)，快一个数量级
    bx, by, bw, bh = cv2.boundingRect(poly_mask)
    if bw == 0 or bh == 0:
        return None, np.zeros_like(poly_mask), None, None, None, None, None
    xmin, ymin = bx, by
    xmax, ymax = bx + bw - 1, by + bh - 1

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

    pts_raw, prism_points, outline_px, z_low, z_high = get_prism_inlier_from_ply(
        mask_1_2_mini=mask_1_2_mini,
        ply_path=ply_path,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        z_offset_range=z_offset_range,
        voxel_mm=voxel_mm,
        max_roi_pts=max_roi_pts
    )

    if prism_points is None or prism_points.shape[0] < 4:
        return pts_raw, mask_1_2_mini, None, prism_points, outline_px, z_low, z_high

    plane_coeff = fit_plane_svd(prism_points)
    return pts_raw, mask_1_2_mini, plane_coeff, prism_points, outline_px, z_low, z_high


def cam_points_to_world(points_cam, T_wc):
    if points_cam is None:
        return None
    T_wc = np.asarray(T_wc, dtype=np.float64)
    if isinstance(points_cam, (tuple, list)):
        xc, yc, zc = points_cam
        pc_h = np.array([[xc], [yc], [zc], [1.0]])
        pw_h = T_wc @ pc_h
        return (float(pw_h[0, 0]), float(pw_h[1, 0]), float(pw_h[2, 0]))
    elif isinstance(points_cam, np.ndarray):
        n = points_cam.shape[0]
        pc_h = np.hstack([points_cam, np.ones((n, 1))]).T
        pw_h = T_wc @ pc_h
        return pw_h[:3, :].T.astype(points_cam.dtype)
    return None


if _HAS_NUMBA:
    @njit(cache=True, fastmath=True)
    def _max_axis_rect_numba(bin_img):
        hh, ww = bin_img.shape
        heights = np.zeros(ww, dtype=np.int32)
        best_area = 0
        bx0 = by0 = bw_ = bh_ = 0
        stack = np.empty(ww + 2, dtype=np.int32)

        for row in range(hh):
            for col in range(ww):
                if bin_img[row, col] > 0:
                    heights[col] += 1
                else:
                    heights[col] = 0
            top = 1
            stack[0] = -1
            for col in range(ww + 1):
                cur_h = heights[col] if col < ww else 0
                while stack[top - 1] != -1 and heights[stack[top - 1]] > cur_h:
                    top -= 1
                    h_ = heights[stack[top]]
                    w_ = col - stack[top - 1] - 1
                    area = h_ * w_
                    if area > best_area:
                        best_area = area
                        bx0 = stack[top - 1] + 1
                        by0 = row - h_ + 1
                        bw_ = w_
                        bh_ = h_
                stack[top] = col
                top += 1
        return best_area, bx0, by0, bw_, bh_


def _max_axis_rect(bin_img):
    if _HAS_NUMBA:
        area, x, y, w_, h_ = _max_axis_rect_numba(bin_img)
        return (int(x), int(y), int(w_), int(h_)), int(area)

    hh, ww = bin_img.shape
    heights = np.zeros(ww, dtype=np.int32)
    best_area = 0
    best = (0, 0, 0, 0)
    for row in range(hh):
        heights = np.where(bin_img[row] > 0, heights + 1, 0)
        stack = [-1]
        for col in range(ww + 1):
            cur_h = int(heights[col]) if col < ww else 0
            while stack[-1] != -1 and int(heights[stack[-1]]) > cur_h:
                h_ = int(heights[stack.pop()])
                w_ = col - stack[-1] - 1
                area = h_ * w_
                if area > best_area:
                    best_area = area
                    best = (stack[-1] + 1, row - h_ + 1, w_, h_)
            stack.append(col)
    return best, best_area


def process_one_sample(
        png_path: str,
        depth_path: str,
        ply_path: str,
        fx: float, fy: float, cx: float, cy: float,
        dist_coeffs: np.ndarray,
        mask_path: str,
        shapes_list: list,
        z_prism_offset: float = 150,
        voxel_mm: float = 5,
        max_roi_pts: int = 3000
):
    """
    只返回 best_label、长方体8个世界坐标、中心世界坐标。
    """
    waybill_dict = {}
    Tcw = np.asarray(load_calibration()['camera_to_world_mm'], dtype=np.float64)

    img = png_path if isinstance(png_path, np.ndarray) else cv2.imread(png_path)
    if img is None:
        raise FileNotFoundError(f"彩色图读取失败 {png_path}")
    H, W = img.shape[:2]

    if mask_path not in _MASK_CACHE:
        _MASK_CACHE[mask_path] = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
    mask = _MASK_CACHE[mask_path]
    if mask is None:
        raise FileNotFoundError(f"mask图片读取失败 {mask_path}")
    mask = (mask > 0).astype(np.uint8) * 255

    camera_matrix = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ], dtype=np.float64)

    mask_1_2 = np.zeros((H, W), dtype=np.uint8)
    poly_12_list = []
    LABEL_12_SET = {'carton', 'soft_bag', 'foam'}
    for shape in shapes_list:
        label = shape.get('label', '')
        pts = shape.get('points', [])
        if label in LABEL_12_SET and len(pts) >= 3:
            m = polygon_to_mask(pts, (H, W))
            mask_1_2 = cv2.bitwise_or(mask_1_2, m)
            poly_12_list.append({"points": pts, "label": label})

    intersect_mask = cv2.bitwise_and(mask_1_2, mask)
    if not np.any(intersect_mask > 0):
        return None, None, None, waybill_dict

    depth_raw = depth_path if isinstance(depth_path, np.ndarray) else cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
    if depth_raw is None:
        raise FileNotFoundError(f"深度图读取失败 {depth_path}")
    depth_np = depth_raw.astype(np.float32)

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
            valid_records.append({
                "xyz": xyz,
                "u": u,
                "v": v,
                "poly_mask": poly_mask,
                "label": item["label"],
                "poly_points": item["points"]
            })

    if len(valid_records) == 0:
        return None, None, None, waybill_dict

    valid_records.sort(key=lambda r: r["xyz"][2])
    best_rec = valid_records[0]
    best_label = best_rec["label"]
    best_poly_mask = best_rec["poly_mask"]
    best_u, best_v = best_rec["u"], best_rec["v"]

    for shape in shapes_list:
        if shape.get('label', '') != 'waybill':
            continue
        wb_pts = shape.get('points', [])
        if len(wb_pts) < 3:
            continue
        wb_mask = polygon_to_mask(wb_pts, (H, W))
        inter_wb = cv2.bitwise_and(wb_mask, best_poly_mask)
        if not np.any(inter_wb > 0):
            continue
        pts_arr = np.array(wb_pts, dtype=np.int32).reshape(-1, 1, 2)
        x, y, w, h = cv2.boundingRect(pts_arr)
        if w <= 0 or h <= 0:
            continue
        x, y = max(0, x), max(0, y)
        x2, y2 = min(W, x + w), min(H, y + h)
        if x2 <= x or y2 <= y:
            continue
        crop = img[y:y2, x:x2]
        try:
            output = barcode_wechat_detect(crop)
        except Exception as e:
            print(f"条码识别异常: {e}")
            output = []
        if output:
            waybill_id = output[0].get("data", "")
            if waybill_id:
                waybill_dict = {waybill_id: output}
        break

    pts_raw, mask_1_2_mini, plane_coeff, prism_points, outline_px, z_low, z_high = \
        get_shrunk_mask_prism_fit_plane(
            poly_mask=best_poly_mask,
            centroid_px=(best_u, best_v),
            ply_path=ply_path,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            z_offset_range=z_prism_offset,
            voxel_mm=voxel_mm,
            max_roi_pts=max_roi_pts
        )

    if plane_coeff is None or pts_raw is None:
        return best_label, None, None, waybill_dict

    # ===== 灰度图：用 pts_raw（原始点云），与原版一致 =====
    gray_grid = None
    origin = None
    u_axis = None
    v_axis = None
    u_min = None
    v_min = None

    if isinstance(ply_path, PreparedCloud):
        pts_in_poly = select_projected(pts_raw, ply_path.raw_uv, best_poly_mask, z_low if z_low is not None else -np.inf, z_high if z_high is not None else np.inf)
    else:
        X_all, Y_all, Z_all = pts_raw[:, 0], pts_raw[:, 1], pts_raw[:, 2]
        valid_z = Z_all > 1e-3
        if z_low is not None and z_high is not None:
            valid_z &= (Z_all >= z_low) & (Z_all <= z_high)
        Xv, Yv, Zv = X_all[valid_z], Y_all[valid_z], Z_all[valid_z]

        if Xv.shape[0] > 0:
            if isinstance(ply_path, PreparedCloud):
                proj_pixels = ply_path.raw_uv[valid_z]
            else:
                pts_norm = np.column_stack([Xv / Zv, Yv / Zv, np.ones_like(Zv)])
                proj_pixels, _ = project_xyz_fast(pts_norm, camera_matrix, dist_coeffs)
            proj_uv = proj_pixels[:, 0, :]
            proj_u = np.clip(np.round(proj_uv[:, 0]).astype(np.int32), 0, W - 1)
            proj_v = np.clip(np.round(proj_uv[:, 1]).astype(np.int32), 0, H - 1)
            inside = best_poly_mask[proj_v, proj_u] > 0
            pts_in_poly = np.column_stack([Xv, Yv, Zv])[inside]

    if True:
        if pts_in_poly.shape[0] >= 3:
            a, b, c, d = plane_coeff
            n = np.array([a, b, c], dtype=np.float64)
            norm = np.linalg.norm(n)
            signed_dist = (pts_in_poly @ n + d) / norm
            dist = np.abs(signed_dist)
            proj_3d = pts_in_poly - (signed_dist[:, None] / (norm ** 2)) * n

            origin = proj_3d.mean(axis=0)

            u_axis = np.array([b, -a, 0.0])
            nu = np.linalg.norm(u_axis)
            if nu < 1e-9:
                u_axis = np.array([1.0, 0.0, 0.0])
            else:
                u_axis = u_axis / nu
            v_axis = np.cross(n / norm, u_axis)
            v_axis = v_axis / np.linalg.norm(v_axis)

            proj_2d = np.column_stack([
                (proj_3d - origin) @ u_axis,
                (proj_3d - origin) @ v_axis
            ])

            u_quant = np.round(proj_2d[:, 0] * 2).astype(np.int32)
            v_quant = np.round(proj_2d[:, 1] * 2).astype(np.int32)
            u_min, u_max = int(u_quant.min()), int(u_quant.max())
            v_min, v_max = int(v_quant.min()), int(v_quant.max())
            grid_W = u_max - u_min + 1
            grid_H = v_max - v_min + 1

            d_min, d_max = float(dist.min()), float(dist.max())
            if d_max - d_min < 1e-9:
                vals = np.full(dist.shape, 128, dtype=np.uint8)
            else:
                vals = np.clip(
                    255.0 * (1.0 - (dist - d_min) / (d_max - d_min)),
                    0, 255
                ).astype(np.uint8)

            linear_idx = (v_quant - v_min) * grid_W + (u_quant - u_min)
            order = np.argsort(vals)
            gray_grid = np.zeros(grid_H * grid_W, dtype=np.uint8)
            gray_grid[linear_idx[order]] = vals[order]
            gray_grid = gray_grid.reshape(grid_H, grid_W)

    if gray_grid is None:
        return best_label, None, None, waybill_dict

    # ===== 最大旋转内接矩形（结果同原版，_max_axis_rect 已 numba 加速）=====
    if best_label in ("carton", "foam"):
        _, bin_mask = cv2.threshold(gray_grid, 220, 255, cv2.THRESH_BINARY)
    else:
        _, bin_mask = cv2.threshold(gray_grid, 20, 255, cv2.THRESH_BINARY)

    kernel7 = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    bin_dil = cv2.dilate(bin_mask, kernel7)
    bin_ero = cv2.erode(bin_dil, kernel7)

    contours, _ = cv2.findContours(bin_ero, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        return best_label, None, None, waybill_dict

    max_cnt = max(contours, key=cv2.contourArea)
    max_region_mask = np.zeros_like(bin_ero)
    cv2.drawContours(max_region_mask, [max_cnt], -1, 255, -1)

    h_img, w_img = max_region_mask.shape
    MAX_DIM = 400
    scale = min(1.0, MAX_DIM / max(h_img, w_img))
    if scale < 1.0:
        small_size = (max(1, int(round(w_img * scale))),
                      max(1, int(round(h_img * scale))))
        small_mask = cv2.resize(max_region_mask, small_size,
                                interpolation=cv2.INTER_NEAREST)
    else:
        small_mask = max_region_mask

    h_s, w_s = small_mask.shape
    diag = int(np.ceil(np.hypot(h_s, w_s))) + 4

    def _evaluate(theta_deg):
        M = cv2.getRotationMatrix2D((w_s / 2.0, h_s / 2.0), theta_deg, 1.0)
        M[0, 2] += diag / 2.0 - w_s / 2.0
        M[1, 2] += diag / 2.0 - h_s / 2.0
        rot = cv2.warpAffine(small_mask, M, (diag, diag),
                             flags=cv2.INTER_NEAREST, borderValue=0)
        (x, y, w_, h_), area = _max_axis_rect(rot)
        pts_rot = np.array([
            [x, y],
            [x + w_ - 1, y],
            [x + w_ - 1, y + h_ - 1],
            [x, y + h_ - 1]
        ], dtype=np.float32)
        M_inv = cv2.invertAffineTransform(M)
        pts_orig = cv2.transform(pts_rot.reshape(-1, 1, 2), M_inv).reshape(-1, 2)
        return area, pts_orig

    best_area_small = -1
    best_theta = 0.0
    best_pts_small = None
    for theta in np.arange(0.0, 90.0, 5.0):
        area, pts = _evaluate(float(theta))
        if area > best_area_small:
            best_area_small = area
            best_theta = float(theta)
            best_pts_small = pts
    for theta in np.arange(max(0.0, best_theta - 5.0),
                           min(90.0, best_theta + 5.0) + 1e-6, 1.0):
        area, pts = _evaluate(float(theta))
        if area > best_area_small:
            best_area_small = area
            best_theta = float(theta)
            best_pts_small = pts

    if best_pts_small is None:
        return best_label, None, None, waybill_dict

    quad_rect_pts = np.round(best_pts_small / scale).astype(np.int32)

    # ===== 网格像素 -> 相机3D -> 投影img像素 -> 深度反算3D =====
    quad_grid_px = quad_rect_pts.astype(np.float64)
    uv_mm = np.column_stack([
        (quad_grid_px[:, 0] + float(u_min)) * 0.5,
        (quad_grid_px[:, 1] + float(v_min)) * 0.5
    ])
    quad_3d_cam = (origin[None, :]
                   + uv_mm[:, 0:1] * u_axis[None, :]
                   + uv_mm[:, 1:2] * v_axis[None, :])

    dummy_rvec = np.zeros(3)
    dummy_tvec = np.zeros(3)
    proj_px, _ = cv2.projectPoints(quad_3d_cam.astype(np.float64),
                                   dummy_rvec, dummy_tvec,
                                   camera_matrix, dist_coeffs)
    quad_img_uv = proj_px.reshape(-1, 2)
    quad_img_uv_int = np.clip(np.round(quad_img_uv).astype(np.int32),
                              [0, 0], [W - 1, H - 1])

    quad_3d_from_depth = np.full((4, 3), np.nan, dtype=np.float64)
    for i in range(4):
        uu, vv = int(quad_img_uv_int[i, 0]), int(quad_img_uv_int[i, 1])
        d_val = float(depth_np[vv, uu])
        xyz = pixel2world_with_distortion(uu, vv, d_val, camera_matrix, dist_coeffs)
        if xyz is not None:
            quad_3d_from_depth[i] = xyz

    if np.any(np.isnan(quad_3d_from_depth)):
        return best_label, None, None, waybill_dict

    # ===== poly_points 步长5采样到平面距离，取最大值 = 长方体高度 =====
    poly_pts_arr = np.asarray(best_rec["poly_points"], dtype=np.float64)[::5]
    n_pts = poly_pts_arr.shape[0]
    if n_pts == 0:
        return best_label, None, None, waybill_dict

    us = np.clip(np.round(poly_pts_arr[:, 0]).astype(np.int32), 0, W - 1)
    vs = np.clip(np.round(poly_pts_arr[:, 1]).astype(np.int32), 0, H - 1)
    ds = depth_np[vs, us].astype(np.float64)
    valid = ds > 0
    if not np.any(valid):
        return best_label, None, None, waybill_dict

    pts_in = poly_pts_arr[valid].reshape(-1, 1, 2)
    und = cv2.undistortPoints(pts_in, camera_matrix, dist_coeffs).reshape(-1, 2)
    Zc = ds[valid]
    Xc = und[:, 0] * Zc
    Yc = und[:, 1] * Zc

    a, b, c, d = plane_coeff
    dists = np.abs(a * Xc + b * Yc + c * Zc + d)
    box_height = float(np.max(dists))

    n = np.array([a, b, c], dtype=np.float64)
    n_norm = np.linalg.norm(n)
    if n_norm < 1e-9:
        n = np.array([0.0, 0.0, 1.0])
    else:
        n = n / n_norm
        if n[2] < 0:
            n = -n
        if abs(n[2]) < 1e-6:
            n = np.array([0.0, 0.0, 1.0])

    bottom = quad_3d_from_depth.astype(np.float64).reshape(4, 3)
    top = bottom + box_height * n[None, :]
    box_8pts = np.vstack([bottom, top])
    center = box_8pts.mean(axis=0)

    box_8pts_world = cam_points_to_world(box_8pts, Tcw)
    center_world = cam_points_to_world(center.reshape(1, 3), Tcw)[0]

    return best_label, box_8pts_world, center_world, waybill_dict


def run_detection(
        rgb_path: str,
        depth_path: str,
        ply_path: str,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray
):
    Z_PRISM_OFFSET = 150
    VOXEL_MM = 5
    MAX_ROI_PTS = 3000

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    from predict_yolo import yoloDetect
    t_yolo = time.perf_counter()
    yolo_list = yoloDetect(rgb_path)
    _LAST_TIMING["yolo_ms"] = round((time.perf_counter() - t_yolo) * 1000, 1)
    t_fit = time.perf_counter()

    clear_ply_cache()
    best_label, box_8pts_world, center_world, waybill_dict = process_one_sample(
        png_path=rgb_path,
        depth_path=depth_path,
        ply_path=ply_path,
        fx=fx, fy=fy, cx=cx, cy=cy,
        dist_coeffs=dist_coeffs,
        mask_path=os.environ.get('MASK_PATH', '/assets/mask1.png'),
        shapes_list=yolo_list,
        z_prism_offset=Z_PRISM_OFFSET,
        voxel_mm=VOXEL_MM,
        max_roi_pts=MAX_ROI_PTS
    )
    _LAST_TIMING["fit_3d_ms"] = round((time.perf_counter() - t_fit) * 1000, 1)
    return best_label, box_8pts_world, center_world, waybill_dict



def serve_live(camera_matrix, dist_coeffs):
    import argparse
    import threading
    import rclpy
    import message_filters
    from sensor_msgs.msg import Image, PointCloud2
    from std_srvs.srv import Trigger
    from std_msgs.msg import String
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    max_frame_age_s = float(os.environ.get('MAX_FRAME_AGE_SEC', '3'))
    if not np.isfinite(max_frame_age_s) or max_frame_age_s <= 0:
        raise ValueError('MAX_FRAME_AGE_SEC must be positive and finite')
    cloud_stride = int(os.environ.get('CLOUD_STRIDE', '1'))
    if not 1 <= cloud_stride <= 32:
        raise ValueError('CLOUD_STRIDE must be 1..32')
    sync_slop_s = float(os.environ.get('SYNC_SLOP_SEC', '0.08'))
    if not np.isfinite(sync_slop_s) or not 0 < sync_slop_s <= 1:
        raise ValueError('SYNC_SLOP_SEC must be in (0, 1]')
    sync_stall_restart_s = float(os.environ.get('SYNC_STALL_RESTART_SEC', '45'))
    if not np.isfinite(sync_stall_restart_s) or sync_stall_restart_s <= max_frame_age_s:
        raise ValueError('SYNC_STALL_RESTART_SEC must exceed MAX_FRAME_AGE_SEC')
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['service'], default='service')
    parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('sf_sorting_server_node')
    group = ReentrantCallbackGroup()
    lock = threading.Lock()
    foreground = threading.Event()
    latest = [None]
    last_prepared = [None]
    pending = [None]
    wake = threading.Event()
    def receive(rgb, depth, points):
        pending[0] = (rgb, depth, points, time.monotonic())
        wake.set()
    def prepare_frames():
        while rclpy.ok():
            if not wake.wait(0.2):
                continue
            if foreground.is_set():
                time.sleep(0.02)
                continue
            wake.clear()
            frame = pending[0]
            try:
                rgb, depth, points, received = frame
                cloud = PreparedCloud(cloud_array(points)[::cloud_stride], camera_matrix, dist_coeffs)
                latest[0] = (rgb, depth, cloud, received)
                last_prepared[0] = time.monotonic()
                age = time.monotonic() - received
                if age > 3:
                    node.get_logger().warning(
                        f'Prepared camera frame already {age:.1f}s old; '
                        f'point cloud preparation {cloud.prepare_ms:.1f}ms')
            except Exception as exc:
                node.get_logger().error('Camera preprocessing failed: ' + str(exc))
    def restart_on_stalled_sync():
        # A camera hotplug can leave an existing DDS subscription silent even
        # while fresh subscribers receive all three topics. Restart this
        # process so Docker recreates the subscriptions from scratch.
        while rclpy.ok():
            time.sleep(5)
            prepared = last_prepared[0]
            if prepared is not None and not foreground.is_set() and time.monotonic() - prepared > sync_stall_restart_s:
                node.get_logger().error(
                    f'No prepared synchronized frame for {time.monotonic() - prepared:.1f}s; '
                    'restarting sorting process to rebuild ROS subscriptions')
                os._exit(75)
    subs = [message_filters.Subscriber(node, typ, topic, qos_profile=10, callback_group=group)
            for typ, topic in [(Image, os.environ.get('RGB_TOPIC', '/vision/top_camera/rgb')), (Image, os.environ.get('DEPTH_TOPIC', '/vision/top_camera/depth')), (PointCloud2, os.environ.get('POINTS_TOPIC', '/vision/top_camera/points'))]]
    sync = message_filters.ApproximateTimeSynchronizer(subs, 30, sync_slop_s)
    sync.registerCallback(receive)
    pub = node.create_publisher(String, '/sf_sorting/result', 10)
    def image_array(msg):
        types = {'rgb8': (np.uint8, 3), 'bgr8': (np.uint8, 3), '16UC1': (np.uint16, 1), 'mono16': (np.uint16, 1), '32FC1': (np.float32, 1)}
        dtype, channels = types[msg.encoding]
        dt = np.dtype(dtype).newbyteorder('>' if msg.is_bigendian else '<')
        arr = np.ndarray((msg.height, msg.width, channels), dtype=dt, buffer=msg.data,
                         strides=(msg.step, dt.itemsize * channels, dt.itemsize)).copy()
        if channels == 1:
            arr = arr[:, :, 0].astype(np.float32)
            return arr * 1000 if msg.encoding == '32FC1' else arr
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else arr
    def cloud_array(msg):
        fields = {f.name: f for f in msg.fields}
        if any(fields[k].datatype != 7 for k in ('x', 'y', 'z')):
            raise ValueError('PointCloud2 xyz must be FLOAT32 meters')
        dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': [('>' if msg.is_bigendian else '<') + 'f4'] * 3,
                       'offsets': [fields[k].offset for k in ('x', 'y', 'z')], 'itemsize': msg.point_step})
        if [fields[k].offset for k in ('x', 'y', 'z')] == [0, 4, 8] and msg.row_step == msg.width * msg.point_step:
            return np.ndarray((msg.height * msg.width, 3), dtype=('>' if msg.is_bigendian else '<') + 'f4', buffer=msg.data, strides=(msg.point_step, 4))
        raw = np.ndarray((msg.height, msg.width), dtype=dt, buffer=msg.data, strides=(msg.row_step, msg.point_step))
        return np.stack([raw[k].ravel() for k in ('x', 'y', 'z')], axis=1)
    def handle(request, response):
        started = time.monotonic()
        with lock:
            try:
                wait_until = time.monotonic() + 0.1
                frame = latest[0]
                while (frame is None or time.monotonic() - frame[3] > max_frame_age_s) and time.monotonic() < wait_until:
                    time.sleep(0.02)
                    frame = latest[0]
                if frame is None or time.monotonic() - frame[3] > max_frame_age_s:
                    raise RuntimeError('No fresh synchronized camera frame; retry when camera preprocessing is ready')
                rgb, depth, points, received = frame
                frame_age_ms = round((time.monotonic() - received) * 1000, 1)
                foreground.set()
                t_parse = time.perf_counter()
                bgr, dep, xyz = image_array(rgb), image_array(depth), points
                parse_ms = round((time.perf_counter() - t_parse) * 1000, 1)
                label, box, center, waybill_dict = run_detection(bgr, dep, xyz, camera_matrix, dist_coeffs)
                ok = box is not None and center is not None
                if ok and (np.shape(box) != (8, 3) or not np.isfinite(box).all() or not np.isfinite(center).all()):
                    raise ValueError('Invalid cuboid or center')
                result = {'success': ok, 'best_label': label,
                          'center_world': center.tolist() if ok else None,
                          'box_8pts_world': box.tolist() if ok else None,
                          'waybill_dict': waybill_dict}
                result['timing'] = dict(_LAST_TIMING, preprocess_ms=xyz.prepare_ms, frame_age_ms=frame_age_ms, parse_ms=parse_ms, wait_frame_ms=round((t_parse-started)*1000, 1))
                response.success = ok
            except Exception as exc:
                response.success = False
                result = {'success': False, 'error': str(exc)}
                node.get_logger().error(str(exc))
            foreground.clear()
            result.setdefault('timing', {})['server_total_ms'] = round((time.monotonic() - started) * 1000, 1)
            response.message = json.dumps(result, ensure_ascii=False, allow_nan=False)
            msg = String()
            msg.data = response.message
            pub.publish(msg)
            print(response.message, flush=True)
            return response
    # Complete imports, model initialization and JIT before exposing the service.
    cv2.setNumThreads(2)
    import torch
    torch.set_num_threads(2)
    valid_points_mm(np.ones((4, 3), dtype=np.float32))
    valid_points_mm(np.ones((4, 4), dtype=np.float32)[:, :3])
    from predict_yolo import yoloDetect
    yoloDetect(np.zeros((2160, 3840, 3), dtype=np.uint8))
    import shapely.geometry
    _max_axis_rect(np.ones((16, 16), dtype=np.uint8))
    voxel_fast(np.ones((4, 3), dtype=np.float32), 5)
    project_xyz_fast(np.ones((4, 3), dtype=np.float32), camera_matrix, dist_coeffs)
    project_xyz_fast(np.ones((4, 3), dtype=np.float64), camera_matrix, dist_coeffs)
    select_projected(np.ones((4, 3), dtype=np.float32), np.ones((4, 1, 2), dtype=np.float32), np.ones((16,16), dtype=np.uint8), -np.inf, np.inf)
    select_projected(np.ones((4, 3), dtype=np.float64), np.ones((4, 1, 2), dtype=np.float64), np.ones((16,16), dtype=np.uint8), -np.inf, np.inf)
    worker = threading.Thread(target=prepare_frames, daemon=True)
    worker.start()
    watchdog = threading.Thread(target=restart_on_stalled_sync, daemon=True)
    watchdog.start()
    # Do not advertise readiness before the first synchronized input is prepared.
    warm_deadline = time.monotonic() + 30
    while latest[0] is None and time.monotonic() < warm_deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
    srv = node.create_service(Trigger, '/sf_sorting/get_result', handle, callback_group=group)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    print('SFsorting1022 live service ready', flush=True)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    calibration = load_calibration()
    camera_matrix = np.asarray(calibration['camera_matrix'], dtype=np.float64)
    dist_coeffs = np.asarray(calibration['dist_coeffs'], dtype=np.float64)
    serve_live(camera_matrix, dist_coeffs)

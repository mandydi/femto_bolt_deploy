import os
import json
import time
import cv2
import numpy as np
from ultralytics import YOLO

BEST_WEIGHT = r"D:\sf_pic\02_模型训练与结果\stack_cage_yolo26n_v2\weights\best.pt"
RGB_DIR = r"D:\sf_pic\01_图像数据与标注\RGB"
DEPTH_DIR = r"D:\sf_pic\04_标定与点云\码垛点云数据\DEPTH"
OUT_PATH = r"D:\sf_pic\02_模型训练与结果\stack_cage_yolo26n_v2\test_results_4k\depth_mask_check.json"

# 测试未参与训练的两张验证样本
val_ids = ["1794223404480745", "1794223442346871"]

model = YOLO(BEST_WEIGHT)

results_summary = []
for fid in val_ids:
    rgb_path = os.path.join(RGB_DIR, f"color_{fid}.png")
    depth_path = os.path.join(DEPTH_DIR, f"depth_{fid}.png")

    rgb = cv2.imdecode(np.fromfile(rgb_path, dtype=np.uint8), cv2.IMREAD_COLOR)
    depth = cv2.imdecode(np.fromfile(depth_path, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    h, w = rgb.shape[:2]

    t0 = time.time()
    preds = model.predict(rgb, imgsz=640, conf=0.25, iou=0.7, device="cpu", retina_masks=True, verbose=False)
    infer_ms = (time.time() - t0) * 1000.0

    r = preds[0]
    frame_info = {
        "frame_id": fid,
        "infer_ms": round(infer_ms, 1),
        "objects": []
    }

    if r.masks is not None and r.boxes is not None:
        masks_np = r.masks.data.cpu().numpy()
        clss = r.boxes.cls.cpu().numpy().astype(int)
        confs = r.boxes.conf.cpu().numpy()
        boxes = r.boxes.xyxy.cpu().numpy()

        for m, c, cf, box in zip(masks_np, clss, confs, boxes):
            cname = model.names[c]
            m_bin = (m > 0.5)
            if m_bin.shape != (h, w):
                m_bin = cv2.resize(m_bin.astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST) > 0

            # 提取掩码内的有效深度点 (depth > 0)
            valid_mask = m_bin & (depth > 0)
            valid_depths = depth[valid_mask]

            obj_stats = {
                "label": cname,
                "confidence": round(float(cf), 4),
                "bbox_xyxy": [round(float(x), 1) for x in box],
                "mask_pixels": int(m_bin.sum()),
                "valid_depth_points": int(valid_depths.size),
            }
            if valid_depths.size > 0:
                obj_stats.update({
                    "depth_min_mm": int(np.percentile(valid_depths, 1)),
                    "depth_median_mm": int(np.median(valid_depths)),
                    "depth_mean_mm": round(float(np.mean(valid_depths)), 1),
                    "depth_max_mm": int(np.percentile(valid_depths, 99)),
                })
            frame_info["objects"].append(obj_stats)

    results_summary.append(frame_info)
    print(json.dumps(frame_info, ensure_ascii=False, indent=2))

with open(OUT_PATH, "w", encoding="utf-8") as f:
    json.dump(results_summary, f, indent=2, ensure_ascii=False)
print(f"Saved depth-mask check to {OUT_PATH}")

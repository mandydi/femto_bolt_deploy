import os
os.environ["ULTRALYTICS_OFFLINE"] = "1"
os.environ["ULTRALYTICS_HIDE_MESSAGE"] = "1"

import sys
sys.path.append("/home/huang0826/SF_sorting/ultralytics")
from ultralytics import YOLO


def _box_polygon(xyxy: list[float]) -> list[list[float]]:
    """Represent a detection box as a four-point polygon."""
    x1, y1, x2, y2 = xyxy
    return [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]

_MODEL = None

def get_model(model_name='packet_best.pt'):
    global _MODEL
    if _MODEL is None:
        if not os.path.exists(model_name):
            script_dir = os.path.dirname(os.path.abspath(__file__))
            cand = os.path.join(script_dir, model_name)
            if os.path.exists(cand):
                model_name = cand
        _MODEL = YOLO(model_name)
    return _MODEL

def yoloDetect(image_source, model_name='packet_best.pt'):
    model = get_model(model_name)
    result = model.predict(
        source=image_source,
        device="cpu",
        imgsz=1280,
        conf=0.25,
        iou=0.7,
        save=False,
        save_txt=False,
        verbose=False
    )[0]
    yolo_list: list[dict[str, object]] = []
    if result.boxes is None or len(result.boxes) == 0:
        print('未检测到任何目标')
        return yolo_list
    polygons = result.masks.xy if result.masks is not None else []
    for index, box in enumerate(result.boxes):
        class_id = int(box.cls.item())
        if index < len(polygons) and len(polygons[index]) >= 3:
            points = polygons[index].tolist()
        else:
            points = _box_polygon([float(value) for value in box.xyxy[0].tolist()])
        yolo_list.append(
            {
                "label": str(result.names[class_id]),
                "points": [[float(x), float(y)] for x, y in points],
            }
        )
    return yolo_list

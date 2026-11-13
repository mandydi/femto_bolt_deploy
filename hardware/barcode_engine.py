"""
工业级多码制条码/二维码识别引擎 (Barcode & QRCode Engine)
版本隔离模块 - 支持 Code128, Code39, QR Code, DataMatrix 等
兼容: zxing-cpp, OpenCV WeChatQRCode (带模型), pyzbar, cv2.barcode
"""
import os
import cv2
import numpy as np

_ZXING_AVAILABLE = False
try:
    import zxingcpp
    _ZXING_AVAILABLE = True
except ImportError:
    pass

_PYZBAR_AVAILABLE = False
try:
    import pyzbar.pyzbar as pyzbar
    _PYZBAR_AVAILABLE = True
except ImportError:
    pass

_WECHAT_DETECTOR = None
def get_wechat_detector(model_dir=None):
    global _WECHAT_DETECTOR
    if _WECHAT_DETECTOR is not None:
        return _WECHAT_DETECTOR
    
    # 候选模型路径
    cand_dirs = [
        model_dir,
        '/home/caseim/wechat_models',
        '/tmp/wechat_models',
        os.path.join(os.path.dirname(__file__), 'wechat_models')
    ]
    for d in cand_dirs:
        if d and os.path.exists(d):
            dp = os.path.join(d, 'detect.prototxt')
            dm = os.path.join(d, 'detect.caffemodel')
            sp = os.path.join(d, 'sr.prototxt')
            sm = os.path.join(d, 'sr.caffemodel')
            if os.path.exists(dp) and os.path.exists(dm) and os.path.exists(sp) and os.path.exists(sm):
                try:
                    _WECHAT_DETECTOR = cv2.wechat_qrcode.WeChatQRCode(dp, dm, sp, sm)
                    return _WECHAT_DETECTOR
                except Exception:
                    pass
    if hasattr(cv2, 'wechat_qrcode'):
        try:
            _WECHAT_DETECTOR = cv2.wechat_qrcode.WeChatQRCode()
        except Exception:
            pass
    return _WECHAT_DETECTOR


def decode_image_single(img):
    """单张图像快速解码，依次尝试 zxing-cpp -> wechat_qrcode -> pyzbar -> cv2.barcode"""
    results = []
    seen = set()

    # 1. zxing-cpp (最强支持 Code 128 / Code 39 / QR / DataMatrix)
    if _ZXING_AVAILABLE:
        try:
            hits = zxingcpp.read_barcodes(img)
            for h in hits:
                txt = h.text.strip()
                if txt and txt not in seen:
                    seen.add(txt)
                    results.append({"data": txt, "type": str(h.format), "engine": "zxingcpp"})
        except Exception:
            pass

    # 2. WeChatQRCode (强化低质量 QR 码)
    detector = get_wechat_detector()
    if detector is not None:
        try:
            texts, points = detector.detectAndDecode(img)
            for t in texts:
                txt = t.strip()
                if txt and txt not in seen:
                    seen.add(txt)
                    results.append({"data": txt, "type": "QRCODE", "engine": "wechat_qrcode"})
        except Exception:
            pass

    # 3. pyzbar 兜底
    if _PYZBAR_AVAILABLE:
        try:
            hits = pyzbar.decode(img)
            for h in hits:
                txt = h.data.decode('utf-8', errors='ignore').strip()
                if txt and txt not in seen:
                    seen.add(txt)
                    results.append({"data": txt, "type": str(h.type), "engine": "pyzbar"})
        except Exception:
            pass

    # 4. cv2.barcode 兜底
    if hasattr(cv2, 'barcode') and not results:
        try:
            bd = cv2.barcode.BarcodeDetector()
            ok, texts, types, _ = bd.detectAndDecode(img)
            if ok:
                for t, tp in zip(texts, types):
                    txt = t.strip()
                    if txt and txt not in seen:
                        seen.add(txt)
                        results.append({"data": txt, "type": str(tp), "engine": "cv2.barcode"})
        except Exception:
            pass

    return results


def decode_barcode_robust(img, enable_rotation=True, enable_scaling=True, enable_filters=True):
    """
    鲁棒多策略解码：
    针对反光、条形码细密模糊、旋转 90/180/270 度进行强化处理。
    """
    if img is None or img.size == 0:
        return []

    # 第一轮：原始输入快速尝试（毫秒级，0 度无额外开销）
    res = decode_image_single(img)
    if res:
        return res

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
    h, w = gray.shape[:2]

    # 构建图像候选流水线
    candidates = [("gray", gray)]

    # 1. 尺度缩放：顺丰单号条码若过小或过大，双三次插值
    if enable_scaling:
        max_dim = max(h, w)
        if max_dim < 600:
            scale = 2.0
            scaled = cv2.resize(gray, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_CUBIC)
            candidates.append(("scaled_2x", scaled))
        elif max_dim > 2500:
            scale = 0.5
            scaled = cv2.resize(gray, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
            candidates.append(("scaled_0.5x", scaled))

    # 2. 图像增强：自适应直方图均衡化 (CLAHE) 对抗反光和局部阴影
    if enable_filters:
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
        clahe_img = clahe.apply(candidates[-1][1])
        candidates.append(("clahe", clahe_img))

        # 锐化
        kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]], dtype=np.float32)
        sharpened = cv2.filter2D(candidates[-1][1], -1, kernel)
        candidates.append(("sharpened", sharpened))

    # 遍历候选图与旋转角度 (0, 90, 270, 180)
    rotations = [0, 90, 270, 180] if enable_rotation else [0]
    
    for tag, c_img in candidates:
        for angle in rotations:
            if angle == 0:
                work_img = c_img
            elif angle == 90:
                work_img = cv2.rotate(c_img, cv2.ROTATE_90_CLOCKWISE)
            elif angle == 270:
                work_img = cv2.rotate(c_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
            elif angle == 180:
                work_img = cv2.rotate(c_img, cv2.ROTATE_180)

            hits = decode_image_single(work_img)
            if hits:
                return hits

    return []

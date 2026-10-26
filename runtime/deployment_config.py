"""Validated per-device calibration; never silently reuse 8001 geometry."""
import json
import math
import os
from functools import lru_cache


@lru_cache(maxsize=1)
def load_calibration():
    path = os.environ.get('CALIBRATION_PATH', '/config/calibration.json')
    with open(path, encoding='utf-8') as stream:
        value = json.load(stream)
    for key, rows, cols in [('camera_matrix', 3, 3), ('camera_to_world_mm', 4, 4)]:
        matrix = value.get(key)
        if not isinstance(matrix, list) or len(matrix) != rows or any(
            not isinstance(row, list) or len(row) != cols or any(
                not isinstance(x, (float, int)) or isinstance(x, bool) or not math.isfinite(x)
                for x in row) for row in matrix
        ):
            raise ValueError(f'{key} must be a finite {rows}x{cols} matrix')
    distortion = value.get('dist_coeffs', [])
    if len(distortion) != 8 or any(not isinstance(x, (int, float)) or not math.isfinite(x) for x in distortion):
        raise ValueError('dist_coeffs requires 8 finite rational-model coefficients')
    if value.get('image_size') != [3840, 2160]:
        raise ValueError('This baseline requires 3840x2160; changing resolution needs algorithm validation')
    if value['camera_matrix'][0][0] <= 0 or value['camera_matrix'][1][1] <= 0:
        raise ValueError('Camera focal lengths must be positive')
    if value['camera_to_world_mm'][3] != [0, 0, 0, 1]:
        raise ValueError('Invalid homogeneous transform last row')
    if value.get('validated_for_deployment') is not True:
        raise ValueError('Supply this device calibration and mark validated_for_deployment only after geometric verification')
    if not value.get('world_frame') or value['world_frame'].startswith('REPLACE'):
        raise ValueError('Specify the verified world_frame')
    return value

#!/usr/bin/env python3
"""Create an isolated camera-frame diagnostic config from live CameraInfo."""
import json
import pathlib
import sys

info = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding='utf-8'))
if (info['width'], info['height']) != (3840, 2160):
    raise SystemExit('RGB CameraInfo must be 3840x2160')
if len(info['k']) != 9 or len(info['d']) != 8:
    raise SystemExit('RGB CameraInfo must contain K[9] and D[8]')
value = {
    'camera_matrix': [info['k'][0:3], info['k'][3:6], info['k'][6:9]],
    'dist_coeffs': info['d'],
    'image_size': [info['width'], info['height']],
    'camera_to_world_mm': [[1, 0, 0, 0], [0, 1, 0, 0],
                           [0, 0, 1, 0], [0, 0, 0, 1]],
    'validated_for_deployment': True,
    'world_frame': 'diagnostic_camera_optical_mm',
    'diagnostic_only': True,
}
target = pathlib.Path(sys.argv[2])
target.parent.mkdir(parents=True, exist_ok=True)
target.write_text(json.dumps(value, indent=2), encoding='utf-8')
print('Diagnostic CameraInfo config created; robot/world extrinsics were not applied.')

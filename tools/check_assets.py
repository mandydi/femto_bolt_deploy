#!/usr/bin/env python3
"""Check transferred private assets without importing ML/ROS dependencies."""
import hashlib
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else 'assets')
manifest = json.loads((root / 'manifest.json').read_text(encoding='utf-8'))
required = {'packet_best.pt', 'mask1.png'}
if not required.issubset(manifest):
    raise SystemExit('Asset manifest must include packet_best.pt and mask1.png')
for name, expected in manifest.items():
    if pathlib.Path(name).name != name:
        raise SystemExit('Manifest paths must be basenames')
    path = root / name
    actual = hashlib.sha256(path.read_bytes()).hexdigest()
    if actual != expected:
        raise SystemExit(f'Hash mismatch: {name}')
    print(f'PASS {name} sha256={actual}')

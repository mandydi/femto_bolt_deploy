"""Restore SDK symlinks flattened by a Windows checkout; reject LFS pointers."""
from pathlib import Path
import sys

root = Path(sys.argv[1]).resolve()
for path in root.rglob("*"):
    if not path.is_file() or path.is_symlink() or ".so" not in path.name:
        continue
    if path.stat().st_size > 512:
        continue
    target_text = path.read_text().strip()
    if target_text.startswith("version https://git-lfs.github.com/spec/"):
        raise SystemExit(f"SDK binary missing (Git LFS pointer): {path}")
    target = (path.parent / target_text).resolve()
    if not target.is_relative_to(root) or not target.is_file():
        raise SystemExit(f"Invalid SDK link: {path} -> {target_text}")
    path.unlink()
    path.symlink_to(target_text)

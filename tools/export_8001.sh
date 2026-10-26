#!/usr/bin/env bash
# Run on 8001. Reads containers and exports assets; never stops/recreates them.
set -euo pipefail
out="${1:?Usage: bash tools/export_8001.sh ABSOLUTE_OUTPUT_DIR [--images]}"
[[ "$out" = /* && ! -e "$out" ]] || { echo 'Use a NEW absolute output directory' >&2; exit 1; }
umask 077
mkdir -p "$out/assets"
camera_id=$(docker inspect -f '{{.Image}}' orbbec-femto-bolt)
# numba/llvmlite were installed in the container writable layer, not its base image.
# --pause=false avoids Docker's default temporary freeze of the production service.
commit_output=$(docker commit --pause=false sf_sorting_server)
sorting_id=$(printf '%s\n' "$commit_output" | grep -E '^sha256:[0-9a-f]{64}$' | tail -1)
[[ "$sorting_id" =~ ^sha256:[0-9a-f]{64}$ ]] || { echo 'Cannot identify committed image' >&2; exit 1; }
source_dir=$(docker inspect sf_sorting_server | python3 -c 'import json,sys; d=json.load(sys.stdin)[0]; w=d["Config"]["WorkingDir"]; print(next(m["Source"] for m in d["Mounts"] if m["Destination"]==w))')
cp "$source_dir/packet_best.pt" "$out/assets/"
cp "$source_dir/mask1.png" "$out/assets/"
python3 - "$out/assets" "$camera_id" "$sorting_id" <<'PY'
import hashlib,json,pathlib,sys
p=pathlib.Path(sys.argv[1])
(p/'manifest.json').write_text(json.dumps({n:hashlib.sha256((p/n).read_bytes()).hexdigest() for n in ['packet_best.pt','mask1.png']},indent=2)+'\n')
(p.parent/'image-ids.json').write_text(json.dumps({'camera':sys.argv[2],'sorting':sys.argv[3]},indent=2)+'\n')
PY
docker exec sf_sorting_server python3 -m pip freeze > "$out/observed-pip-freeze.txt"
if [[ "${2:-}" == --images ]]; then
    # Export by container image ID, never by a mutable tag.
    docker save "$camera_id" "$sorting_id" | gzip -1 > "$out/images.tar.gz"
fi
(cd "$out"; sha256sum assets/* image-ids.json observed-pip-freeze.txt > SHA256SUMS; if [[ -f images.tar.gz ]]; then sha256sum images.tar.gz >> SHA256SUMS; fi)
echo "Exported to $out. Keep this bundle private; transfer outside Git."
echo 'Sorting snapshot includes its writable layer; bind-mounted code and weights are supplied separately.'

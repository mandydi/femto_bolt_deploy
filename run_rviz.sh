#!/usr/bin/env bash
# View the live camera topics on the workstation desktop using the imported sorting image.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONFIG="$ROOT/femto_bolt_deploy.rviz"
[[ -f "$CONFIG" ]] || { echo "Missing RViz config: $CONFIG" >&2; exit 1; }
if docker info >/dev/null 2>&1; then D=(docker); else D=(sudo docker); fi
"${D[@]}" image inspect orbbec-femto-bolt:sorting-local >/dev/null 2>&1 || {
  echo 'Missing sorting-local image. Import the private deployment bundle first.' >&2; exit 1;
}
DISP="${DISPLAY:-:0}"
if [[ -n "${ROS_DOMAIN_ID:-}" ]]; then
  DOMAIN="$ROS_DOMAIN_ID"
else
  DOMAIN="$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' femto-camera 2>/dev/null | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1 || true)"
  DOMAIN="${DOMAIN:-0}"
fi
AUTH="${XAUTHORITY:-}"
if [[ -z "$AUTH" || ! -r "$AUTH" ]]; then
  AUTH="$(find "/run/user/$(id -u)" -maxdepth 1 -name '.mutter-Xwaylandauth.*' -print -quit 2>/dev/null || true)"
fi
if [[ -z "$AUTH" || ! -r "$AUTH" ]]; then AUTH="$HOME/.Xauthority"; fi
[[ -r "$AUTH" ]] || {
  echo 'Cannot read the desktop Xauthority. Run from the workstation desktop session.' >&2; exit 1;
}
args=(--rm --network host --ipc host
  -e "DISPLAY=$DISP" -e XAUTHORITY=/tmp/femto-xauth
  -e "ROS_DOMAIN_ID=$DOMAIN" -e ROS_LOCALHOST_ONLY=0
  -e QT_X11_NO_MITSHM=1 -e QT_QPA_PLATFORM=xcb
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro
  -v "$AUTH:/tmp/femto-xauth:ro"
  -v "$CONFIG:/opt/femto/config.rviz:ro")
if [[ -d /dev/dri ]]; then args+=(--device /dev/dri:/dev/dri); fi
echo "Opening RViz on $DISP (ROS_DOMAIN_ID=$DOMAIN). Close the window to stop the viewer."
"${D[@]}" run "${args[@]}" orbbec-femto-bolt:sorting-local rviz2 -d /opt/femto/config.rviz "$@"

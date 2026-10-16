#!/usr/bin/env bash
# 8003 异设备调用 8001 顺丰分拣服务
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sudo docker run --rm --network host -e ROS_DOMAIN_ID=0 \
  -v "$SCRIPT_DIR/test_sorting_client.py:/tmp/test_sorting_client.py" \
  orbbec-femto-bolt:humble /opt/femto/entrypoint.sh python3 /tmp/test_sorting_client.py "$@"

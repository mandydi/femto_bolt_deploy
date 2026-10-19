#!/usr/bin/env bash
# ==============================================================================
# 顺丰分拣服务调用快捷脚本 (向后兼容入口)
# 
# 实际工程生产调用推荐统一使用:
#   bash deploy.sh server client
# 
# 本脚本作为旧工程习惯的兼容代理，自动透传所有参数至 deploy.sh server client
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$SCRIPT_DIR/deploy.sh" server client "$@"

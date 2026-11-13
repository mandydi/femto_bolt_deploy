#!/usr/bin/env bash
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
pkill -f "usb_scan_service.py" 2>/dev/null || true
sleep 0.5
nohup python3 "$SCRIPT_DIR/usb_scan_service.py" > "$SCRIPT_DIR/usb_service.log" 2>&1 &
echo "USB scan service started PID: $!"
sleep 1.5
curl -s http://127.0.0.1:8005/health || echo "Failed to ping health"
echo ""

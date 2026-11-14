#!/bin/bash
# Pulls flight-recorder logs from the Control Hub into ./logs/<timestamp>/ (USB-C adb).
set -e
cd "$(dirname "$0")/.."
ADB=${ADB:-adb}
$ADB devices
OUT="logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
$ADB pull /sdcard/FIRST/biobuzz_logs/. "$OUT/"
# Also grab the hub's logcat (crashes, SDK warnings) from the same run.
$ADB logcat -d -t 4000 > "$OUT/logcat.txt" 2>/dev/null || true
echo "Saved to $OUT"
ls -la "$OUT" | tail -n 15

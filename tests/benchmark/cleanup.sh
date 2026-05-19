#!/usr/bin/env bash
# Stop swarm simulation processes left after benchmark runs.

set -euo pipefail

BENCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib_benchmark.sh
source "$BENCH_DIR/lib_benchmark.sh"

bench_log "Cleaning up swarm processes..."

pkill -f "run_swarm_simulation.sh" 2>/dev/null || true
pkill -f "golang-simulator" 2>/dev/null || true
pkill -f "mavproxy.py" 2>/dev/null || true
pkill -f "go-mavproxy" 2>/dev/null || true
pkill -f "arducopter" 2>/dev/null || true
pkill -f "ArduCopter" 2>/dev/null || true

sleep 2

if pgrep -f "$PROCESS_PATTERN" >/dev/null 2>&1; then
  bench_log "Force killing remaining processes..."
  pkill -9 -f "arducopter" 2>/dev/null || true
  pkill -9 -f "ArduCopter" 2>/dev/null || true
  pkill -9 -f "mavproxy.py" 2>/dev/null || true
  pkill -9 -f "go-mavproxy" 2>/dev/null || true
  pkill -9 -f "golang-simulator" 2>/dev/null || true
  sleep 1
fi

rm -f "$SWARM_SIM_ROOT/config_qemu_temp.json" 2>/dev/null || true
rm -f "$SWARM_SIM_ROOT/logs/go_mavproxy_sessions.json" 2>/dev/null || true

if pgrep -f "$PROCESS_PATTERN" >/dev/null 2>&1; then
  bench_log "WARNING: processes still running:"
  pgrep -af "$PROCESS_PATTERN" || true
  exit 1
fi

bench_log "Cleanup complete."
exit 0

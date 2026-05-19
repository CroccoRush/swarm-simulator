#!/usr/bin/env bash
# Run one benchmark case: launcher + resource monitor + readiness detection.

set -euo pipefail

BENCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib_benchmark.sh
source "$BENCH_DIR/lib_benchmark.sh"

DRONES=""
MODE=""
TRANSPORT="default"
CONFIG_FILE_OVERRIDE=""
OUT_DIR=""
RUN_ID=""
DRY_RUN=0
SCENARIO="${SCENARIO:-scenarios/simple_flight.yaml}"

usage() {
  cat <<EOF
Usage: $0 --drones N --mode MODE [options]

Modes: python_per_instance, go_per_instance, go_single_process

Options:
  --config PATH       Override config JSON
  --transport TYPE    default | tcp | unix (for generated transport configs)
  --out-dir PATH      Output directory (default: results/<run_id>/<case_name>)
  --run-id ID         Benchmark run id (default: timestamp)
  --dry-run           Print commands only
  -h, --help
EOF
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --drones) DRONES=$2; shift 2 ;;
    --mode) MODE=$2; shift 2 ;;
    --config) CONFIG_FILE_OVERRIDE=$2; shift 2 ;;
    --transport) TRANSPORT=$2; shift 2 ;;
    --out-dir) OUT_DIR=$2; shift 2 ;;
    --run-id) RUN_ID=$2; shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h | --help) usage ;;
    *) echo "Unknown option: $1" >&2; usage ;;
  esac
done

[[ -n "$DRONES" && -n "$MODE" ]] || usage

RUN_ID="${RUN_ID:-$BENCHMARK_RUN_ID}"
CASE_NAME=$(bench_case_name "$DRONES" "$MODE" "$TRANSPORT")
if [[ -z "$OUT_DIR" ]]; then
  OUT_DIR="$(bench_results_dir "$RUN_ID")/$CASE_NAME"
fi

if [[ -n "$CONFIG_FILE_OVERRIDE" ]]; then
  CONFIG_PATH="$CONFIG_FILE_OVERRIDE"
elif [[ "$TRANSPORT" == "unix" || "$TRANSPORT" == "tcp" ]]; then
  CONFIG_PATH=$(bench_config_for_drones "$DRONES" "$TRANSPORT")
  if [[ ! -f "$CONFIG_PATH" ]]; then
    bench_generate_transport_configs "$DRONES"
  fi
else
  CONFIG_PATH=$(bench_ensure_drone_config "$DRONES")
fi

if [[ ! -f "$CONFIG_PATH" ]]; then
  echo "ERROR: config not found: $CONFIG_PATH" >&2
  exit 1
fi

bench_apply_mode "$MODE"
export CONFIG_FILE="$CONFIG_PATH"

mkdir -p "$OUT_DIR"
STOP_FILE="$OUT_DIR/.stop_monitor"
T0_FILE="$OUT_DIR/t0_ms"
READINESS_FILE="$OUT_DIR/readiness_ms"
rm -f "$STOP_FILE" "$READINESS_FILE"

cat >"$OUT_DIR/case_meta.env" <<EOF
run_id=$RUN_ID
case_name=$CASE_NAME
num_drones=$DRONES
mode=$MODE
mavproxy_impl=$MAVPROXY_IMPL
mavproxy_topology=$MAVPROXY_TOPOLOGY
transport=$TRANSPORT
config_file=$CONFIG_PATH
scenario=$SCENARIO
nic_iface=$(bench_detect_nic)
EOF

bench_log "Case: $CASE_NAME config=$CONFIG_PATH mode=$MAVPROXY_IMPL/$MAVPROXY_TOPOLOGY"
bench_log "Output: $OUT_DIR"

if [[ "$DRY_RUN" -eq 1 ]]; then
  echo "Would run: CONFIG_FILE=$CONFIG_PATH MAVPROXY_IMPL=$MAVPROXY_IMPL MAVPROXY_TOPOLOGY=$MAVPROXY_TOPOLOGY"
  echo "  $SWARM_SIM_ROOT/run_swarm_simulation.sh experiment direct $SCENARIO --headless"
  exit 0
fi

"$BENCH_DIR/cleanup.sh" || true
sleep 2

T0_MS=$(bench_ms_now)
echo "$T0_MS" >"$T0_FILE"

"$BENCH_DIR/monitor_resources.sh" \
  --out-dir "$OUT_DIR" \
  --stop-file "$STOP_FILE" \
  --t0-file "$T0_FILE" \
  --readiness-file "$READINESS_FILE" \
  --interval "$BENCHMARK_INTERVAL_SEC" &
MONITOR_PID=$!

cd "$SWARM_SIM_ROOT"
# shellcheck disable=SC2064
trap '"$BENCH_DIR/cleanup.sh" || true; kill $MONITOR_PID 2>/dev/null || true' EXIT

./run_swarm_simulation.sh experiment direct "$SCENARIO" --headless \
  >"$OUT_DIR/run.log" 2>&1 &
LAUNCHER_PID=$!

readiness_ms=-1
result="timeout"
if readiness_ms=$(bench_wait_for_readiness "$OUT_DIR/run.log" "$READINESS_TIMEOUT_SEC" "$T0_MS"); then
  echo "$readiness_ms" >"$READINESS_FILE"
  result="success"
  bench_log "Readiness in ${readiness_ms} ms"
  sleep "$POST_READY_SECONDS"
else
  echo "-1" >"$READINESS_FILE"
  bench_log "Readiness timeout (${READINESS_TIMEOUT_SEC}s)"
fi

kill "$LAUNCHER_PID" 2>/dev/null || true
sleep 1
"$BENCH_DIR/cleanup.sh" || true

touch "$STOP_FILE"
wait "$MONITOR_PID" 2>/dev/null || true

# Build summary_line.json
success_flag="false"
[[ "$result" == "success" ]] && success_flag="true"

# shellcheck disable=SC1090
[[ -f "$OUT_DIR/monitor_summary.env" ]] && source "$OUT_DIR/monitor_summary.env"

export BENCH_SUMMARY_OUT_DIR="$OUT_DIR"
export BENCH_READINESS_MS="$readiness_ms"
export BENCH_RESULT="$result"
export BENCH_SUCCESS="$success_flag"
python3 "$BENCH_DIR/write_summary.py"

bench_log "Done: $CASE_NAME result=$result readiness_ms=$readiness_ms"
[[ "$result" == "success" ]] && exit 0 || exit 1

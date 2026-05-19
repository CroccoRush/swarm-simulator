#!/usr/bin/env bash
# Full scenario runs (simple_flight) for 4/8/16/32 drones × MAVProxy modes.
# Archives logs/ per case and builds drone_trajectories.png via app/proff.py.

set -euo pipefail

BENCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib_benchmark.sh
source "$BENCH_DIR/lib_benchmark.sh"

DRONES_LIST="${DRONES_LIST:-4,8,16,32}"
MODES_LIST="${MODES_LIST:-python_per_instance,go_per_instance,go_single_process}"
SCENARIO="${SCENARIO:-scenarios/simple_flight.yaml}"
RUN_ID="${BENCHMARK_RUN_ID:-trajectory_$(date +%Y%m%d_%H%M%S)}"
RUN_TIMEOUT_SEC="${RUN_TIMEOUT_SEC:-900}"
DRY_RUN=0

TRAJECTORY_ROOT="$BENCH_DIR/trajectory_runs/$RUN_ID"

usage() {
  cat <<EOF
Usage: $0 [options]

Runs full experiment (until simulator exits), copies logs/, generates drone_trajectories.png.

Options:
  --drones LIST       default: 4,8,16,32
  --modes LIST        default: all three MAVProxy modes
  --run-id ID
  --timeout SEC       per-case wall timeout (default: 900)
  --dry-run
  -h, --help
EOF
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --drones) DRONES_LIST=$2; shift 2 ;;
    --modes) MODES_LIST=$2; shift 2 ;;
    --run-id) RUN_ID=$2; shift 2 ;;
    --timeout) RUN_TIMEOUT_SEC=$2; shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h | --help) usage ;;
    *) echo "Unknown: $1" >&2; usage ;;
  esac
done

bench_clear_workspace_logs() {
  mkdir -p "$SWARM_SIM_ROOT/logs"
  find "$SWARM_SIM_ROOT/logs" -maxdepth 1 -type f \( \
    -name 'drone_*' -o -name 'mavproxy_*' -o -name 'qemu_*' \
    -o -name '*.BIN' -o -name '*.tlog' -o -name '*.tlog.raw' \
    -o -name 'mav.*' -o -name 'eeprom.bin' \
  \) -delete 2>/dev/null || true
  rm -f "$SWARM_SIM_ROOT/logs/go_mavproxy_sessions.json" 2>/dev/null || true
}

bench_archive_logs() {
  local dest=$1
  mkdir -p "$dest"
  find "$SWARM_SIM_ROOT/logs" -maxdepth 1 -type f \
    \( -name 'drone_*' -o -name 'mavproxy_*' -o -name 'qemu_*' \
       -o -name '*.BIN' -o -name '*.tlog' -o -name '*.tlog.raw' -o -name 'mav.*' \) \
    -exec cp -a {} "$dest/" \;
  if [[ -f "$SWARM_SIM_ROOT/logs/go_mavproxy_sessions.json" ]]; then
    cp -a "$SWARM_SIM_ROOT/logs/go_mavproxy_sessions.json" "$dest/"
  fi
}

bench_run_proff_paths() {
  local out_dir=$1
  local n=$2
  local logs_dir="$out_dir/logs"
  if ! ls "$logs_dir"/drone_*_position*.csv >/dev/null 2>&1; then
    bench_log "WARN: no position CSV in $logs_dir — skip proff"
    return 1
  fi
  (
    cd "$SWARM_SIM_ROOT/app"
    python3 proff.py --mode paths \
      --logs-dir "$logs_dir" \
      --output-dir "$out_dir" \
      --drone-count "$n" \
      --target-exp 1
  )
}

mkdir -p "$TRAJECTORY_ROOT"
bench_log "Trajectory matrix run_id=$RUN_ID root=$TRAJECTORY_ROOT"

IFS=',' read -ra DRONE_COUNTS <<<"$DRONES_LIST"
IFS=',' read -ra MODES <<<"$MODES_LIST"

FAILURES=0

for n in "${DRONE_COUNTS[@]}"; do
  n="${n// /}"
  [[ -z "$n" ]] && continue
  cfg=$(bench_ensure_drone_config "$n")
  for mode in "${MODES[@]}"; do
    mode="${mode// /}"
    [[ -z "$mode" ]] && continue
    case_name=$(bench_case_name "$n" "$mode")
    out_dir="$TRAJECTORY_ROOT/$case_name"
    mkdir -p "$out_dir"

    bench_apply_mode "$mode"
    export CONFIG_FILE="$cfg"

    cat >"$out_dir/case_meta.env" <<EOF
run_id=$RUN_ID
case_name=$case_name
num_drones=$n
mode=$mode
mavproxy_impl=$MAVPROXY_IMPL
mavproxy_topology=$MAVPROXY_TOPOLOGY
config_file=$cfg
scenario=$SCENARIO
EOF

    bench_log "=== $case_name (full scenario) ==="

    if [[ "$DRY_RUN" -eq 1 ]]; then
      echo "Would run -> $out_dir"
      continue
    fi

    "$BENCH_DIR/cleanup.sh" || true
    bench_clear_workspace_logs
    sleep 2

    cd "$SWARM_SIM_ROOT"
    set +e
    timeout "$RUN_TIMEOUT_SEC" \
      ./run_swarm_simulation.sh experiment direct "$SCENARIO" --headless \
      >"$out_dir/run.log" 2>&1
    sim_exit=$?
    set -e

    echo "simulator_exit=$sim_exit" >>"$out_dir/case_meta.env"
    bench_archive_logs "$out_dir/logs"

    if [[ "$sim_exit" -eq 124 ]]; then
      bench_log "TIMEOUT after ${RUN_TIMEOUT_SEC}s for $case_name"
      FAILURES=$((FAILURES + 1))
    elif [[ "$sim_exit" -ne 0 ]]; then
      bench_log "Simulator exited with code $sim_exit for $case_name"
      FAILURES=$((FAILURES + 1))
    fi

    if ! bench_run_proff_paths "$out_dir" "$n"; then
      bench_log "proff paths failed for $case_name"
      FAILURES=$((FAILURES + 1))
    elif [[ -f "$out_dir/drone_trajectories.png" ]]; then
      bench_log "Saved $out_dir/drone_trajectories.png"
    fi

    "$BENCH_DIR/cleanup.sh" || true
    sleep 5
  done
done

bench_log "Trajectory matrix done. failures=$FAILURES root=$TRAJECTORY_ROOT"
[[ "$FAILURES" -eq 0 ]] && exit 0 || exit 1

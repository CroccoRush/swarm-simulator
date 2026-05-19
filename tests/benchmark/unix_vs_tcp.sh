#!/usr/bin/env bash
# Compare Serial5 TCP vs Unix socket transport (fixed go single-process MAVProxy).

set -euo pipefail

BENCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib_benchmark.sh
source "$BENCH_DIR/lib_benchmark.sh"

DRONES_LIST="${DRONES_LIST:-4,8,16}"
INCLUDE_32=0
RUN_ID=""
DRY_RUN=0
SKIP_PLOT=0
FAILURES=0
MAVPROXY_MODE="${MAVPROXY_MODE:-go_single_process}"

usage() {
  cat <<EOF
Usage: $0 [options]

Compare serial5 tcp vs unix using generated configs.
Default MAVProxy: go_single_process

Options:
  --drones LIST     (default: 4,8,16)
  --include-32
  --run-id ID
  --mode MODE       MAVProxy mode (default: go_single_process)
  --dry-run
  --skip-plot
  -h, --help
EOF
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --drones) DRONES_LIST=$2; shift 2 ;;
    --include-32) INCLUDE_32=1; shift ;;
    --run-id) RUN_ID=$2; shift 2 ;;
    --mode) MAVPROXY_MODE=$2; shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    --skip-plot) SKIP_PLOT=1; shift ;;
    -h | --help) usage ;;
    *) echo "Unknown: $1" >&2; usage ;;
  esac
done

if [[ "$INCLUDE_32" -eq 1 ]]; then
  DRONES_LIST="${DRONES_LIST},32"
fi

RUN_ID="${RUN_ID:-${BENCHMARK_RUN_ID}_unix_tcp}"
export BENCHMARK_RUN_ID="$RUN_ID"
RESULTS_ROOT="$(bench_results_dir "$RUN_ID")"
mkdir -p "$RESULTS_ROOT"

bench_log "Unix vs TCP run_id=$RUN_ID drones=$DRONES_LIST"

if [[ "$DRY_RUN" -eq 0 ]]; then
  bench_generate_transport_configs "$DRONES_LIST"
fi

IFS=',' read -ra DRONE_COUNTS <<<"$DRONES_LIST"
for n in "${DRONE_COUNTS[@]}"; do
  n="${n// /}"
  [[ -z "$n" ]] && continue
  for transport in tcp unix; do
    tcp_cfg="$BENCH_DIR/generated/config_${n}_tcp.json"
    unix_cfg="$BENCH_DIR/generated/config_${n}_unix.json"
    if [[ ! -f "$tcp_cfg" || ! -f "$unix_cfg" ]]; then
      bench_log "SKIP: missing generated config for $n"
      continue
    fi
    if [[ "$transport" == "unix" && "${SKIP_UNIX:-0}" == "1" ]]; then
      bench_log "SKIP unix (SKIP_UNIX=1)"
      continue
    fi
    bench_log "=== ${n} drones / serial5=$transport / $MAVPROXY_MODE ==="
    if [[ "$DRY_RUN" -eq 1 ]]; then
      "$BENCH_DIR/run_single_case.sh" \
        --drones "$n" --mode "$MAVPROXY_MODE" --transport "$transport" \
        --run-id "$RUN_ID" --dry-run
      continue
    fi
    if ! "$BENCH_DIR/run_single_case.sh" \
      --drones "$n" --mode "$MAVPROXY_MODE" --transport "$transport" \
      --run-id "$RUN_ID"; then
      if [[ "$transport" == "unix" ]]; then
        bench_log "Unix run failed — ensure ardupilot-swarming SITL with UDS support (SITL_BIN)"
      fi
      FAILURES=$((FAILURES + 1))
    fi
    sleep 5
  done
done

if [[ "$DRY_RUN" -eq 0 ]]; then
  python3 "$BENCH_DIR/parse_results.py" "$RESULTS_ROOT" || true
  if [[ "$SKIP_PLOT" -eq 0 ]] && python3 -c "import matplotlib" 2>/dev/null; then
    python3 "$BENCH_DIR/plot_results.py" "$RESULTS_ROOT" || true
  fi
fi

bench_log "Unix vs TCP complete. failures=$FAILURES"
[[ "$FAILURES" -eq 0 ]] && exit 0 || exit 1

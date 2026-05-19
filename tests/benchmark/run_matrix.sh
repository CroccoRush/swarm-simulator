#!/usr/bin/env bash
# Standard benchmark matrix: 4/8/16/32 drones × three MAVProxy modes.

set -euo pipefail

BENCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib_benchmark.sh
source "$BENCH_DIR/lib_benchmark.sh"

DRONES_LIST="4,8,16,32"
MODES_LIST="python_per_instance,go_per_instance,go_single_process"
RUN_ID=""
DRY_RUN=0
SKIP_PLOT=0
FAIR_PYTHON=0
FAILURES=0

usage() {
  cat <<EOF
Usage: $0 [options]

Options:
  --drones LIST       Comma-separated (default: 4,8,16,32)
  --modes LIST        Comma-separated modes (default: all three)
  --run-id ID         Results run id
  --dry-run
  --skip-plot         Do not run plot_results.py after matrix
  --fair-python       Set MAVPROXY_NO_STATE=1 for python cases
  -h, --help
EOF
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --drones) DRONES_LIST=$2; shift 2 ;;
    --modes) MODES_LIST=$2; shift 2 ;;
    --run-id) RUN_ID=$2; shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    --skip-plot) SKIP_PLOT=1; shift ;;
    --fair-python) FAIR_PYTHON=1; shift ;;
    -h | --help) usage ;;
    *) echo "Unknown: $1" >&2; usage ;;
  esac
done

RUN_ID="${RUN_ID:-$BENCHMARK_RUN_ID}"
export BENCHMARK_RUN_ID="$RUN_ID"
RESULTS_ROOT="$(bench_results_dir "$RUN_ID")"
mkdir -p "$RESULTS_ROOT"

bench_log "Matrix run_id=$RUN_ID results=$RESULTS_ROOT"

IFS=',' read -ra DRONE_COUNTS <<<"$DRONES_LIST"
IFS=',' read -ra MODES <<<"$MODES_LIST"

for n in "${DRONE_COUNTS[@]}"; do
  n="${n// /}"
  [[ -z "$n" ]] && continue
  if ! cfg=$(bench_ensure_drone_config "$n"); then
    bench_log "SKIP: cannot create config for $n drones"
    continue
  fi
  for mode in "${MODES[@]}"; do
    mode="${mode// /}"
    [[ -z "$mode" ]] && continue
    bench_log "=== ${n} drones / $mode ==="
  if [[ "$DRY_RUN" -eq 1 ]]; then
      "$BENCH_DIR/run_single_case.sh" --drones "$n" --mode "$mode" --run-id "$RUN_ID" --dry-run
      continue
    fi
    extra_env=()
    if [[ "$FAIR_PYTHON" -eq 1 && "$mode" == python_per_instance ]]; then
      extra_env=(env MAVPROXY_NO_STATE=1)
    fi
    if ! "${extra_env[@]}" "$BENCH_DIR/run_single_case.sh" --drones "$n" --mode "$mode" --run-id "$RUN_ID"; then
      FAILURES=$((FAILURES + 1))
      bench_log "FAILED: ${n}sitl_${mode}"
    fi
    sleep 5
  done
done

if [[ "$DRY_RUN" -eq 0 ]]; then
  python3 "$BENCH_DIR/parse_results.py" "$RESULTS_ROOT" || true
  if [[ "$SKIP_PLOT" -eq 0 ]]; then
    if python3 -c "import matplotlib" 2>/dev/null; then
      python3 "$BENCH_DIR/plot_results.py" "$RESULTS_ROOT" || true
    else
      bench_log "matplotlib not installed; skip plots (pip install -r tests/benchmark/requirements.txt)"
    fi
  fi
fi

bench_log "Matrix complete. failures=$FAILURES"
[[ "$FAILURES" -eq 0 ]] && exit 0 || exit 1

#!/usr/bin/env bash
# Quick smoke test: 4 drones, all MAVProxy modes, readiness check.

set -euo pipefail

BENCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../benchmark" && pwd)"
# shellcheck source=../benchmark/lib_benchmark.sh
source "$BENCH_DIR/lib_benchmark.sh"

DRONES="${SMOKE_DRONES:-4}"
RUN_ID="smoke_$(date +%Y%m%d_%H%M%S)"
FAILURES=0

bench_log "Smoke test: ${DRONES} drones, 3 MAVProxy modes"

"$BENCH_DIR/cleanup.sh" || true

for mode in python_per_instance go_per_instance go_single_process; do
  bench_log "--- smoke: $mode ---"
  if ! "$BENCH_DIR/run_single_case.sh" --drones "$DRONES" --mode "$mode" --run-id "$RUN_ID"; then
    FAILURES=$((FAILURES + 1))
  fi
  sleep 3
done

RESULTS="$(bench_results_dir "$RUN_ID")"
if [[ -d "$RESULTS" ]]; then
  python3 "$BENCH_DIR/parse_results.py" "$RESULTS" || true
fi

if [[ "$FAILURES" -eq 0 ]]; then
  bench_log "Smoke test PASSED"
  exit 0
fi

bench_log "Smoke test FAILED ($FAILURES cases)"
exit 1

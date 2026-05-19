#!/usr/bin/env bash
# Shared helpers for swarm-simulation benchmark suite.

set -euo pipefail

BENCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SWARM_SIM_ROOT="$(cd "$BENCH_DIR/../.." && pwd)"
export SWARM_SIM_ROOT

# Defaults (override via environment)
export BENCHMARK_RUN_ID="${BENCHMARK_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
export READINESS_TIMEOUT_SEC="${READINESS_TIMEOUT_SEC:-130}"
export POST_READY_SECONDS="${POST_READY_SECONDS:-30}"
export BENCHMARK_INTERVAL_SEC="${BENCHMARK_INTERVAL_SEC:-1}"
export BENCHMARK_NET_IFACE="${BENCHMARK_NET_IFACE:-}"

READINESS_MARKER='All drones are connected and have a position fix.'
PROCESS_PATTERN='arducopter|ArduCopter|mavproxy.py|go-mavproxy|golang-simulator'

bench_log() {
  echo "[benchmark] $*"
}

bench_detect_nic() {
  if [[ -n "$BENCHMARK_NET_IFACE" ]]; then
    echo "$BENCHMARK_NET_IFACE"
    return 0
  fi
  local dev
  dev=$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for (i = 1; i <= NF; i++) if ($i == "dev") { print $(i + 1); exit }}')
  if [[ -n "$dev" && -d "/sys/class/net/$dev" ]]; then
    echo "$dev"
    return 0
  fi
  for dev in eth0 ens33 enp0s3 wlan0; do
    if [[ -d "/sys/class/net/$dev" ]]; then
      echo "$dev"
      return 0
    fi
  done
  echo "eth0"
}

bench_classify_pid() {
  local pid=$1
  local cmdline comm
  if [[ ! -r "/proc/$pid/cmdline" ]]; then
    echo "other"
    return 0
  fi
  cmdline=$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true)
  comm=$(cat "/proc/$pid/comm" 2>/dev/null || true)
  if [[ "$cmdline" == *"arducopter"* ]] || [[ "$cmdline" == *"ArduCopter"* ]] || [[ "$comm" == *"arducopter"* ]]; then
    echo "sitl"
  elif [[ "$cmdline" == *"mavproxy.py"* ]] || [[ "$cmdline" == *"go-mavproxy"* ]] || [[ "$comm" == *"mavproxy"* ]]; then
    echo "proxy"
  elif [[ "$cmdline" == *"golang-simulator"* ]] || [[ "$cmdline" == *"/simulator"* && "$cmdline" != *"config-generator"* ]]; then
    echo "simulator"
  else
    echo "other"
  fi
}

bench_mode_to_env() {
  local mode=$1
  case "$mode" in
    python_per_instance)
      echo "python per-instance"
      ;;
    go_per_instance)
      echo "go per-instance"
      ;;
    go_single_process)
      echo "go single-process"
      ;;
    *)
      echo "unknown unknown" >&2
      return 1
      ;;
  esac
}

bench_apply_mode() {
  local mode=$1
  local impl topology
  read -r impl topology < <(bench_mode_to_env "$mode")
  export MAVPROXY_IMPL="$impl"
  export MAVPROXY_TOPOLOGY="$topology"
}

bench_config_for_drones() {
  local n=$1
  local transport=${2:-tcp}
  if [[ "$transport" == "unix" ]]; then
    echo "$BENCH_DIR/generated/config_${n}_unix.json"
  elif [[ "$transport" == "tcp" && -f "$BENCH_DIR/generated/config_${n}_tcp.json" ]]; then
    echo "$BENCH_DIR/generated/config_${n}_tcp.json"
  else
    echo "$SWARM_SIM_ROOT/config/config_${n}.json"
  fi
}

bench_ensure_drone_config() {
  local n=$1
  local cfg="$SWARM_SIM_ROOT/config/config_${n}.json"
  if [[ -f "$cfg" ]]; then
    echo "$cfg"
    return 0
  fi
  bench_log "Generating missing config/config_${n}.json..."
  bench_ensure_config_generator
  mkdir -p "$SWARM_SIM_ROOT/config"
  "$SWARM_SIM_ROOT/bin/config-generator" -drones "$n" -formation grid -spacing 30 \
    -range 1000 -serial5-type tcp -output "$cfg"
  echo "$cfg"
}

bench_case_name() {
  local drones=$1 mode=$2 transport=${3:-}
  if [[ -n "$transport" && "$transport" != "default" ]]; then
    echo "${drones}sitl_${mode}_${transport}"
  else
    echo "${drones}sitl_${mode}"
  fi
}

bench_results_dir() {
  local run_id=${1:-$BENCHMARK_RUN_ID}
  echo "$BENCH_DIR/results/$run_id"
}

bench_ensure_config_generator() {
  local gen="$SWARM_SIM_ROOT/bin/config-generator"
  if [[ -x "$gen" ]]; then
    return 0
  fi
  gen="$SWARM_SIM_ROOT/golang_app/build/simulator"
  if [[ -x "$SWARM_SIM_ROOT/golang_app/build/config-generator" ]]; then
    mkdir -p "$SWARM_SIM_ROOT/bin"
    cp "$SWARM_SIM_ROOT/golang_app/build/config-generator" "$SWARM_SIM_ROOT/bin/config-generator"
    return 0
  fi
  bench_log "Building config-generator..."
  make -C "$SWARM_SIM_ROOT/golang_app" config-generator
  mkdir -p "$SWARM_SIM_ROOT/bin"
  cp "$SWARM_SIM_ROOT/golang_app/build/config-generator" "$SWARM_SIM_ROOT/bin/config-generator"
}

bench_generate_transport_configs() {
  local drones_list=$1
  bench_ensure_config_generator
  mkdir -p "$BENCH_DIR/generated"
  local gen="$SWARM_SIM_ROOT/bin/config-generator"
  local n
  IFS=',' read -ra _arr <<<"$drones_list"
  for n in "${_arr[@]}"; do
    n="${n// /}"
    [[ -z "$n" ]] && continue
  bench_log "Generating configs for $n drones (tcp + unix)..."
    "$gen" -drones "$n" -formation grid -spacing 30 -range 1000 \
      -serial5-type tcp -output "$BENCH_DIR/generated/config_${n}_tcp.json"
    "$gen" -drones "$n" -formation grid -spacing 30 -range 1000 \
      -serial5-type unix -serial5-base-path /tmp/swarm_sitl \
      -output "$BENCH_DIR/generated/config_${n}_unix.json"
  done
}

bench_wait_for_readiness() {
  local log_file=$1
  local timeout_sec=$2
  local start_ms=$3
  local deadline=$((SECONDS + timeout_sec))
  while ((SECONDS < deadline)); do
    if [[ -f "$log_file" ]] && grep -qF "$READINESS_MARKER" "$log_file"; then
      local now_ms
      now_ms=$(date +%s%3N 2>/dev/null || echo $(( $(date +%s) * 1000 )))
      echo $((now_ms - start_ms))
      return 0
    fi
    sleep 0.5
  done
  echo "-1"
  return 1
}

bench_ms_now() {
  date +%s%3N 2>/dev/null || echo $(( $(date +%s) * 1000 ))
}

bench_count_drones_in_config() {
  local cfg=$1
  jq '.drones | length' "$cfg"
}

#!/usr/bin/env bash
# Sample CPU/RAM (total + per-class) and NIC/lo network counters.

set -euo pipefail

BENCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib_benchmark.sh
source "$BENCH_DIR/lib_benchmark.sh"

OUT_DIR=""
STOP_FILE=""
T0_FILE=""
READINESS_FILE=""
INTERVAL="${BENCHMARK_INTERVAL_SEC:-1}"

usage() {
  echo "Usage: $0 --out-dir DIR --stop-file FILE [--t0-file FILE] [--readiness-file FILE] [--interval SEC]"
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --out-dir) OUT_DIR=$2; shift 2 ;;
    --stop-file) STOP_FILE=$2; shift 2 ;;
    --t0-file) T0_FILE=$2; shift 2 ;;
    --readiness-file) READINESS_FILE=$2; shift 2 ;;
    --interval) INTERVAL=$2; shift 2 ;;
    -h | --help) usage ;;
    *) echo "Unknown option: $1" >&2; usage ;;
  esac
done

[[ -n "$OUT_DIR" && -n "$STOP_FILE" ]] || usage
mkdir -p "$OUT_DIR"

TS_CSV="$OUT_DIR/metrics_timeseries.csv"
PROC_CSV="$OUT_DIR/metrics_per_process.csv"
NET_CSV="$OUT_DIR/metrics_network.csv"

echo "timestamp_ms,total_rss_kb,total_cpu_pct,num_procs" >"$TS_CSV"
echo "timestamp_ms,rss_sitl_kb,rss_proxy_kb,rss_sim_kb,cpu_sitl_pct,cpu_proxy_pct,cpu_sim_pct,num_sitl,num_proxy,num_sim" >"$PROC_CSV"
echo "timestamp_ms,nic_rx_kbps,nic_tx_kbps,lo_rx_kbps,lo_tx_kbps,nic_rx_total_mb,nic_tx_total_mb,lo_rx_total_mb,lo_tx_total_mb" >"$NET_CSV"

NIC=$(bench_detect_nic)
CLK=$(getconf CLK_TCK 2>/dev/null || echo 100)

read_net() {
  local iface=$1 dir=$2
  local f="/sys/class/net/$iface/statistics/${dir}_bytes"
  if [[ -r "$f" ]]; then
    cat "$f"
  else
    echo 0
  fi
}

declare -A PREV_CPU_JIFFIES
NIC_RX0=$(read_net "$NIC" rx)
NIC_TX0=$(read_net "$NIC" tx)
LO_RX0=$(read_net lo rx)
LO_TX0=$(read_net lo tx)
NIC_RX_BASE=$NIC_RX0
NIC_TX_BASE=$NIC_TX0
LO_RX_BASE=$LO_RX0
LO_TX_BASE=$LO_TX0

T0_MS=$(cat "$T0_FILE" 2>/dev/null || bench_ms_now)

peak_total_rss=0
peak_total_cpu=0
peak_rss_sitl=0 peak_rss_proxy=0 peak_rss_sim=0
peak_cpu_sitl=0 peak_cpu_proxy=0 peak_cpu_sim=0
peak_lo_tx_kbps=0 peak_lo_rx_kbps=0
peak_nic_tx_kbps=0

sum_rss_sitl_post=0 sum_rss_proxy_post=0 sum_rss_sim_post=0
sum_cpu_sitl_post=0 sum_cpu_proxy_post=0 sum_cpu_sim_post=0
sum_lo_tx_post=0 sum_lo_rx_post=0
post_samples=0
readiness_ms=-1

while [[ ! -f "$STOP_FILE" ]]; do
  NOW_MS=$(bench_ms_now)
  REL_MS=$((NOW_MS - T0_MS))

  if [[ -f "$READINESS_FILE" && "$readiness_ms" -lt 0 ]]; then
    readiness_ms=$(cat "$READINESS_FILE" 2>/dev/null || echo -1)
  fi

  total_rss=0
  total_cpu_pct=0
  rss_sitl=0 rss_proxy=0 rss_sim=0
  cpu_sitl=0 cpu_proxy=0 cpu_sim=0
  num_sitl=0 num_proxy=0 num_sim=0
  num_procs=0

  while IFS= read -r pid; do
    [[ -z "$pid" ]] && continue
    [[ -r "/proc/$pid/status" ]] || continue
    local_rss=$(awk '/^VmRSS:/ {print $2}' "/proc/$pid/status" 2>/dev/null || echo 0)
    local_rss=${local_rss:-0}
    cls=$(bench_classify_pid "$pid")
    [[ "$cls" == "other" ]] && continue

    num_procs=$((num_procs + 1))
    total_rss=$((total_rss + local_rss))

    stat_line=$(awk '{print $14, $15}' "/proc/$pid/stat" 2>/dev/null || echo "0 0")
    utime=$(echo "$stat_line" | awk '{print $1}')
    stime=$(echo "$stat_line" | awk '{print $2}')
    jiffies=$((utime + stime))
    prev=${PREV_CPU_JIFFIES[$pid]:-}
    PREV_CPU_JIFFIES[$pid]=$jiffies
    cpu_delta=0
    if [[ -n "$prev" ]]; then
      cpu_delta=$((jiffies - prev))
    fi
    # %CPU for this process over interval (approximate)
    pcpu=$(awk -v d="$cpu_delta" -v c="$CLK" -v i="$INTERVAL" \
      'BEGIN { if (i > 0) printf "%.2f", (d / c / i) * 100; else print 0 }')
    total_cpu_pct=$(awk -v a="$total_cpu_pct" -v b="$pcpu" 'BEGIN { printf "%.2f", a + b }')

    case "$cls" in
      sitl)
        rss_sitl=$((rss_sitl + local_rss))
        cpu_sitl=$(awk -v a="$cpu_sitl" -v b="$pcpu" 'BEGIN { printf "%.2f", a + b }')
        num_sitl=$((num_sitl + 1))
        ;;
      proxy)
        rss_proxy=$((rss_proxy + local_rss))
        cpu_proxy=$(awk -v a="$cpu_proxy" -v b="$pcpu" 'BEGIN { printf "%.2f", a + b }')
        num_proxy=$((num_proxy + 1))
        ;;
      simulator)
        rss_sim=$((rss_sim + local_rss))
        cpu_sim=$(awk -v a="$cpu_sim" -v b="$pcpu" 'BEGIN { printf "%.2f", a + b }')
        num_sim=$((num_sim + 1))
        ;;
    esac
  done < <(pgrep -f "$PROCESS_PATTERN" 2>/dev/null || true)

  NIC_RX=$(read_net "$NIC" rx)
  NIC_TX=$(read_net "$NIC" tx)
  LO_RX=$(read_net lo rx)
  LO_TX=$(read_net lo tx)

  nic_rx_kbps=$(awk -v d="$((NIC_RX - NIC_RX0))" -v i="$INTERVAL" 'BEGIN { printf "%.2f", (d / 1024) / i }')
  nic_tx_kbps=$(awk -v d="$((NIC_TX - NIC_TX0))" -v i="$INTERVAL" 'BEGIN { printf "%.2f", (d / 1024) / i }')
  lo_rx_kbps=$(awk -v d="$((LO_RX - LO_RX0))" -v i="$INTERVAL" 'BEGIN { printf "%.2f", (d / 1024) / i }')
  lo_tx_kbps=$(awk -v d="$((LO_TX - LO_TX0))" -v i="$INTERVAL" 'BEGIN { printf "%.2f", (d / 1024) / i }')
  NIC_RX0=$NIC_RX
  NIC_TX0=$NIC_TX
  LO_RX0=$LO_RX
  LO_TX0=$LO_TX

  nic_rx_mb=$(awk -v b="$((NIC_RX - NIC_RX_BASE))" 'BEGIN { printf "%.4f", b / 1048576 }')
  nic_tx_mb=$(awk -v b="$((NIC_TX - NIC_TX_BASE))" 'BEGIN { printf "%.4f", b / 1048576 }')
  lo_rx_mb=$(awk -v b="$((LO_RX - LO_RX_BASE))" 'BEGIN { printf "%.4f", b / 1048576 }')
  lo_tx_mb=$(awk -v b="$((LO_TX - LO_TX_BASE))" 'BEGIN { printf "%.4f", b / 1048576 }')

  echo "$REL_MS,$total_rss,$total_cpu_pct,$num_procs" >>"$TS_CSV"
  echo "$REL_MS,$rss_sitl,$rss_proxy,$rss_sim,$cpu_sitl,$cpu_proxy,$cpu_sim,$num_sitl,$num_proxy,$num_sim" >>"$PROC_CSV"
  echo "$REL_MS,$nic_rx_kbps,$nic_tx_kbps,$lo_rx_kbps,$lo_tx_kbps,$nic_rx_mb,$nic_tx_mb,$lo_rx_mb,$lo_tx_mb" >>"$NET_CSV"

  peak_total_rss=$((total_rss > peak_total_rss ? total_rss : peak_total_rss))
  peak_total_cpu=$(awk -v a="$peak_total_cpu" -v b="$total_cpu_pct" 'BEGIN { if (b > a) print b; else print a }')
  peak_rss_sitl=$((rss_sitl > peak_rss_sitl ? rss_sitl : peak_rss_sitl))
  peak_rss_proxy=$((rss_proxy > peak_rss_proxy ? rss_proxy : peak_rss_proxy))
  peak_rss_sim=$((rss_sim > peak_rss_sim ? rss_sim : peak_rss_sim))
  peak_cpu_sitl=$(awk -v a="$peak_cpu_sitl" -v b="$cpu_sitl" 'BEGIN { if (b > a) print b; else print a }')
  peak_cpu_proxy=$(awk -v a="$peak_cpu_proxy" -v b="$cpu_proxy" 'BEGIN { if (b > a) print b; else print a }')
  peak_cpu_sim=$(awk -v a="$peak_cpu_sim" -v b="$cpu_sim" 'BEGIN { if (b > a) print b; else print a }')
  peak_lo_tx_kbps=$(awk -v a="$peak_lo_tx_kbps" -v b="$lo_tx_kbps" 'BEGIN { if (b > a) print b; else print a }')
  peak_lo_rx_kbps=$(awk -v a="$peak_lo_rx_kbps" -v b="$lo_rx_kbps" 'BEGIN { if (b > a) print b; else print a }')
  peak_nic_tx_kbps=$(awk -v a="$peak_nic_tx_kbps" -v b="$nic_tx_kbps" 'BEGIN { if (b > a) print b; else print a }')

  post_ready=0
  if [[ "$readiness_ms" -ge 0 && "$REL_MS" -ge "$readiness_ms" ]]; then
    post_ready=1
  fi
  if [[ "$post_ready" -eq 1 ]]; then
    sum_rss_sitl_post=$((sum_rss_sitl_post + rss_sitl))
    sum_rss_proxy_post=$((sum_rss_proxy_post + rss_proxy))
    sum_rss_sim_post=$((sum_rss_sim_post + rss_sim))
    sum_cpu_sitl_post=$(awk -v a="$sum_cpu_sitl_post" -v b="$cpu_sitl" 'BEGIN { printf "%.2f", a + b }')
    sum_cpu_proxy_post=$(awk -v a="$sum_cpu_proxy_post" -v b="$cpu_proxy" 'BEGIN { printf "%.2f", a + b }')
    sum_cpu_sim_post=$(awk -v a="$sum_cpu_sim_post" -v b="$cpu_sim" 'BEGIN { printf "%.2f", a + b }')
    sum_lo_tx_post=$(awk -v a="$sum_lo_tx_post" -v b="$lo_tx_kbps" 'BEGIN { printf "%.2f", a + b }')
    sum_lo_rx_post=$(awk -v a="$sum_lo_rx_post" -v b="$lo_rx_kbps" 'BEGIN { printf "%.2f", a + b }')
    post_samples=$((post_samples + 1))
  fi

  sleep "$INTERVAL"
done

total_nic_rx_mb=$(awk -v b="$(( $(read_net "$NIC" rx) - NIC_RX_BASE ))" 'BEGIN { printf "%.4f", b / 1048576 }')
total_nic_tx_mb=$(awk -v b="$(( $(read_net "$NIC" tx) - NIC_TX_BASE ))" 'BEGIN { printf "%.4f", b / 1048576 }')
total_lo_rx_mb=$(awk -v b="$(( $(read_net lo rx) - LO_RX_BASE ))" 'BEGIN { printf "%.4f", b / 1048576 }')
total_lo_tx_mb=$(awk -v b="$(( $(read_net lo tx) - LO_TX_BASE ))" 'BEGIN { printf "%.4f", b / 1048576 }')

avg_rss_sitl=0 avg_rss_proxy=0 avg_rss_sim=0
avg_cpu_sitl=0 avg_cpu_proxy=0 avg_cpu_sim=0
avg_lo_tx_kbps=0
if [[ "$post_samples" -gt 0 ]]; then
  avg_rss_sitl=$((sum_rss_sitl_post / post_samples))
  avg_rss_proxy=$((sum_rss_proxy_post / post_samples))
  avg_rss_sim=$((sum_rss_sim_post / post_samples))
  avg_cpu_sitl=$(awk -v s="$sum_cpu_sitl_post" -v n="$post_samples" 'BEGIN { printf "%.2f", s / n }')
  avg_cpu_proxy=$(awk -v s="$sum_cpu_proxy_post" -v n="$post_samples" 'BEGIN { printf "%.2f", s / n }')
  avg_cpu_sim=$(awk -v s="$sum_cpu_sim_post" -v n="$post_samples" 'BEGIN { printf "%.2f", s / n }')
  avg_lo_tx_kbps=$(awk -v s="$sum_lo_tx_post" -v n="$post_samples" 'BEGIN { printf "%.2f", s / n }')
fi

cat >"$OUT_DIR/monitor_summary.env" <<EOF
nic_iface=$NIC
peak_rss_kb=$peak_total_rss
peak_cpu_pct=$peak_total_cpu
peak_rss_sitl_kb=$peak_rss_sitl
peak_rss_proxy_kb=$peak_rss_proxy
peak_rss_sim_kb=$peak_rss_sim
peak_cpu_sitl_pct=$peak_cpu_sitl
peak_cpu_proxy_pct=$peak_cpu_proxy
peak_cpu_sim_pct=$peak_cpu_sim
total_nic_rx_mb=$total_nic_rx_mb
total_nic_tx_mb=$total_nic_tx_mb
total_lo_rx_mb=$total_lo_rx_mb
total_lo_tx_mb=$total_lo_tx_mb
peak_lo_tx_kbps=$peak_lo_tx_kbps
peak_lo_rx_kbps=$peak_lo_rx_kbps
peak_nic_tx_kbps=$peak_nic_tx_kbps
avg_rss_sitl_kb=$avg_rss_sitl
avg_rss_proxy_kb=$avg_rss_proxy
avg_rss_sim_kb=$avg_rss_sim
avg_cpu_sitl_pct=$avg_cpu_sitl
avg_cpu_proxy_pct=$avg_cpu_proxy
avg_cpu_sim_pct=$avg_cpu_sim
avg_lo_tx_kbps_post_ready=$avg_lo_tx_kbps
post_ready_samples=$post_samples
EOF

bench_log "Monitor stopped. NIC=$NIC peaks: RSS=${peak_total_rss}KB CPU=${peak_total_cpu}% lo_tx=${total_lo_tx_mb}MB"
exit 0

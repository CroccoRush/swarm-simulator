#!/usr/bin/env python3
"""Merge case_meta.env + monitor_summary.env into summary_line.json."""

from __future__ import annotations

import json
import os
from pathlib import Path


def _load_env(path: Path) -> dict[str, str]:
    data: dict[str, str] = {}
    if not path.is_file():
        return data
    for line in path.read_text().splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            data[k.strip()] = v.strip()
    return data


def _num(s: str | None, typ: type, default: int | float = 0):
    if s is None or s == "":
        return default
    try:
        return typ(s)
    except (TypeError, ValueError):
        return default


def main() -> None:
    out_dir = Path(os.environ["BENCH_SUMMARY_OUT_DIR"])
    meta = _load_env(out_dir / "case_meta.env")
    mon = _load_env(out_dir / "monitor_summary.env")

    summary = {
        "run_id": meta.get("run_id"),
        "case_name": meta.get("case_name"),
        "num_drones": int(meta.get("num_drones", 0) or 0),
        "mode": meta.get("mode"),
        "mavproxy_impl": meta.get("mavproxy_impl"),
        "mavproxy_topology": meta.get("mavproxy_topology"),
        "transport": meta.get("transport"),
        "config_file": meta.get("config_file"),
        "nic_iface": meta.get("nic_iface") or mon.get("nic_iface"),
        "readiness_ms": int(os.environ.get("BENCH_READINESS_MS", -1)),
        "result": os.environ.get("BENCH_RESULT", "unknown"),
        "success": os.environ.get("BENCH_SUCCESS", "false") == "true",
        "peak_rss_kb": _num(mon.get("peak_rss_kb"), int),
        "peak_cpu_pct": _num(mon.get("peak_cpu_pct"), float),
        "peak_rss_sitl_kb": _num(mon.get("peak_rss_sitl_kb"), int),
        "peak_rss_proxy_kb": _num(mon.get("peak_rss_proxy_kb"), int),
        "peak_rss_sim_kb": _num(mon.get("peak_rss_sim_kb"), int),
        "peak_cpu_sitl_pct": _num(mon.get("peak_cpu_sitl_pct"), float),
        "peak_cpu_proxy_pct": _num(mon.get("peak_cpu_proxy_pct"), float),
        "peak_cpu_sim_pct": _num(mon.get("peak_cpu_sim_pct"), float),
        "total_nic_rx_mb": _num(mon.get("total_nic_rx_mb"), float),
        "total_nic_tx_mb": _num(mon.get("total_nic_tx_mb"), float),
        "total_lo_rx_mb": _num(mon.get("total_lo_rx_mb"), float),
        "total_lo_tx_mb": _num(mon.get("total_lo_tx_mb"), float),
        "peak_lo_tx_kbps": _num(mon.get("peak_lo_tx_kbps"), float),
        "peak_lo_rx_kbps": _num(mon.get("peak_lo_rx_kbps"), float),
        "avg_lo_tx_kbps_post_ready": _num(mon.get("avg_lo_tx_kbps_post_ready"), float),
        "avg_rss_sitl_kb": _num(mon.get("avg_rss_sitl_kb"), int),
        "avg_rss_proxy_kb": _num(mon.get("avg_rss_proxy_kb"), int),
        "avg_rss_sim_kb": _num(mon.get("avg_rss_sim_kb"), int),
    }
    (out_dir / "summary_line.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()

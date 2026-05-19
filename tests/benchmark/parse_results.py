#!/usr/bin/env python3
"""Aggregate summary_line.json files into summary.csv and SCALING_SUMMARY.md."""

from __future__ import annotations

import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

FIELDS = [
    "run_id",
    "case_name",
    "num_drones",
    "mode",
    "mavproxy_impl",
    "mavproxy_topology",
    "transport",
    "readiness_ms",
    "result",
    "success",
    "peak_rss_kb",
    "peak_cpu_pct",
    "peak_rss_sitl_kb",
    "peak_rss_proxy_kb",
    "peak_rss_sim_kb",
    "peak_cpu_sitl_pct",
    "peak_cpu_proxy_pct",
    "peak_cpu_sim_pct",
    "total_nic_rx_mb",
    "total_nic_tx_mb",
    "total_lo_rx_mb",
    "total_lo_tx_mb",
    "peak_lo_tx_kbps",
    "avg_lo_tx_kbps_post_ready",
    "nic_iface",
]


def load_summaries(root: Path) -> list[dict]:
    rows: list[dict] = []
    for path in sorted(root.rglob("summary_line.json")):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            rows.append(data)
        except (json.JSONDecodeError, OSError) as e:
            print(f"WARN: skip {path}: {e}", file=sys.stderr)
    return rows


def write_csv(root: Path, rows: list[dict]) -> Path:
    out = root / "summary.csv"
    with out.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in FIELDS})
    (root / "summary.json").write_text(
        json.dumps(rows, indent=2) + "\n", encoding="utf-8"
    )
    return out


def _fmt_ms(v) -> str:
    try:
        n = int(v)
        return f"{n} ms" if n >= 0 else "N/A"
    except (TypeError, ValueError):
        return "N/A"


def _fmt_kb(v) -> str:
    try:
        return f"{int(v)} KB"
    except (TypeError, ValueError):
        return "N/A"


def _fmt_cpu(v) -> str:
    try:
        return f"{float(v):.2f}%"
    except (TypeError, ValueError):
        return "N/A"


def _fmt_mb(v) -> str:
    try:
        return f"{float(v):.4f} MB"
    except (TypeError, ValueError):
        return "N/A"


def write_scaling_md(root: Path, rows: list[dict]) -> Path:
    """Write markdown tables grouped by num_drones (matrix) or transport (unix vs tcp)."""
    out = root / "SCALING_SUMMARY.md"
    lines = [
        "# Scaling Summary",
        "",
        f"Run: `{root.name}`",
        "",
        "Infrastructure readiness: log line "
        "`All drones are connected and have a position fix.`",
        "",
    ]

    by_drones: dict[int, list[dict]] = defaultdict(list)
    for r in rows:
        n = r.get("num_drones")
        if n is not None:
            by_drones[int(n)].append(r)

    if by_drones:
        for n in sorted(by_drones):
            lines.append(f"## {n} SITL")
            lines.append("")
            lines.append(
                "| Mode | Readiness | Peak RSS | Peak CPU | "
                "RSS sitl/proxy/sim | lo TX | NIC TX | Result |"
            )
            lines.append("| --- | ---: | ---: | ---: | --- | ---: | ---: | --- |")
            for r in sorted(by_drones[n], key=lambda x: x.get("mode", "")):
                mode = r.get("mode", r.get("case_name", "?"))
                rss_br = (
                    f"{r.get('peak_rss_sitl_kb', 0)}/"
                    f"{r.get('peak_rss_proxy_kb', 0)}/"
                    f"{r.get('peak_rss_sim_kb', 0)}"
                )
                lines.append(
                    f"| `{mode}` | {_fmt_ms(r.get('readiness_ms'))} | "
                    f"{_fmt_kb(r.get('peak_rss_kb'))} | {_fmt_cpu(r.get('peak_cpu_pct'))} | "
                    f"{rss_br} | {_fmt_mb(r.get('total_lo_tx_mb'))} | "
                    f"{_fmt_mb(r.get('total_nic_tx_mb'))} | {r.get('result', '?')} |"
                )
            lines.append("")

    transport_rows = [r for r in rows if r.get("transport") in ("tcp", "unix")]
    if transport_rows:
        lines.append("## Serial5 transport (TCP vs Unix)")
        lines.append("")
        lines.append(
            "| Drones | Transport | Mode | Readiness | Peak RSS | lo TX | Result |"
        )
        lines.append("| ---: | --- | --- | ---: | ---: | ---: | --- |")
        for r in sorted(
            transport_rows,
            key=lambda x: (x.get("num_drones", 0), x.get("transport", "")),
        ):
            lines.append(
                f"| {r.get('num_drones')} | {r.get('transport')} | "
                f"`{r.get('mode', '?')}` | {_fmt_ms(r.get('readiness_ms'))} | "
                f"{_fmt_kb(r.get('peak_rss_kb'))} | "
                f"{_fmt_mb(r.get('total_lo_tx_mb'))} | {r.get('result', '?')} |"
            )
        lines.append("")

    lines.extend(
        [
            "## Notes",
            "",
            "- **lo TX/RX**: localhost TCP/UDP (MAVLink, Serial5 tcp); primary metric for MAVProxy comparison.",
            "- **NIC TX/RX**: default-route interface; often near zero for headless localhost runs.",
            "- Per-process RSS columns: sitl / proxy / simulator peaks.",
            "",
        ]
    )

    out.write_text("\n".join(lines), encoding="utf-8")

    repo_root = root
    for _ in range(6):
        if (repo_root / "run_swarm_simulation.sh").is_file():
            break
        if repo_root.parent == repo_root:
            break
        repo_root = repo_root.parent
    if (repo_root / "run_swarm_simulation.sh").is_file():
        repo_logs = repo_root / "logs" / "benchmarks" / "SCALING_SUMMARY.md"
        repo_logs.parent.mkdir(parents=True, exist_ok=True)
        repo_logs.write_text("\n".join(lines), encoding="utf-8")
        print(f"Copied summary to {repo_logs}")

    return out


def main() -> int:
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <results_run_dir>", file=sys.stderr)
        return 1
    root = Path(sys.argv[1]).resolve()
    if not root.is_dir():
        print(f"Not a directory: {root}", file=sys.stderr)
        return 1

    rows = load_summaries(root)
    if not rows:
        print(f"No summary_line.json under {root}", file=sys.stderr)
        return 1

    csv_path = write_csv(root, rows)
    md_path = write_scaling_md(root, rows)
    print(f"Wrote {len(rows)} rows -> {csv_path}")
    print(f"Wrote {md_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

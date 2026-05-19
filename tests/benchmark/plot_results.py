#!/usr/bin/env python3
"""Generate benchmark plots from summary.csv and per-case timeseries."""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


MODES_ORDER = [
    "python_per_instance",
    "go_per_instance",
    "go_single_process",
]
MODE_LABELS = {
    "python_per_instance": "python per-inst",
    "go_per_instance": "go per-inst",
    "go_single_process": "go single-proc",
}


def _load_summary(root: Path) -> pd.DataFrame:
    csv_path = root / "summary.csv"
    if not csv_path.is_file():
        raise FileNotFoundError(f"Run parse_results.py first: missing {csv_path}")
    df = pd.read_csv(csv_path)
    df["success"] = df["success"].astype(str).str.lower() == "true"
    return df


def _matrix_df(df: pd.DataFrame) -> pd.DataFrame:
    """Rows for standard MAVProxy matrix (default transport)."""
    m = df[df["transport"].fillna("default").isin(["default", ""])].copy()
    if m.empty:
        m = df[~df["transport"].isin(["tcp", "unix"])].copy()
    return m


def plot_scaling_bars(df: pd.DataFrame, plots_dir: Path) -> None:
    metrics = [
        ("readiness_ms", "Readiness (ms)", "scaling_readiness.png"),
        ("peak_rss_kb", "Peak RSS (KB)", "scaling_peak_rss.png"),
        ("peak_cpu_pct", "Peak CPU (%)", "scaling_peak_cpu.png"),
        ("total_lo_tx_mb", "Total lo TX (MB)", "scaling_lo_tx.png"),
        ("peak_lo_tx_kbps", "Peak lo TX (KB/s)", "scaling_peak_lo_tx_kbps.png"),
    ]
    mdf = _matrix_df(df)
    if mdf.empty:
        return

    drones = sorted(mdf["num_drones"].unique())
    x = range(len(drones))
    width = 0.25

    for col, ylabel, fname in metrics:
        fig, ax = plt.subplots(figsize=(10, 5))
        for i, mode in enumerate(MODES_ORDER):
            sub = mdf[mdf["mode"] == mode]
            if sub.empty:
                continue
            vals = [
                sub[sub["num_drones"] == d][col].iloc[0]
                if len(sub[sub["num_drones"] == d])
                else 0
                for d in drones
            ]
            off = [xi + (i - 1) * width for xi in x]
            ax.bar(off, vals, width, label=MODE_LABELS.get(mode, mode))
        ax.set_xticks(list(x))
        ax.set_xticklabels([str(d) for d in drones])
        ax.set_xlabel("Drones")
        ax.set_ylabel(ylabel)
        ax.set_title(ylabel)
        ax.legend()
        ax.grid(axis="y", alpha=0.3)
        fig.tight_layout()
        fig.savefig(plots_dir / fname, dpi=120)
        plt.close(fig)


def plot_per_process_rss(df: pd.DataFrame, plots_dir: Path) -> None:
    mdf = _matrix_df(df)
    if mdf.empty:
        return
    fig, ax = plt.subplots(figsize=(10, 5))
    drones = sorted(mdf["num_drones"].unique())
    x = range(len(drones))
    width = 0.12
    classes = [
        ("peak_rss_sitl_kb", "SITL"),
        ("peak_rss_proxy_kb", "Proxy"),
        ("peak_rss_sim_kb", "Simulator"),
    ]
    for mi, mode in enumerate(MODES_ORDER):
        sub = mdf[mdf["mode"] == mode]
        if sub.empty:
            continue
        base = [xi + (mi - 1) * 0.35 for xi in x]
        bottom = [0.0] * len(drones)
        for ci, (col, label) in enumerate(classes):
            vals = [
                float(sub[sub["num_drones"] == d][col].iloc[0])
                if len(sub[sub["num_drones"] == d])
                else 0
                for d in drones
            ]
            ax.bar(
                base,
                vals,
                width * 0.9,
                bottom=bottom,
                label=f"{MODE_LABELS.get(mode, mode)} {label}",
            )
            bottom = [b + v for b, v in zip(bottom, vals)]
    ax.set_xticks(list(x))
    ax.set_xticklabels([str(d) for d in drones])
    ax.set_xlabel("Drones")
    ax.set_ylabel("Peak RSS (KB)")
    ax.set_title("Peak RSS by process class")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(plots_dir / "scaling_rss_breakdown.png", dpi=120)
    plt.close(fig)


def plot_transport_compare(df: pd.DataFrame, plots_dir: Path) -> None:
    tdf = df[df["transport"].isin(["tcp", "unix"])].copy()
    if tdf.empty:
        return
    for metric, ylabel, fname in [
        ("peak_rss_kb", "Peak RSS (KB)", "transport_peak_rss.png"),
        ("total_lo_tx_mb", "Total lo TX (MB)", "transport_lo_tx.png"),
        ("readiness_ms", "Readiness (ms)", "transport_readiness.png"),
    ]:
        fig, ax = plt.subplots(figsize=(8, 5))
        drones = sorted(tdf["num_drones"].unique())
        x = range(len(drones))
        w = 0.35
        for i, transport in enumerate(["tcp", "unix"]):
            sub = tdf[tdf["transport"] == transport]
            vals = [
                float(sub[sub["num_drones"] == d][metric].iloc[0])
                if len(sub[sub["num_drones"] == d])
                else 0
                for d in drones
            ]
            off = [xi + (i - 0.5) * w for xi in x]
            ax.bar(off, vals, w, label=transport)
        ax.set_xticks(list(x))
        ax.set_xticklabels([str(d) for d in drones])
        ax.set_xlabel("Drones")
        ax.set_ylabel(ylabel)
        ax.set_title(f"Serial5: TCP vs Unix — {ylabel}")
        ax.legend()
        ax.grid(axis="y", alpha=0.3)
        fig.tight_layout()
        fig.savefig(plots_dir / fname, dpi=120)
        plt.close(fig)


def plot_case_timeseries(case_dir: Path, plots_dir: Path, readiness_ms: float) -> None:
    ts = case_dir / "metrics_timeseries.csv"
    proc = case_dir / "metrics_per_process.csv"
    net = case_dir / "metrics_network.csv"
    if not ts.is_file():
        return
    name = case_dir.name
    tdf = pd.read_csv(ts)
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    axes[0].plot(tdf["timestamp_ms"] / 1000, tdf["total_rss_kb"], label="total RSS")
    axes[0].set_ylabel("RSS (KB)")
    axes[0].legend()
    axes[0].grid(alpha=0.3)

    axes[1].plot(tdf["timestamp_ms"] / 1000, tdf["total_cpu_pct"], label="total CPU")
    axes[1].set_ylabel("CPU (%)")
    axes[1].legend()
    axes[1].grid(alpha=0.3)

    if proc.is_file():
        pdf = pd.read_csv(proc)
        axes[0].plot(
            pdf["timestamp_ms"] / 1000,
            pdf["rss_sitl_kb"],
            label="sitl",
            alpha=0.7,
        )
        axes[0].plot(
            pdf["timestamp_ms"] / 1000,
            pdf["rss_proxy_kb"],
            label="proxy",
            alpha=0.7,
        )
        axes[0].plot(
            pdf["timestamp_ms"] / 1000,
            pdf["rss_sim_kb"],
            label="sim",
            alpha=0.7,
        )
        axes[0].legend(fontsize=8)

    if net.is_file():
        ndf = pd.read_csv(net)
        axes[2].plot(
            ndf["timestamp_ms"] / 1000, ndf["lo_tx_kbps"], label="lo TX"
        )
        axes[2].plot(
            ndf["timestamp_ms"] / 1000, ndf["lo_rx_kbps"], label="lo RX", alpha=0.7
        )
        axes[2].plot(
            ndf["timestamp_ms"] / 1000, ndf["nic_tx_kbps"], label="NIC TX", alpha=0.7
        )
        axes[2].set_ylabel("KB/s")
        axes[2].legend()
        axes[2].grid(alpha=0.3)

    if readiness_ms and readiness_ms > 0:
        for ax in axes:
            ax.axvline(readiness_ms / 1000, color="red", linestyle="--", alpha=0.6)

    axes[2].set_xlabel("Time (s)")
    fig.suptitle(f"Timeseries: {name}")
    fig.tight_layout()
    safe = name.replace("/", "_")
    fig.savefig(plots_dir / f"timeseries_{safe}.png", dpi=100)
    plt.close(fig)


def main() -> int:
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <results_run_dir>", file=sys.stderr)
        return 1
    root = Path(sys.argv[1]).resolve()
    plots_dir = root / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)

    df = _load_summary(root)
    plot_scaling_bars(df, plots_dir)
    plot_per_process_rss(df, plots_dir)
    plot_transport_compare(df, plots_dir)

    for case_dir in sorted(p for p in root.iterdir() if p.is_dir()):
        if not (case_dir / "summary_line.json").is_file():
            continue
        import json

        meta = json.loads((case_dir / "summary_line.json").read_text())
        readiness = meta.get("readiness_ms", -1)
        plot_case_timeseries(case_dir, plots_dir, readiness)

    print(f"Plots written to {plots_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

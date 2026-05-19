from pathlib import Path

import matplotlib.pyplot as plt

from .config import DRONE_COLORS, EXPERIMENT_COLORS
from .experiments import ErrorSeries, ExperimentResult


def _ensure_output_dir(output_dir: Path):
    output_dir.mkdir(parents=True, exist_ok=True)


def _set_axes(title: str, xlabel: str = "Time (s)", ylabel: str = "Formation Error (m)"):
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True, alpha=0.3)


def plot_experiment_comparison(
    experiment_results: dict[int, ExperimentResult],
    output_dir: Path,
    filename: str = "specific_experiments_comparison.png",
):
    _ensure_output_dir(output_dir)
    plt.figure(figsize=(12, 6))
    for idx, (exp_num, result) in enumerate(sorted(experiment_results.items())):
        color = EXPERIMENT_COLORS[idx % len(EXPERIMENT_COLORS)]
        for algo_name, series in result.series_by_algorithm.items():
            linestyle = "--" if "ALVP" in algo_name else "-"
            plt.plot(
                series.times_sec,
                series.errors,
                linestyle=linestyle,
                color=color,
                linewidth=1.5,
                alpha=0.8,
                label=f"{algo_name} Exp {exp_num}",
            )
    _set_axes("Formation Error Comparison")
    plt.legend(loc="upper right")
    plt.tight_layout()
    out = output_dir / filename
    plt.savefig(out)
    plt.close()
    print(f"Saved graph to {out}")


def plot_average_series(series_by_algorithm: dict[str, ErrorSeries], output_dir: Path):
    _ensure_output_dir(output_dir)
    plt.figure(figsize=(12, 6))
    for name, series in series_by_algorithm.items():
        if not series.times_sec:
            continue
        color = "b" if name == "LVP" else "r"
        plt.plot(series.times_sec, series.errors, color=color, linewidth=2, label=series.label)
    _set_axes("Average Formation Error")
    plt.legend(loc="upper right")
    plt.tight_layout()
    out = output_dir / "average_error_comparison.png"
    plt.savefig(out)
    plt.close()
    print(f"Saved graph to {out}")


def plot_single_experiment(result: ExperimentResult, output_dir: Path):
    _ensure_output_dir(output_dir)
    plt.figure(figsize=(12, 6))
    for name, series in result.series_by_algorithm.items():
        color = "blue" if name == "LVP" else "red"
        plt.plot(series.times_sec, series.errors, color=color, linewidth=1.5, label=name)
    _set_axes("Single Experiment Comparison")
    plt.legend(loc="upper right")
    plt.tight_layout()
    out = output_dir / "single_experiments_comparison.png"
    plt.savefig(out)
    plt.close()
    print(f"Saved graph to {out}")


def plot_paths(time_to_data_by_algorithm: dict[str, dict], output_dir: Path, drone_count: int):
    _ensure_output_dir(output_dir)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    axes = [ax1, ax2]
    names = list(time_to_data_by_algorithm.keys())[:2]

    for ax, name in zip(axes, names):
        ax.set_title(f"Drone Trajectories: {name}")
        data = time_to_data_by_algorithm[name]
        times = sorted(data.keys())
        trajectories = {i: {"x": [], "y": []} for i in range(drone_count)}
        for t in times:
            for i in range(drone_count):
                trajectories[i]["x"].append(data[t][i]["x"])
                trajectories[i]["y"].append(data[t][i]["y"])
        for drone_id, drone_data in trajectories.items():
            if not drone_data["x"]:
                continue
            color = DRONE_COLORS[drone_id % len(DRONE_COLORS)]
            ax.plot(drone_data["x"], drone_data["y"], color=color, linewidth=1.5, label=f"Drone {drone_id+1}")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.grid(True, alpha=0.3)
        ax.axis("equal")

    handles, labels = ax1.get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=5, bbox_to_anchor=(0.5, 0.05))
    plt.tight_layout()
    plt.subplots_adjust(bottom=0.15)
    out = output_dir / "drone_trajectories.png"
    plt.savefig(out)
    plt.close()
    print(f"Saved graph to {out}")


def plot_boxplot(errors_by_time: dict[int, list[float]], output_dir: Path, algorithm_name: str):
    _ensure_output_dir(output_dir)
    if not errors_by_time:
        print(f"No data for boxplot: {algorithm_name}")
        return
    sorted_times = sorted(errors_by_time.keys())
    start_time = sorted_times[0]
    labels = [f"{(t - start_time)/1e9:.1f}" for t in sorted_times]
    data = [errors_by_time[t] for t in sorted_times]

    plt.figure(figsize=(12, 6))
    ax = plt.subplot(111)
    box = ax.boxplot(data, patch_artist=True, tick_labels=labels)
    for patch in box["boxes"]:
        patch.set_facecolor("lightblue")
    plt.title(f"Formation Error Distribution: {algorithm_name}")
    plt.xlabel("Time from Active Phase Start (seconds)")
    plt.ylabel("Formation Error (m)")
    plt.grid(True, alpha=0.3)
    if len(labels) > 20:
        step = max(1, len(labels) // 20)
        plt.xticks(range(0, len(labels), step), labels[::step], rotation=45, ha="right")
    else:
        plt.xticks(rotation=45, ha="right")

    out = output_dir / f"{algorithm_name.lower()}_error_boxplot.png"
    plt.tight_layout()
    plt.savefig(out)
    plt.close()
    print(f"Saved graph to {out}")

import argparse
from pathlib import Path

from .config import AnalysisConfig, FormationConfig
from .experiments import (
    aggregate_average_series,
    collect_errors_by_time,
    load_paths_payload,
    load_experiment_series,
    load_single_experiment,
)
from .plotting import (
    plot_average_series,
    plot_boxplot,
    plot_experiment_comparison,
    plot_paths,
    plot_single_experiment,
)


def _build_parser():
    parser = argparse.ArgumentParser(description="Drone formation analysis tool")
    parser.add_argument(
        "--mode",
        type=str,
        choices=["specific", "single", "average", "boxplot", "paths", "errors"],
        default="errors",
        help="Analysis mode",
    )
    parser.add_argument("--experiments", type=int, nargs="+", default=[1], help="Experiment numbers")
    parser.add_argument("--target-exp", type=int, default=1, help="Experiment number for single/paths mode")
    parser.add_argument("--num-experiments", type=int, default=10, help="Count for average mode")
    parser.add_argument("--drone-count", type=int, default=4, help="Number of drones")
    parser.add_argument("--logs-dir", type=str, default="./logs", help="Directory with drone log csv files")
    parser.add_argument("--output-dir", type=str, default=".", help="Directory for output plots")
    parser.add_argument("--exp-distance", type=float, default=10.0, help="Expected formation spacing")
    return parser


def main():
    args = _build_parser().parse_args()
    cfg = AnalysisConfig(
        drone_count=args.drone_count,
        logs_dir=Path(args.logs_dir),
        output_dir=Path(args.output_dir),
        experiment_count=args.num_experiments,
    )
    formation_cfg = FormationConfig(expected_distance_m=args.exp_distance)

    if args.mode in {"specific", "errors"}:
        res = load_experiment_series(cfg, formation_cfg, args.experiments)
        plot_experiment_comparison(res, output_dir=cfg.output_dir)
    elif args.mode == "single":
        res = load_single_experiment(cfg, formation_cfg, args.target_exp)
        plot_single_experiment(res, output_dir=cfg.output_dir)
    elif args.mode == "average":
        numbers = list(range(1, args.num_experiments + 1))
        res = aggregate_average_series(cfg, formation_cfg, numbers)
        plot_average_series(res, output_dir=cfg.output_dir)
    elif args.mode == "boxplot":
        numbers = list(range(1, args.num_experiments + 1))
        res = collect_errors_by_time(cfg, formation_cfg, numbers)
        for algo_name, errors_by_time in res.items():
            plot_boxplot(errors_by_time, output_dir=cfg.output_dir, algorithm_name=algo_name)
    elif args.mode == "paths":
        res = load_paths_payload(cfg=cfg, target_exp=args.target_exp)
        plot_paths(res, output_dir=cfg.output_dir, drone_count=cfg.drone_count)


if __name__ == "__main__":
    main()

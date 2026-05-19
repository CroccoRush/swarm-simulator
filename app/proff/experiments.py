from collections import defaultdict
from dataclasses import dataclass

import numpy as np

from .config import AlgorithmConfig, AnalysisConfig, FormationConfig
from .loader import load_time_series, summarize_markers
from .metrics import normalize_to_active_phase, process_formation, smooth_data


@dataclass
class ErrorSeries:
    label: str
    times_sec: list[float]
    errors: list[float]


@dataclass
class ExperimentResult:
    series_by_algorithm: dict[str, ErrorSeries]
    markers_by_message: dict[str, list[int]]


def _suffix(param_set: int, exp_num: int) -> str:
    return ""#f"_set{param_set}_exp{exp_num}"


def load_single_experiment(
    cfg: AnalysisConfig,
    formation_cfg: FormationConfig,
    exp_num: int,
    algorithms: list[AlgorithmConfig] | None = None,
) -> ExperimentResult:
    algorithms = algorithms or cfg.algorithms
    series_by_algorithm: dict[str, ErrorSeries] = {}
    markers_agg: dict[str, list[int]] = defaultdict(list)

    for algorithm in algorithms:
        time_to_data, markers = load_time_series(
            logs_dir=cfg.logs_dir,
            drone_count=cfg.drone_count,
            suffix=_suffix(algorithm.param_set, exp_num),
            time_round_ns=cfg.time_round_ns,
        )
        by_msg = summarize_markers(markers)
        for msg, values in by_msg.items():
            markers_agg[msg].extend(values)

        times, errors, _ = process_formation(
            time_to_data, formation_cfg=formation_cfg, time_round_ns=cfg.time_round_ns
        )
        times, errors, _ = normalize_to_active_phase(
            times, errors, active_phase_fraction=cfg.active_phase_fraction
        )
        if len(errors) > cfg.smooth_window:
            errors = smooth_data(errors, cfg.smooth_window)
            times = times[: len(errors)]

        series_by_algorithm[algorithm.name] = ErrorSeries(
            label=f"{algorithm.name} Exp {exp_num}",
            times_sec=times,
            errors=errors,
        )

    return ExperimentResult(series_by_algorithm=series_by_algorithm, markers_by_message=markers_agg)


def load_experiment_series(
    cfg: AnalysisConfig,
    formation_cfg: FormationConfig,
    experiment_numbers: list[int],
    algorithms: list[AlgorithmConfig] | None = None,
) -> dict[int, ExperimentResult]:
    results: dict[int, ExperimentResult] = {}
    for exp_num in experiment_numbers:
        try:
            results[exp_num] = load_single_experiment(
                cfg=cfg, formation_cfg=formation_cfg, exp_num=exp_num, algorithms=algorithms
            )
        except Exception as error:
            print(f"Error processing experiment {exp_num}: {error}")
    return results


def aggregate_average_series(
    cfg: AnalysisConfig,
    formation_cfg: FormationConfig,
    experiment_numbers: list[int],
    algorithms: list[AlgorithmConfig] | None = None,
) -> dict[str, ErrorSeries]:
    algorithms = algorithms or cfg.algorithms
    by_algorithm_time_errors: dict[str, dict[int, list[float]]] = {
        algorithm.name: defaultdict(list) for algorithm in algorithms
    }

    for exp_num in experiment_numbers:
        for algorithm in algorithms:
            try:
                time_to_data, _ = load_time_series(
                    logs_dir=cfg.logs_dir,
                    drone_count=cfg.drone_count,
                    suffix=_suffix(algorithm.param_set, exp_num),
                    time_round_ns=cfg.time_round_ns,
                )
                _, _, errors_by_time = process_formation(
                    time_to_data,
                    formation_cfg=formation_cfg,
                    time_round_ns=cfg.time_round_ns,
                )
                for t, errs in errors_by_time.items():
                    by_algorithm_time_errors[algorithm.name][t].extend(errs)
            except Exception as error:
                print(f"Error processing {algorithm.name} exp{exp_num}: {error}")

    aggregate: dict[str, ErrorSeries] = {}
    for algorithm in algorithms:
        series_map = by_algorithm_time_errors[algorithm.name]
        if not series_map:
            aggregate[algorithm.name] = ErrorSeries(algorithm.name, [], [])
            continue
        all_times = sorted(series_map.keys())
        start_time = min(all_times) + int((max(all_times) - min(all_times)) * cfg.active_phase_fraction)
        times_sec = []
        errors = []
        for time in sorted([t for t in all_times if t >= start_time]):
            values = series_map.get(time, [])
            if values:
                times_sec.append((time - start_time) / 1e9)
                errors.append(float(np.mean(values)))
        if len(errors) > cfg.smooth_window:
            errors = smooth_data(errors, cfg.smooth_window)
            times_sec = times_sec[: len(errors)]

        aggregate[algorithm.name] = ErrorSeries(
            label=f"{algorithm.name} Average",
            times_sec=times_sec,
            errors=errors,
        )
    return aggregate


def collect_errors_by_time(
    cfg: AnalysisConfig,
    formation_cfg: FormationConfig,
    experiment_numbers: list[int],
    algorithms: list[AlgorithmConfig] | None = None,
) -> dict[str, dict[int, list[float]]]:
    algorithms = algorithms or cfg.algorithms
    result: dict[str, dict[int, list[float]]] = {
        algorithm.name: defaultdict(list) for algorithm in algorithms
    }

    for exp_num in experiment_numbers:
        for algorithm in algorithms:
            try:
                time_to_data, _ = load_time_series(
                    logs_dir=cfg.logs_dir,
                    drone_count=cfg.drone_count,
                    suffix=_suffix(algorithm.param_set, exp_num),
                    time_round_ns=cfg.time_round_ns,
                )
                _, _, errors_by_time = process_formation(
                    time_to_data,
                    formation_cfg=formation_cfg,
                    time_round_ns=cfg.time_round_ns,
                )
                for time_key, values in errors_by_time.items():
                    result[algorithm.name][time_key].extend(values)
            except Exception as error:
                print(f"Error collecting time errors for {algorithm.name} exp{exp_num}: {error}")
    return result


def load_paths_payload(
    cfg: AnalysisConfig,
    target_exp: int,
    algorithms: list[AlgorithmConfig] | None = None,
) -> dict[str, dict]:
    algorithms = algorithms or cfg.algorithms
    by_algorithm: dict[str, dict] = {}
    markers_by_algorithm: dict[str, dict[str, list[int]]] = {}

    for algorithm in algorithms:
        time_to_data, markers = load_time_series(
            logs_dir=cfg.logs_dir,
            drone_count=cfg.drone_count,
            suffix=_suffix(algorithm.param_set, target_exp),
            time_round_ns=cfg.time_round_ns,
        )
        by_algorithm[algorithm.name] = time_to_data
        markers_by_algorithm[algorithm.name] = summarize_markers(markers)

    for algo_name, marker_map in markers_by_algorithm.items():
        if marker_map:
            print(f"{algo_name} markers: {dict(marker_map)}")

    return by_algorithm

from dataclasses import dataclass, field
from pathlib import Path


EARTH_RADIUS = 6_372_795


@dataclass(frozen=True)
class FormationConfig:
    expected_distance_m: float = 10.0

    @property
    def target_positions(self) -> list[list[float]]:
        d = self.expected_distance_m
        return [
            [-d, -d],
            [-d, 0],
            [-d, d],
            [0, -d],
            [0, d],
            [d, -d],
            [d, 0],
            [d, d],
        ]


@dataclass(frozen=True)
class AlgorithmConfig:
    name: str
    param_set: int
    line_style: str = "-"


@dataclass
class AnalysisConfig:
    logs_dir: Path = Path("./logs")
    output_dir: Path = Path(".")
    drone_count: int = 2
    experiment_count: int = 10
    active_phase_fraction: float = 0.5
    time_round_ns: int = 10**8
    smooth_window: int = 5
    algorithms: list[AlgorithmConfig] = field(
        default_factory=lambda: [
            AlgorithmConfig(name="LVP", param_set=1, line_style="-"),
            AlgorithmConfig(name="ALVP", param_set=2, line_style="--"),
        ]
    )


DRONE_COLORS = [
    "blue",
    "green",
    "red",
    "orange",
    "purple",
    "brown",
    "pink",
    "gray",
    "olive",
    "cyan",
]

EXPERIMENT_COLORS = [
    "blue",
    "red",
    "green",
    "purple",
    "orange",
    "cyan",
    "magenta",
    "yellow",
    "black",
    "brown",
]

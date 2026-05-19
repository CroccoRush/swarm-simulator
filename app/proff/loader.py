from collections import defaultdict
from pathlib import Path

import numpy as np
import pandas as pd

from .config import EARTH_RADIUS


def latlon_to_meters(lat, lon, lat0, lon0):
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)
    lat0_rad = np.radians(lat0)
    lon0_rad = np.radians(lon0)

    x = EARTH_RADIUS * (lon_rad - lon0_rad) * np.cos(lat0_rad)
    y = EARTH_RADIUS * (lat_rad - lat0_rad)
    return x, y


def _read_single_log(path: Path) -> tuple[pd.DataFrame, list[tuple[int, str]]]:
    data = pd.read_csv(path, sep=",")
    if "mark" not in data.columns:
        raise ValueError(f"Log file {path} does not contain required 'mark' column")

    marker_mask = data["mark"].notna() & (data["mark"].astype(str).str.strip() != "")
    markers: list[tuple[int, str]] = []
    for _, row in data.loc[marker_mask, ["time", "mark"]].iterrows():
        markers.append((int(row["time"]), str(row["mark"]).strip()))

    data = data.loc[~marker_mask].copy()
    for col in ["time", "lat", "lon", "alt", "hdg"]:
        data[col] = pd.to_numeric(data[col], errors="coerce")
    data = data.dropna(subset=["time", "lat", "lon", "alt", "hdg"])
    data = data.loc[(data["lat"] != 0) & (data["lon"] != 0)]
    return data, markers


def combine_drone_data(data_list: list[pd.DataFrame], time_round_ns: int) -> pd.DataFrame:
    combined = data_list[0][["time", "lat", "lon"]]
    for i, data in enumerate(data_list):
        combined = pd.merge(
            combined,
            data[["time", "x", "y", "hdg", "lat", "lon"]],
            on="time",
            how="outer",
            suffixes=("", f"_{i}"),
        )
    combined.rename(
        columns={
            "x": "x_0",
            "y": "y_0",
            "hdg": "hdg_0",
            "lat": "lat_0",
            "lon": "lon_0",
        },
        inplace=True,
    )
    combined = combined.sort_values(by="time").reset_index(drop=True)
    combined["time_rounded"] = (combined["time"] // time_round_ns) * time_round_ns

    cols = []
    for i in range(len(data_list)):
        for prefix in ["x_", "y_", "hdg_", "lat_", "lon_"]:
            name = f"{prefix}{i}"
            if name in combined.columns:
                cols.append(name)

    grouped = combined.groupby("time_rounded")[cols].mean().reset_index()
    grouped["time"] = combined.groupby("time_rounded")["time"].mean().reset_index()["time"]
    grouped = grouped.sort_values(by="time").reset_index(drop=True)
    return grouped


def fill_missing_values(df: pd.DataFrame, drone_count: int) -> pd.DataFrame:
    for i in range(drone_count):
        for col in [f"x_{i}", f"y_{i}", f"lat_{i}", f"lon_{i}", f"hdg_{i}"]:
            df[col] = df[col].ffill()
    return df


def create_time_to_data(combined_data: pd.DataFrame, drone_count: int) -> dict:
    time_to_data = {}
    for _, row in combined_data.iterrows():
        drone_data = []
        for i in range(drone_count):
            drone_data.append(
                {
                    "x": row.get(f"x_{i}"),
                    "y": row.get(f"y_{i}"),
                    "hdg": row.get(f"hdg_{i}"),
                    "lat": row.get(f"lat_{i}"),
                    "lon": row.get(f"lon_{i}"),
                }
            )
        time_to_data[row["time"]] = drone_data
    return time_to_data


def load_time_series(
    logs_dir: Path, drone_count: int, suffix: str, time_round_ns: int
) -> tuple[dict, list[tuple[int, str]]]:
    data_list = []
    all_markers = []
    lat0, lon0 = None, None

    for drone_id in range(drone_count):
        path = logs_dir / f"drone_{drone_id}_position{suffix}.csv"
        one_data, markers = _read_single_log(path)
        all_markers.extend(markers)

        if lat0 is None and lon0 is None and not one_data.empty:
            lat0, lon0 = one_data.iloc[0]["lat"] * 1e-7, one_data.iloc[0]["lon"] * 1e-7
        one_data[["x", "y"]] = one_data.apply(
            lambda row: pd.Series(
                latlon_to_meters(row["lat"] * 1e-7, row["lon"] * 1e-7, lat0, lon0)
            ),
            axis=1,
        )
        one_data["hdg"] = one_data["hdg"] * 0.01
        one_data["lat"] = one_data["lat"] * 1e-7
        one_data["lon"] = one_data["lon"] * 1e-7
        data_list.append(one_data)

    combined = combine_drone_data(data_list, time_round_ns=time_round_ns)
    combined = fill_missing_values(combined, drone_count=drone_count)
    required_columns = [f"x_{i}" for i in range(drone_count)]
    combined = combined.loc[pd.notna(combined[required_columns]).all(axis=1)]

    start = combined.iloc[0]["time"]
    combined[["time"]] = combined.apply(lambda row: pd.Series(row["time"] - start), axis=1)

    normalized_markers = [(ts - int(start), msg) for ts, msg in all_markers]
    return create_time_to_data(combined, drone_count), normalized_markers


def summarize_markers(markers: list[tuple[int, str]]) -> dict[str, list[int]]:
    by_message: dict[str, list[int]] = defaultdict(list)
    for ts, msg in markers:
        by_message[msg].append(ts)
    return by_message

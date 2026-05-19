import math
from collections import defaultdict

import numpy as np
from scipy import spatial

from .config import FormationConfig


def rotate_point(x, y, angle_deg):
    angle_rad = math.radians(angle_deg)
    x_rot = x * math.cos(angle_rad) - y * math.sin(angle_rad)
    y_rot = x * math.sin(angle_rad) + y * math.cos(angle_rad)
    return x_rot, y_rot


def distance(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def smooth_data(data, window_size=5):
    if len(data) < window_size:
        return data
    smoothed = []
    for i in range(len(data)):
        start = max(0, i - window_size // 2)
        end = min(len(data), i + window_size // 2 + 1)
        smoothed.append(np.mean(data[start:end]))
    return smoothed


def rotated_coordinates(drone_data, avg_heading):
    center_x = np.mean([d["x"] for d in drone_data])
    center_y = np.mean([d["y"] for d in drone_data])
    rotated = []
    for d in drone_data:
        shifted_x = d["x"] - center_x
        shifted_y = d["y"] - center_y
        rot_x, rot_y = rotate_point(shifted_x, shifted_y, avg_heading)
        rotated.append([rot_x, rot_y])
    return rotated


def calculate_formation_error(drones_positions, formation_cfg: FormationConfig):
    kd_tree = spatial.KDTree(drones_positions)
    count = len(drones_positions)
    formation_error = 0
    max_neighbor_distance = 1.5 * formation_cfg.expected_distance_m

    for src_drone in drones_positions:
        dist, indx = kd_tree.query(src_drone, k=count)
        neighbour_count = 0
        drone_error = 0
        for i, d in zip(indx[1:count], dist[1:count]):
            if d >= max_neighbor_distance:
                continue
            neighbour_count += 1
            target_drone = drones_positions[i]
            target_delta = [target_drone[0] - src_drone[0], target_drone[1] - src_drone[1]]
            min_dist = min([distance(target_pos, target_delta) for target_pos in formation_cfg.target_positions])
            drone_error += min_dist

        if neighbour_count != 0:
            formation_error += drone_error / neighbour_count

    if count != 0:
        formation_error = formation_error / count
    return formation_error


def process_formation(time_to_data, formation_cfg: FormationConfig, time_round_ns: int):
    times, errors, errors_by_time = [], [], defaultdict(list)
    for time, drone_data in time_to_data.items():
        rounded = (time // time_round_ns) * time_round_ns
        headings = [d["hdg"] for d in drone_data]
        avg_heading = np.mean(headings)
        rotated = rotated_coordinates(drone_data, avg_heading)
        error = calculate_formation_error(rotated, formation_cfg=formation_cfg)
        times.append(rounded)
        errors.append(error)
        errors_by_time[rounded].append(error)
    return times, errors, errors_by_time


def normalize_to_active_phase(times, errors, active_phase_fraction=0.5):
    if not times:
        return [], [], None
    sort_idx = np.argsort(times)
    times = [times[i] for i in sort_idx]
    errors = [errors[i] for i in sort_idx]
    min_time = min(times)
    max_time = max(times)
    start_time = min_time + int((max_time - min_time) * active_phase_fraction)
    filtered_times = []
    filtered_errors = []
    for t, e in zip(times, errors):
        if t >= start_time:
            filtered_times.append((t - start_time) / 1e9)
            filtered_errors.append(e)
    return filtered_times, filtered_errors, start_time

import math

import numpy as np
import pandas as pd
from scipy import spatial
import matplotlib.pyplot as plt


EXP_DISTANCE = 10
TARGET_POSITIONS = [
    [-EXP_DISTANCE, -EXP_DISTANCE], [-EXP_DISTANCE, 0], [-EXP_DISTANCE, EXP_DISTANCE],
    [0, -EXP_DISTANCE], [0, EXP_DISTANCE],
    [EXP_DISTANCE, -EXP_DISTANCE], [EXP_DISTANCE, 0], [EXP_DISTANCE, EXP_DISTANCE]
]
DRONE_COUNT = 4
EARTH_RADIUS = 6_372_795  # Радиус Земли в метрах

# Цвета для каждого дрона
DRONE_COLORS = ['red', 'blue', 'green', 'purple', 'orange', 'cyan', 'magenta', 'yellow']

def rotate_point(x, y, angle_deg):
    """
    Вращает точку (x, y) вокруг начала координат на заданный угол в градусах.
    """
    angle_rad = math.radians(angle_deg)
    x_rot = x * math.cos(angle_rad) - y * math.sin(angle_rad)
    y_rot = x * math.sin(angle_rad) + y * math.cos(angle_rad)
    return x_rot, y_rot


def latlon_to_meters(lat, lon, lat0, lon0):
    """
    Переводит широту и долготу в метры относительно опорной точки.
    :param lat: Текущая широта (в градусах).
    :param lon: Текущая долгота (в градусах).
    :param lat0: Широта опорной точки (в градусах).
    :param lon0: Долгота опорной точки (в градусах).
    :return: Кортеж (x, y) в метрах.
    """
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)

    delta_lat = lat_rad - lat0_rad
    delta_lon = lon_rad - lon0_rad

    y = EARTH_RADIUS * delta_lat
    x = EARTH_RADIUS * delta_lon * math.cos(lat0_rad)
    return x, y

def meters_to_latlon(x, y, lat0, lon0):
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)

    lat_rad = lat0_rad + y / EARTH_RADIUS
    lon_rad = lon0_rad + x / (EARTH_RADIUS * math.cos(lat0_rad))

    return math.degrees(lat_rad), math.degrees(lon_rad)

def distance(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


# Функция для объединения данных по точному времени
def combine_data(data_list):
    # Объединяем данные по времени
    combined_data = data_list[0][['time', 'lat', 'lon']]
    for i, data in enumerate(data_list):
        combined_data = pd.merge(combined_data, data[['time', 'x', 'y', 'hdg', 'lat', 'lon']],
                                 on='time', how='outer', suffixes=('', f'_{i}'))

    combined_data.rename(columns={
        'x': 'x_0',
        'y': 'y_0',
        'hdg': 'hdg_0',
        'lat': 'lat_0',
        'lon': 'lon_0'
    }, inplace=True)
    combined_data = combined_data.sort_values(by='time').reset_index(drop=True)
    return combined_data


# Функция для заполнения пропусков методом forward fill
def fill_missing_values(combined_data, num_drones):
    for i in range(num_drones):
        # Заполняем пропуски в координатах и направлении для каждого дрона
        combined_data[f'x_{i}'] = combined_data[f'x_{i}'].ffill()
        combined_data[f'y_{i}'] = combined_data[f'y_{i}'].ffill()
        combined_data[f'hdg_{i}'] = combined_data[f'hdg_{i}'].ffill()
        combined_data[f'lat_{i}'] = combined_data[f'lat_{i}'].ffill()
        combined_data[f'lon_{i}'] = combined_data[f'lon_{i}'].ffill()
    return combined_data


def create_time_to_data(combined_data, num_drones):
    time_to_data = {}
    for _, row in combined_data.iterrows():
        drone_data = []
        for i in range(num_drones):
            drone_data.append({
                'x': row.get(f'x_{i}'),
                'y': row.get(f'y_{i}'),
                'hdg': row.get(f'hdg_{i}'),
                'lat': row.get(f'lat_{i}'),
                'lon': row.get(f'lon_{i}')
            })
        time_to_data[row["time"]] = drone_data
    return time_to_data


def get_data(num_drones: int):
    data_list = list()
    lat0, lon0 = None, None  # опорная точка для вычисления расстояний
    for id in range(num_drones):
        one_data = pd.read_csv(f"./logs/drone_{id}_position.csv", sep=",")
        one_data = one_data.loc[(one_data['lat'] != 0) & (one_data['lon'] != 0)]
        if lat0 is None and lon0 is None:
            lat0, lon0 = one_data.iloc[0]["lat"] * 1e-7, one_data.iloc[0]["lon"] * 1e-7
        one_data[["x", "y"]] = one_data.apply(
            lambda row: pd.Series(latlon_to_meters(row["lat"] * 1e-7, row["lon"] * 1e-7, lat0, lon0)),
            axis=1
        )
        # Конвертируем heading (hdg) из centidegrees в degrees
        one_data["hdg"] = one_data["hdg"] * 0.01
        one_data["lat"] = one_data["lat"] * 1e-7
        one_data["lon"] = one_data["lon"] * 1e-7
        data_list.append(one_data)

    combined_data = combine_data(data_list)
    combined_data = fill_missing_values(combined_data, num_drones)
    combined_data = combined_data.loc[
        pd.notna(combined_data["x_0"]) &
        pd.notna(combined_data["x_1"]) &
        pd.notna(combined_data["x_2"]) &
        pd.notna(combined_data["x_3"])
    ]
    starting_time = combined_data.iloc[0]["time"]
    combined_data[["time"]] = combined_data.apply(
            lambda row: pd.Series(row["time"] - starting_time),
            axis=1
    )
    time_to_data = create_time_to_data(combined_data, num_drones)
    return time_to_data


def plot_error(time_data, error_data):
    plt.figure(figsize=(50, 12))
    plt.plot(time_data, error_data, marker='.', linestyle='-', color='b', label='Formation Error')
    plt.xlabel('Time', fontsize=12)
    plt.ylabel('Error (m)', fontsize=12)
    plt.title('Formation Error vs Time', fontsize=14)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


def plot_trajectories(time_to_data, num_drones):
    # Создаем фигуру с двумя subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 8))

    # Подготовка данных для каждого дрона
    all_times = sorted(time_to_data.keys())
    drone_x = [[] for _ in range(num_drones)]
    drone_y = [[] for _ in range(num_drones)]
    drone_lat = [[] for _ in range(num_drones)]
    drone_lon = [[] for _ in range(num_drones)]

    for time in all_times:
        data = time_to_data[time]
        for i in range(num_drones):
            drone_x[i].append(data[i]['x'])
            drone_y[i].append(data[i]['y'])
            drone_lat[i].append(data[i]['lat'])
            drone_lon[i].append(data[i]['lon'])

    # График в метрах
    for i in range(num_drones):
        ax1.plot(drone_x[i], drone_y[i],
                color=DRONE_COLORS[i],
                label=f'Drone {i}',
                linewidth=2)
        # Начальная точка
        ax1.scatter(drone_x[i][0], drone_y[i][0],
                   color=DRONE_COLORS[i],
                   marker='o', s=100,
                   edgecolor='black',
                   label=f'Start {i}')
        # Конечная точка
        ax1.scatter(drone_x[i][-1], drone_y[i][-1],
                   color=DRONE_COLORS[i],
                   marker='s', s=100,
                   edgecolor='black',
                   label=f'End {i}')

    ax1.set_title('Drone Trajectories (Meters from Reference)', fontsize=14)
    ax1.set_xlabel('X (m)', fontsize=12)
    ax1.set_ylabel('Y (m)', fontsize=12)
    ax1.grid(True)
    ax1.legend()

    # График в географических координатах
    for i in range(num_drones):
        ax2.plot(drone_lon[i], drone_lat[i],
                color=DRONE_COLORS[i],
                label=f'Drone {i}',
                linewidth=2)
        # Начальная точка
        ax2.scatter(drone_lon[i][0], drone_lat[i][0],
                   color=DRONE_COLORS[i],
                   marker='o', s=100,
                   edgecolor='black',
                   label=f'Start {i}')
        # Конечная точка
        ax2.scatter(drone_lon[i][-1], drone_lat[i][-1],
                   color=DRONE_COLORS[i],
                   marker='s', s=100,
                   edgecolor='black',
                   label=f'End {i}')

    ax2.set_title('Drone Trajectories (Geographic Coordinates)', fontsize=14)
    ax2.set_xlabel('Longitude (degrees)', fontsize=12)
    ax2.set_ylabel('Latitude (degrees)', fontsize=12)
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    plt.show()


def smooth_data(data):
    new_data = list()
    n = len(data)

    if n >= 1:
        new_data.append(data[0])
    if n >= 2:
        new_data.append((data[0] + data[1]) / 2)
    if n >= 3:
        for i in range(1, n-1):
            new_data.append((data[i-1] + data[i] + data[i+1]) / 3)

    return new_data


def main():
    time_to_data = get_data(DRONE_COUNT)
    error_data = list()
    time_data = list()
    for time, drone_data in time_to_data.items():
        headings = [d['hdg'] for d in drone_data]
        avg_heading = np.mean(headings)

        # Вращаем координаты дронов так, чтобы среднее направление было "вверх" (0 градусов)
        rotated_coords = []
        center_x = np.mean([d['x'] for d in drone_data])
        center_y = np.mean([d['y'] for d in drone_data])

        for d in drone_data:
            shifted_x = d['x'] - center_x
            shifted_y = d['y'] - center_y
            rot_x, rot_y = rotate_point(shifted_x, shifted_y, avg_heading)
            rotated_coords.append([rot_x, rot_y])

        kd_tree = spatial.KDTree(rotated_coords)
        count = len(rotated_coords)
        formation_error = 0
        for src_drone in rotated_coords:
            dist, indx = kd_tree.query(src_drone, k=9)
            neighbour_count = 0
            drone_error = 0
            for i, d in zip(indx[1:count], dist[1:count]):  # skip first because its src drone
                if d >= 1.5 * EXP_DISTANCE:
                    continue
                neighbour_count += 1
                target_drone = rotated_coords[i]
                target_delta = [target_drone[0] - src_drone[0], target_drone[1] - src_drone[1]]
                min_dist = min([distance(target_pos, target_delta) for target_pos in TARGET_POSITIONS])
                drone_error += min_dist

            if neighbour_count > 0:
                formation_error += drone_error / neighbour_count

        time_data.append(time)
        error_data.append(formation_error / count)

    smoothed_error = smooth_data(error_data)

    plot_error(time_data, smoothed_error)
    plot_trajectories(time_to_data, DRONE_COUNT)

if __name__ == "__main__":
    main()

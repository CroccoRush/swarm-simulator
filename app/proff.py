import math
from collections import defaultdict

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
EARTH_RADIUS = 6_372_795  # Earth radius in meters

# Colors for each drone and experiment
DRONE_COLORS = ['blue', 'green', 'red', 'orange', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']
EXPERIMENT_COLORS = ['blue', 'red', 'green', 'purple', 'orange', 'cyan', 'magenta', 'yellow', 'black', 'brown']


def rotate_point(x, y, angle_deg):
    """
    Rotates the point (x, y) around the origin by the given angle in degrees.
    """
    angle_rad = math.radians(angle_deg)
    x_rot = x * math.cos(angle_rad) - y * math.sin(angle_rad)
    y_rot = x * math.sin(angle_rad) + y * math.cos(angle_rad)
    return x_rot, y_rot


def latlon_to_meters(lat, lon, lat0, lon0):
    """
    Converts latitude and longitude to meters relative to the reference point.
    :param lat: Current latitude (in degrees).
    :param lon: Current longitude (in degrees).
    :param lat0: Latitude of the reference point (in degrees).
    :param lon0: Longitude of the reference point (in degrees).
    :return: Tuple (x, y) in meters.
    """
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)
    lat0_rad = np.radians(lat0)
    lon0_rad = np.radians(lon0)
    
    x = EARTH_RADIUS * (lon_rad - lon0_rad) * np.cos(lat0_rad)
    y = EARTH_RADIUS * (lat_rad - lat0_rad)
    
    return x, y


def meters_to_latlon(x, y, lat0, lon0):
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)

    lat_rad = lat0_rad + y / EARTH_RADIUS
    lon_rad = lon0_rad + x / (EARTH_RADIUS * math.cos(lat0_rad))

    return math.degrees(lat_rad), math.degrees(lon_rad)


def distance(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def combine_data(data_list):
    # Combine data by time
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
    
    # Sort by time
    combined_data = combined_data.sort_values(by='time').reset_index(drop=True)
    
    # Round time stamps to milliseconds for grouping
    combined_data['time_rounded'] = (combined_data['time'] // 10**8) * 10**8
    
    # List of columns to average for each time stamp
    columns_to_average = []
    for i in range(len(data_list)):
        for col_prefix in ['x_', 'y_', 'hdg_', 'lat_', 'lon_']:
            col_name = f"{col_prefix}{i}"
            if col_name in combined_data.columns:
                columns_to_average.append(col_name)
    
    # Group by rounded time stamp and calculate the mean
    grouped_data = combined_data.groupby('time_rounded')[columns_to_average].mean().reset_index()
    
    # Set the original time as the mean of the time stamp
    time_avg = combined_data.groupby('time_rounded')['time'].mean().reset_index()
    grouped_data['time'] = time_avg['time']
    
    # Sort again by time
    grouped_data = grouped_data.sort_values(by='time').reset_index(drop=True)
    
    return grouped_data


def fill_missing_values(df, num_drones):
    """Fills missing values with previous known positions"""
    for i in range(num_drones):
        # Fill missing values with forward fill method
        for col in [f'x_{i}', f'y_{i}', f'lat_{i}', f'lon_{i}', f'hdg_{i}']:
            df[col] = df[col].ffill()
    return df


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


def get_data(num_drones: int, suffix: str = ""):
    data_list = list()
    lat0, lon0 = None, None  # reference point for distance calculation
    for id in range(num_drones):
        one_data = pd.read_csv(f"./logs/drone_{id}_position{suffix}.csv", sep=",")
        one_data = one_data.loc[(one_data['lat'] != 0) & (one_data['lon'] != 0)]
        if lat0 is None and lon0 is None:
            lat0, lon0 = one_data.iloc[0]["lat"] * 1e-7, one_data.iloc[0]["lon"] * 1e-7
        one_data[["x", "y"]] = one_data.apply(
            lambda row: pd.Series(latlon_to_meters(row["lat"] * 1e-7, row["lon"] * 1e-7, lat0, lon0)),
            axis=1
        )
        # Convert heading (hdg) from centidegrees to degrees
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


def plot_error(time_data1, error_data1, time_data2, error_data2):
    plt.figure(figsize=(50, 12))
    plt.plot(time_data1, error_data1, marker='.', linestyle='-', color='b', label='Formation Error (LVP)')
    plt.plot(time_data2, error_data2, marker='.', linestyle='-', color='r', label='Formation Error (ALVP)')
    plt.xlabel('Time', fontsize=12)
    plt.ylabel('Error (m)', fontsize=12)
    plt.title('Formation Error vs Time', fontsize=14)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


def plot_trajectories(time_to_data1, time_to_data2, num_drones):
    fig, ax = plt.subplots(figsize=(8, 16))

    # Function to prepare trajectory data
    def prepare_trajectory_data(time_to_data):
        all_times = sorted(time_to_data.keys())
        drone_x = [[] for _ in range(num_drones)]
        drone_y = [[] for _ in range(num_drones)]

        for time in all_times:
            data = time_to_data[time]
            for i in range(num_drones):
                drone_x[i].append(data[i]['x'])
                drone_y[i].append(data[i]['y'])
        return drone_x, drone_y

    # Preparing data for the first set
    drone_x1, drone_y1 = prepare_trajectory_data(time_to_data1)
    # Preparing data for the second set
    drone_x2, drone_y2 = prepare_trajectory_data(time_to_data2)

    # Graph in meters for the first set (solid lines)
    for i in range(num_drones):
        ax.plot(drone_x1[i], drone_y1[i],
                color=DRONE_COLORS[i],
                label=f'Drone {i} (LVP)',
                linewidth=2)
        # Initial point
        ax.scatter(drone_x1[i][0], drone_y1[i][0],
                   color=DRONE_COLORS[i],
                   marker='o', s=100,
                   edgecolor='black',
                   label=f'Start {i}')
        # Final point
        ax.scatter(drone_x1[i][-1], drone_y1[i][-1],
                   color=DRONE_COLORS[i],
                   marker='s', s=100,
                   edgecolor='black',
                   label=f'End {i}')

    # Graph in meters for the second set (dashed lines)
    for i in range(num_drones):
        ax.plot(drone_x2[i], drone_y2[i],
                color=DRONE_COLORS[i],
                linestyle='--',
                label=f'Drone {i} (ALVP)',
                linewidth=2)
        # Initial point
        ax.scatter(drone_x2[i][0], drone_y2[i][0],
                   color=DRONE_COLORS[i],
                   marker='o', s=100,
                   edgecolor='black',
                   label=f'Start {i}')
        # Final point
        ax.scatter(drone_x2[i][-1], drone_y2[i][-1],
                   color=DRONE_COLORS[i],
                   marker='s', s=100,
                   edgecolor='black',
                   label=f'End {i}')

    ax.set_title('Drone Trajectories Comparison (Meters from Reference)', fontsize=14)
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.grid(True)
    ax.legend()

    plt.tight_layout()
    plt.show()


def smooth_data(data, window_size=5):
    """Smooths data using a moving average"""
    if len(data) < window_size:
        return data
        
    smoothed = []
    for i in range(len(data)):
        # Determine the start and end of the window
        start = max(0, i - window_size // 2)
        end = min(len(data), i + window_size // 2 + 1)
        window = data[start:end]
        smoothed.append(np.mean(window))
    return smoothed


def calculate_formation_error(drones_positions):
    """
    Calculates the formation error for given drone positions
    
    Parameters:
    -----------
    drones_positions : list of list
        List of drone positions in the format [[x1, y1], [x2, y2], ...]
    
    Returns:
    --------
    float
        Formation error
    """
    kd_tree = spatial.KDTree(drones_positions)
    count = len(drones_positions)
    formation_error = 0
    
    for src_drone in drones_positions:
        dist, indx = kd_tree.query(src_drone, k=count)
        neighbour_count = 0
        drone_error = 0
        
        for i, d in zip(indx[1:count], dist[1:count]):  # skip first because its src drone
            if d >= 1.5 * EXP_DISTANCE:
                continue
            neighbour_count += 1
            target_drone = drones_positions[i]
            target_delta = [target_drone[0] - src_drone[0], target_drone[1] - src_drone[1]]
            min_dist = min([distance(target_pos, target_delta) for target_pos in TARGET_POSITIONS])
            drone_error += min_dist
        
        if neighbour_count != 0:
            formation_error += drone_error / neighbour_count
    
    # Average formation error
    if count != 0:
        formation_error = formation_error / count
    
    return formation_error


def process_formation(time_to_data):
    """
    Calculates the formation error for each moment of time

    Parameters:
    -----------
    time_to_data : dict
        Experiment data

    Returns:
    --------
    times : list
        Time stamps
    errors : list
        Formation errors for corresponding time stamps
    """
    times, errors, errors_by_time = [], [], defaultdict(list)

    for time, drone_data in time_to_data.items():
        # Round time to 0.1 seconds
        time_rounded = (time // 10 ** 8) * 10 ** 8
        headings = [d['hdg'] for d in drone_data]
        avg_heading = np.mean(headings)

        rotated_coords = get_rotated_coordinates(drone_data, avg_heading)
        formation_error = calculate_formation_error(rotated_coords)

        times.append(time_rounded)
        errors.append(formation_error)
        errors_by_time[time_rounded].append(formation_error)

    return times, errors, errors_by_time


def process_experiment_data(param_set, exp_num):
    """
    Processes the data of a single experiment and calculates the formation errors
    
    Parameters:
    -----------
    param_set : int
        Number of the parameter set (1 for LVP, 2 for ALVP)
    exp_num : int
        Number of the experiment
        
    Returns:
    --------
    filtered_times : list
        Filtered and normalized time stamps
    filtered_errors : list
        Formation errors for corresponding time stamps
    """
    try:
        # Get data for the specified algorithm and experiment
        time_to_data = get_data(DRONE_COUNT, f"_set{param_set}_exp{exp_num}")
        times, errors, _ = process_formation(time_to_data)
        
        # Sort data by time
        sorted_indices = np.argsort(times)
        times = [times[i] for i in sorted_indices]
        errors = [errors[i] for i in sorted_indices]
        
        # Smooth errors if there is enough data
        if len(errors) > 5:
            errors = smooth_data(errors)
            if len(errors) < len(times):
                times = times[:len(errors)]
        
        # Determine the start time of the active phase (after half of the time)
        if not times:
            return [], []
            
        min_time = min(times)
        max_time = max(times)
        time_range = max_time - min_time
        start_time = min_time + time_range // 2
        
        # Filter and normalize time
        filtered_times = []
        filtered_errors = []
        for t, e in zip(times, errors):
            if t >= start_time:
                filtered_times.append((t - start_time) / 10**9)  # Time in seconds from the start of the active phase
                filtered_errors.append(e)
        
        return filtered_times, filtered_errors
        
    except Exception as e:
        algorithm = "LVP" if param_set == 1 else "ALVP"
        print(f"Error processing {algorithm} exp{exp_num}: {e}")
        return [], []


def add_vertical_time_markers(ax, time_markers, y_max):
    """Adds vertical lines for time markers"""
    for i, marker in enumerate(time_markers):
        ax.axvline(x=marker, color='r', linestyle='--', alpha=0.7)
        acts = ['Run', '    Flying straight', '    U-turn', '    Flying straight', '    Rest']
        ax.text(marker, 0.95 * y_max, f"{acts[i]}", color='r', verticalalignment='top')


def plot_specific_experiments(experiment_nums):
    """
    Plots formation errors for specific experiments comparing LVP and ALVP algorithms.
    
    Parameters:
    experiment_nums (list): A list of experiment numbers to plot
    """
    print(f"Analyzing specific experiments: {experiment_nums}")

    plt.figure(figsize=(12, 6))
    lvp_data = {}  # Format: {exp_num: (times, errors)}
    alvp_data = {}
    for i, exp_num in enumerate(experiment_nums):
        print(f"\nProcessing experiment {exp_num}")
        color = EXPERIMENT_COLORS[i % len(EXPERIMENT_COLORS)]
        
        # Process data of the LVP algorithm (parameter 1)
        lvp_times, lvp_errors = process_experiment_data(1, exp_num)
        
        # If there is data, save and build a graph for LVP
        if lvp_times and lvp_errors:
            lvp_data[exp_num] = (lvp_times, lvp_errors)
            plt.plot(lvp_times, lvp_errors, linestyle='-', color=color, 
                     linewidth=1.5, alpha=0.7, label=f'LVP Exp {exp_num}')
        
        # Process data of the ALVP algorithm (parameter 2)
        alvp_times, alvp_errors = process_experiment_data(2, exp_num)
        
        # If there is data, save and build a graph for ALVP
        if alvp_times and alvp_errors:
            alvp_data[exp_num] = (alvp_times, alvp_errors)
            plt.plot(alvp_times, alvp_errors, linestyle='--', color=color, 
                     linewidth=1.5, alpha=0.7, label=f'ALVP Exp {exp_num}')
    
    # Check if there is data for building a graph
    if not lvp_data and not alvp_data:
        print("No data available for specific experiments plot")
        return
    
    # Collect all times to determine the X axis
    all_times = []
    for exp_data in lvp_data.values():
        all_times.extend(exp_data[0])
    for exp_data in alvp_data.values():
        all_times.extend(exp_data[0])
    
    if all_times:
        # Determine the minimum and maximum value for the data
        all_errors = []
        for exp_data in lvp_data.values():
            all_errors.extend(exp_data[1])
        for exp_data in alvp_data.values():
            all_errors.extend(exp_data[1])
        
        y_min = min(all_errors) * 0.9 if all_errors else 0
        y_max = max(all_errors) * 1.1 if all_errors else 10
        
        # Add vertical lines
        time_markers = [max(all_times) - seconds for seconds in [66.5, 65, 45, 30, 10]]
        add_vertical_time_markers(
            plt.gca(),
            time_markers,
            y_max
        )
        
        # Set the limits of the axes
        plt.ylim(y_min, y_max)
    
    # Add the title and axes labels
    plt.title('Formation Error Comparison for Specific Experiments: LVP vs ALVP')
    plt.xlabel('Time from Experiment Start (seconds)')
    plt.ylabel('Formation Error (meters)')
    
    # Add the legend and grid
    plt.legend(loc='upper right')
    plt.grid(True, alpha=0.3)
    
    # Save the graph
    plt.tight_layout()
    plt.savefig('specific_experiments_comparison.png')
    print("Saved specific experiments comparison graph to specific_experiments_comparison.png")
    plt.close()


def plot_single_experiment(exp_num: int):
    """
    Plots formation errors for single experiment comparing LVP and ALVP algorithms.

    Parameters:
    experiment_num (int): A number of experiment to plot
    """
    print(f"Analyzing specific experiment: {exp_num}")

    plt.figure(figsize=(12, 6))
    lvp_data = ()
    alvp_data = ()

    # Process data of the LVP algorithm (parameter 1)
    lvp_times, lvp_errors = process_experiment_data(1, exp_num)

    # If there is data, save and build a graph for LVP
    if lvp_times and lvp_errors:
        lvp_data = (lvp_times, lvp_errors)
        plt.plot(lvp_times, lvp_errors, linestyle='-', color='blue',
                 linewidth=1.5, alpha=0.7, label=f'LVP Algorithm')

    # Process data of the ALVP algorithm (parameter 2)
    alvp_times, alvp_errors = process_experiment_data(2, exp_num)

    # If there is data, save and build a graph for ALVP
    if alvp_times and alvp_errors:
        alvp_data = (alvp_times, alvp_errors)
        plt.plot(alvp_times, alvp_errors, linestyle='-', color='red',
                 linewidth=1.5, alpha=0.7, label=f'ALVP Algorithm')

    # Check if there is data for building a graph
    if not lvp_data and not alvp_data:
        print("No data available for experiment plot")
        return

    # Collect all times to determine the X axis
    all_times = []
    all_times.extend(lvp_data[0])
    all_times.extend(alvp_data[0])

    if all_times:
        # Determine the minimum and maximum value for the data
        all_errors = []
        all_errors.extend(lvp_data[1])
        all_errors.extend(alvp_data[1])

        y_min = min(all_errors) * 0.9 if all_errors else 0
        y_max = max(all_errors) * 1.1 if all_errors else 10

        # Add vertical lines
        time_markers = [max(all_times) - seconds for seconds in [66.5, 65, 45, 30, 10]]
        add_vertical_time_markers(
            plt.gca(),
            time_markers,
            y_max
        )

        # Set the limits of the axes
        plt.ylim(y_min, y_max)

    # Add the title and axes labels
    plt.title('Formation Error Comparison for Experiments: LVP vs ALVP')
    plt.xlabel('Time from Experiment Start (seconds)')
    plt.ylabel('Formation Error (meters)')

    # Add the legend and grid
    plt.legend(loc='upper right')
    plt.grid(True, alpha=0.3)

    # Save the graph
    plt.tight_layout()
    plt.savefig('single_experiments_comparison.png')
    print("Saved single experiments comparison graph to single_experiments_comparison.png")
    plt.close()


def get_rotated_coordinates(drone_data, avg_heading):
    """
    Gets the rotated coordinates of the drones to align them according to the average heading
    
    Parameters:
    -----------
    drone_data : list of dict
        Data about the drones containing x, y and hdg
    avg_heading : float
        Average heading
        
    Returns:
    --------
    list of list
        List of rotated coordinates in the format [[x1, y1], [x2, y2], ...]
    """
    # Rotate coordinates to align them according to the average heading
    rotated_coords = []
    center_x = np.mean([d['x'] for d in drone_data])
    center_y = np.mean([d['y'] for d in drone_data])
    
    for d in drone_data:
        shifted_x = d['x'] - center_x
        shifted_y = d['y'] - center_y
        rot_x, rot_y = rotate_point(shifted_x, shifted_y, avg_heading)
        rotated_coords.append([rot_x, rot_y])
    
    return rotated_coords


def average(n_experiments=10):
    """
    Builds a graph comparing average formation errors for LVP and ALVP algorithms.
    
    Parameters:
    n_experiments (int): Number of experiments to analyze
    """
    lvp_avg_errors = {}  # Format: {time: [errors from all experiments]}
    alvp_avg_errors = {}
    for param_set in [1, 2]:
        for exp_num in range(1, n_experiments + 1):
            print(f"\nProcessing set{param_set}_exp{exp_num}")
            
            # Get data for all drones for the experiment
            try:
                time_to_data = get_data(DRONE_COUNT, f"_set{param_set}_exp{exp_num}")
                _, _, errors_by_time = process_formation(time_to_data)
                if param_set == 1:
                    lvp_avg_errors = errors_by_time
                else:
                    alvp_avg_errors = errors_by_time
            
            except Exception as e:
                print(f"Error processing experiment set{param_set}_exp{exp_num}: {e}")
                continue
    
    # Check if there is data
    if not lvp_avg_errors and not alvp_avg_errors:
        print("No valid data found for analysis!")
        return
    
    # Get all unique time stamps
    all_times = sorted(set(list(lvp_avg_errors.keys()) + list(alvp_avg_errors.keys())))
    if not all_times:
        print("No valid time data found!")
        return
    
    # Determine the start time of the active phase (after half of the time)
    min_time = min(all_times)
    max_time = max(all_times)
    time_range = max_time - min_time
    start_time = min_time + time_range // 2
    
    # Filter data by time and calculate the average values
    lvp_times = []
    lvp_errors = []
    alvp_times = []
    alvp_errors = []
    
    for time in sorted([t for t in all_times if t >= start_time]):
        # Process LVP data
        if time in lvp_avg_errors and lvp_avg_errors[time]:
            lvp_times.append((time - start_time) / 10**9)  # Time in seconds from the start of the active phase
            lvp_errors.append(np.mean(lvp_avg_errors[time]))
        
        # Process ALVP data
        if time in alvp_avg_errors and alvp_avg_errors[time]:
            alvp_times.append((time - start_time) / 10**9)  # Time in seconds from the start of the active phase
            alvp_errors.append(np.mean(alvp_avg_errors[time]))
    
    # Smooth data if there is enough data
    if len(lvp_errors) > 5:
        lvp_errors = smooth_data(lvp_errors)
        # Correct the time length if the error length has changed
        if len(lvp_errors) < len(lvp_times):
            lvp_times = lvp_times[:len(lvp_errors)]
    
    if len(alvp_errors) > 5:
        alvp_errors = smooth_data(alvp_errors)
        # Correct the time length if the error length has changed
        if len(alvp_errors) < len(alvp_times):
            alvp_times = alvp_times[:len(alvp_errors)]
    
    # Build a graph comparison
    plt.figure(figsize=(12, 6))
    
    if lvp_times and lvp_errors:
        plt.plot(lvp_times, lvp_errors, 'b-', linewidth=2, label='LVP Algorithm')
    
    if alvp_times and alvp_errors:
        plt.plot(alvp_times, alvp_errors, 'r-', linewidth=2, label='ALVP Algorithm')
    
    # Get all times to determine the X axis
    all_times_for_plot = lvp_times + alvp_times
    if all_times_for_plot:
        # Determine the minimum and maximum value for the data
        all_errors = lvp_errors + alvp_errors
        y_min = min(all_errors) * 0.9 if all_errors else 0
        y_max = max(all_errors) * 1.1 if all_errors else 10
        
        # Add vertical lines
        time_markers = [max(all_times_for_plot) - seconds for seconds in [66.5, 65, 45, 30, 10]]
        add_vertical_time_markers(
            plt.gca(), 
            time_markers,
            y_max
        )
        
        # Set the limits of the axes
        plt.ylim(y_min, y_max)
    
    # Add the title and axes labels
    plt.title('Average Formation Error Comparison: LVP vs ALVP')
    plt.xlabel('Time from Experiment Start (seconds)')
    plt.ylabel('Formation Error (meters)')
    
    # Add the legend and grid
    plt.legend(loc='upper right')
    plt.grid(True, alpha=0.3)
    
    # Save the graph
    plt.tight_layout()
    plt.savefig('average_error_comparison.png')
    print("Saved average error comparison graph to average_error_comparison.png")
    plt.close()
    
    # Print the statistics
    print("\nStatistical Analysis of Average Errors:")
    if lvp_errors:
        print(f"LVP - Mean: {np.mean(lvp_errors):.2f}m, Std: {np.std(lvp_errors):.2f}m")
        # Calculate integral (area under curve) for LVP
        lvp_integral = np.trapezoid(lvp_errors, lvp_times)
        print(f"LVP - Integral: {lvp_integral:.2f} m·s")
    if alvp_errors:
        print(f"ALVP - Mean: {np.mean(alvp_errors):.2f}m, Std: {np.std(alvp_errors):.2f}m")
        # Calculate integral (area under curve) for ALVP
        alvp_integral = np.trapezoid(alvp_errors, alvp_times)
        print(f"ALVP - Integral: {alvp_integral:.2f} m·s")
    
    # Compare integrals
    if lvp_errors and alvp_errors:
        print(f"\nComparison of integrals:")
        print(f"LVP/ALVP ratio: {lvp_integral/alvp_integral:.2f}")
        print(f"Difference (LVP - ALVP): {lvp_integral - alvp_integral:.2f} m·s")


def create_boxplot(data, labels, title, output_filename, time_markers=None):
    """Creates a Box plot with given data and labels"""
    if not data or not labels:
        print(f"No data available for boxplot: {title}")
        return
        
    # Create a figure and axes
    plt.figure(figsize=(12, 6))
    ax = plt.subplot(111)
        
    # Build a Box plot
    box = ax.boxplot(data, patch_artist=True, tick_labels=labels)
    
    # Configure the styles for the Box plot
    for patch in box['boxes']:
        patch.set_facecolor('lightblue')
    
    for whisker in box['whiskers']:
        whisker.set(color='gray', linewidth=1)
    
    for cap in box['caps']:
        cap.set(color='gray', linewidth=1)
    
    for median in box['medians']:
        median.set(color='red', linewidth=1.5)
    
    for flier in box['fliers']:
        flier.set(marker='o', markerfacecolor='red', markersize=4, markeredgecolor='none', alpha=0.5)
    
    # Determine the minimum and maximum value for the graph
    all_values = [val for sublist in data for val in sublist]
    y_min = min(all_values) * 0.9 if all_values else 0
    y_max = max(all_values) * 1.1 if all_values else 10
    
    # Set the limits of the axes
    ax.set_ylim(y_min, y_max)
    
    # If time markers are provided, add vertical lines
    if time_markers:
        add_vertical_time_markers(ax, time_markers, y_max)
    
    # Add the title and axes labels
    plt.title(title)
    plt.xlabel('Time from Experiment Start (seconds)')
    plt.ylabel('Formation Error (meters)')
    
    # Configure the labels on the X axis - show only every 20 labels for readability
    if len(labels) > 20:
        step = len(labels) // 20
        plt.xticks(range(0, len(labels), step), labels[::step], rotation=45, ha='right')
    else:
        plt.xticks(rotation=45, ha='right')
    
    # Add the grid and legend
    plt.grid(True, alpha=0.3)
    
    # Save the graph
    plt.tight_layout()
    plt.savefig(output_filename)
    print(f"Saved graph to {output_filename}")
    plt.close()


def plot_error_lines(error_data_by_exp, algorithm):
    """
    Builds a graph of formation errors for all experiments of one algorithm
    
    Parameters:
    -----------
    error_data_by_exp : dict
        Dictionary with data about formation errors for each experiment
    algorithm : str
        Name of the algorithm ('LVP' or 'ALVP')
    """
    if not error_data_by_exp:
        print(f"No data available for {algorithm} error line plot")
        return

    plt.figure(figsize=(12, 6))
    ax = plt.subplot(111)

    all_times = []
    for exp_num, (times, _) in error_data_by_exp.items():
        all_times.extend(times)
    
    if not all_times:
        print(f"No time data available for {algorithm} error line plot")
        return
    
    min_time = min(all_times)
    max_time = max(all_times)
    
    # Filter and build data for each experiment
    y_min = float('inf')
    y_max = float('-inf')

    times_sec = []
    for idx, (exp_num, data) in enumerate(error_data_by_exp.items()):
        times, errors = data
        
        # Convert times to seconds from the start of the experiment
        times_sec = [(t - min_time) / 10**9 for t in times]
        
        # Sort data by time
        sorted_data = sorted(zip(times_sec, errors), key=lambda x: x[0])
        times_sec, errors = zip(*sorted_data) if sorted_data else ([], [])
        
        # Smooth errors if there is enough data
        if len(errors) > 5:
            errors = smooth_data(errors)
            # Cut times if the length of the error array has changed
            if len(errors) < len(times_sec):
                times_sec = times_sec[:len(errors)]
        
        # Update min/max for the Y axis
        if errors:
            y_min = min(y_min, min(errors))
            y_max = max(y_max, max(errors))
        
        # Build a graph of errors for the current experiment
        color = EXPERIMENT_COLORS[idx % len(EXPERIMENT_COLORS)]
        ax.plot(times_sec, errors, color=color, linewidth=1.5, label=f'Experiment {exp_num}')
    
    # Configure the limits of the axes
    if y_min != float('inf') and y_max != float('-inf'):
        ax.set_ylim(y_min * 0.9, y_max * 1.1)
    
    # Add vertical time markers
    if times_sec:
        time_markers = [max(times_sec) - seconds for seconds in [66.5, 65, 45, 30, 10]]
        add_vertical_time_markers(ax, time_markers, y_max * 1.1)
    
    # Add the title and axes labels
    plt.title(f'Formation Error Over Time: {algorithm} Algorithm')
    plt.xlabel('Time from Experiment Start (seconds)')
    plt.ylabel('Formation Error (meters)')
    
    # Add the legend
    plt.legend(loc='upper right', fontsize=9)
    
    # Add the grid
    plt.grid(True, alpha=0.3)
    
    # Save the graph
    plt.tight_layout()
    plt.savefig(f'{algorithm.lower()}_error_lines.png')
    print(f"Saved error line graph to {algorithm.lower()}_error_lines.png")
    plt.close()


def process_error_data_by_time(errors_by_time, start_time):
    """
    Processes errors by time, applying filtering and smoothing
    
    Parameters:
    -----------
    errors_by_time : dict
        Dictionary with formation errors by time {time: [errors]}
    start_time : int
        Start time of the active phase for filtering
        
    Returns:
    --------
    dict
        Filtered and smoothed dictionary with errors
    list
        Prepared data for boxplot
    list
        Time labels for boxplot
    """
    # Filter data by time
    filtered_data = {t: errors for t, errors in errors_by_time.items() if t >= start_time}
    
    # Smooth data for each time point
    for t, errors in filtered_data.items():
        if len(errors) > 5:  # Apply smoothing only if there is enough data
            filtered_data[t] = smooth_data(errors)
    
    # Additional smoothing along the time axis (median smoothing)
    if len(filtered_data) > 5:
        sorted_times = sorted(filtered_data.keys())
        medians = [np.median(filtered_data[t]) for t in sorted_times]
        smoothed_medians = smooth_data(medians)
        
        # Correct the sizes of arrays if needed
        if len(smoothed_medians) < len(sorted_times):
            sorted_times = sorted_times[:len(smoothed_medians)]
        
        # Apply correction to data based on smoothed medians
        for i, t in enumerate(sorted_times):
            if t in filtered_data:
                # Calculate the correction factor
                original_median = np.median(filtered_data[t])
                if original_median > 0:
                    correction_factor = smoothed_medians[i] / original_median
                    # Apply correction to each value
                    filtered_data[t] = [e * correction_factor for e in filtered_data[t]]
    
    # Prepare data for Box plot
    boxplot_data = []
    boxplot_labels = []
    
    for time in sorted(filtered_data.keys()):
        boxplot_data.append(filtered_data[time])
        # Time from the start of the active phase (instead of absolute time)
        boxplot_labels.append(f"{(time-start_time)/10**9:.1f}")
    
    return filtered_data, boxplot_data, boxplot_labels


def analyze_logs(n_experiments=10):
    """Analyzes logs and builds Box plot of formation errors"""
    # Dictionaries for storing formation errors by time
    lvp_errors_by_time = defaultdict(list)
    alvp_errors_by_time = defaultdict(list)

    # Dictionaries for storing formation errors by experiments
    lvp_errors_by_exp = {}
    alvp_errors_by_exp = {}

    # Analyze experiments for LVP (1) and ALVP (2)
    for param_set in [1, 2]:
        for exp_num in range(1, n_experiments + 1):
            print(f"\nProcessing set{param_set}_exp{exp_num}")
            
            # Get data for all drones for the experiment
            try:
                time_to_data = get_data(DRONE_COUNT, f"_set{param_set}_exp{exp_num}")
                exp_times, exp_errors, _ = process_formation(time_to_data)
                
                # Calculate formation error for each time point
                for time, drone_data in time_to_data.items():
                    # Round time to 0.1 seconds
                    time_rounded = (time // 10 ** 8) * 10 ** 8
                    headings = [d['hdg'] for d in drone_data]
                    avg_heading = np.mean(headings)

                    rotated_coords = get_rotated_coordinates(drone_data, avg_heading)
                    formation_error = calculate_formation_error(rotated_coords)

                    if param_set == 1:
                        lvp_errors_by_time[time_rounded].append(formation_error)
                    else:
                        alvp_errors_by_time[time_rounded].append(formation_error)
                
                # Smooth errors for the current experiment
                if len(exp_errors) > 5:  # Apply smoothing only if there is enough data
                    exp_errors = smooth_data(exp_errors)
                    # Correct the time length if the length of the error has changed after smoothing
                    if len(exp_errors) < len(exp_times):
                        exp_times = exp_times[:len(exp_errors)]
                
                # Save the experiment data in the corresponding dictionary
                if exp_times and exp_errors:
                    if param_set == 1:
                        lvp_errors_by_exp[exp_num] = (exp_times, exp_errors)
                    else:
                        alvp_errors_by_exp[exp_num] = (exp_times, exp_errors)
            
            except Exception as e:
                print(f"Error processing experiment set{param_set}_exp{exp_num}: {e}")
                continue
    
    # Check if there is data
    if not lvp_errors_by_time and not alvp_errors_by_time:
        print("No valid data found for analysis!")
        return
    
    # Determine the start time of the active phase (after half of the experiment time)
    all_times = []
    for errors_dict in [lvp_errors_by_time, alvp_errors_by_time]:
        all_times.extend(errors_dict.keys())
    
    if not all_times:
        print("No valid time data found!")
        return
        
    min_time = min(all_times)
    max_time = max(all_times)
    time_range = max_time - min_time
    start_time = min_time + time_range // 2
    
    # Process data for LVP and ALVP using a common function
    _, lvp_data, lvp_labels = process_error_data_by_time(lvp_errors_by_time, start_time)
    _, alvp_data, alvp_labels = process_error_data_by_time(alvp_errors_by_time, start_time)
    
    # Time markers for vertical lines (seconds from the end of the graph)
    time_markers = [66.5, 65, 45, 30, 10]

    # Create a Box plot for LVP
    create_boxplot(
        lvp_data, 
        lvp_labels, 
        'Formation Error Distribution Over Time: LVP Algorithm',
        'lvp_error_boxplot.png',
        time_markers
    )
    
    # Create a Box plot for ALVP
    create_boxplot(
        alvp_data, 
        alvp_labels, 
        'Formation Error Distribution Over Time: ALVP Algorithm',
        'alvp_error_boxplot.png',
        time_markers
    )
    
    # Build graphs of formation error lines for individual experiments (starting from half of the time)
    plot_error_lines(lvp_errors_by_exp, "LVP")
    plot_error_lines(alvp_errors_by_exp, "ALVP")
    
    # Print the statistics for the filtered data
    print("\nStatistical Analysis (after filtering):")
    if lvp_data:
        flat_lvp = [item for sublist in lvp_data for item in sublist]
        print(f"LVP - Mean: {np.mean(flat_lvp):.2f}m, Std: {np.std(flat_lvp):.2f}m")
    if alvp_data:
        flat_alvp = [item for sublist in alvp_data for item in sublist]
        print(f"ALVP - Mean: {np.mean(flat_alvp):.2f}m, Std: {np.std(flat_alvp):.2f}m")


def drone_paths(target_exp):
    """Visualizes the drone paths for LVP and ALVP algorithms"""
    # Dictionaries for storing drone trajectories data
    lvp_trajectories = {}
    alvp_trajectories = {}
    
    # Get data for LVP and ALVP algorithms
    for param_set in [1, 2]:  # 1 = LVP, 2 = ALVP
        try:
            # Get data for all drones for the experiment
            time_to_data = get_data(DRONE_COUNT, f"_set{param_set}_exp{target_exp}")
            
            # Convert data to a format convenient for building trajectories
            drone_trajectories = {i: {'times': [], 'x': [], 'y': []} for i in range(DRONE_COUNT)}
            
            # Filter and sort data by time
            sorted_times = sorted(time_to_data.keys())
            
            # Determine the start time of the active phase (half of the total time)
            min_time = sorted_times[0]
            max_time = sorted_times[-1]
            time_range = max_time - min_time
            start_time = min_time + time_range // 2
            
            # Collect data for each drone's trajectory
            for time in sorted_times:
                if time < start_time:
                    continue  # Skip data before the active phase
                
                drone_data = time_to_data[time]
                for i, data in enumerate(drone_data):
                    if i < DRONE_COUNT:
                        drone_trajectories[i]['times'].append(time)
                        drone_trajectories[i]['x'].append(data['x'])
                        drone_trajectories[i]['y'].append(data['y'])
            
            # Save the trajectory data
            if param_set == 1:
                lvp_trajectories = drone_trajectories
            else:
                alvp_trajectories = drone_trajectories
                
        except Exception as e:
            print(f"Error processing experiment set{param_set}_exp{target_exp}: {e}")
            continue
    
    # Check if there is data
    if not lvp_trajectories and not alvp_trajectories:
        print("No valid trajectory data found!")
        return
    
    # Create a common graph with two subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

    # Build trajectories for LVP
    ax1.set_title('Drone Trajectories: LVP Algorithm')
    if lvp_trajectories:
        for drone_id, data in lvp_trajectories.items():
            if data['x'] and data['y']:
                color = DRONE_COLORS[drone_id % len(DRONE_COLORS)]
                ax1.plot(data['x'], data['y'], color=color, linewidth=1.5, label=f'Drone {drone_id+1}')
                # Mark the final position of the drone
                ax1.scatter(data['x'][-1], data['y'][-1], color=color, s=100, marker='o')
                # Mark the initial position of the drone
                ax1.scatter(data['x'][0], data['y'][0], color=color, s=100, marker='x')
    else:
        ax1.text(0.5, 0.5, 'No LVP data available', ha='center', va='center')
    
    # Build trajectories for ALVP
    ax2.set_title('Drone Trajectories: ALVP Algorithm')
    if alvp_trajectories:
        for drone_id, data in alvp_trajectories.items():
            if data['x'] and data['y']:
                color = DRONE_COLORS[drone_id % len(DRONE_COLORS)]
                ax2.plot(data['x'], data['y'], color=color, linewidth=1.5, label=f'Drone {drone_id+1}')
                # Mark the final position of the drone
                ax2.scatter(data['x'][-1], data['y'][-1], color=color, s=100, marker='o')
                # Mark the initial position of the drone
                ax2.scatter(data['x'][0], data['y'][0], color=color, s=100, marker='x')
    else:
        ax2.text(0.5, 0.5, 'No ALVP data available', ha='center', va='center')
    
    # Configure the graphs
    for ax in [ax1, ax2]:
        ax.set_xlabel('X Position (meters)')
        ax.set_ylabel('Y Position (meters)')
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        # Set the same limits for both graphs
        # Collect all coordinates to determine the limits
        all_x = []
        all_y = []
        for trajectories in [lvp_trajectories, alvp_trajectories]:
            for _, data in trajectories.items():
                all_x.extend(data['x'])
                all_y.extend(data['y'])
        
        if all_x and all_y:
            x_min, x_max = min(all_x), max(all_x)
            y_min, y_max = min(all_y), max(all_y)
            
            # Add padding for better display
            x_padding = (x_max - x_min) * 0.1
            y_padding = (y_max - y_min) * 0.1
            
            # Set the limits
            ax.set_xlim(x_min - x_padding, x_max + x_padding)
            ax.set_ylim(y_min - y_padding, y_max + y_padding)
    
    # Add a common legend
    handles, labels = ax1.get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', ncol=5, bbox_to_anchor=(0.5, 0.05))
    
    # Improve the layout
    plt.tight_layout()
    plt.subplots_adjust(bottom=0.15)  # Place for the legend at the bottom
    
    # Save the graph
    plt.savefig('drone_trajectories.png')
    print("Saved drone trajectories visualization to drone_trajectories.png")
    plt.close()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="""
===================================
== Drone Formation Analysis Tool ==
===================================
""",
        usage="""
Available modes:
- boxplot: Generate Box plots and per-experiment error graphs
- average: Generate averaged error graph
- paths: Visualize drone paths
- specific: Compare specific experiments

Examples:
  python proff.py --mode boxplot
  python proff.py --mode average --num-experiments 5
  python proff.py --mode paths --target-exp 1
  python proff.py --mode specific --experiments 1 3 5
""",
    )
    parser.add_argument(
        '--mode', type=str,
        choices=['boxplot', 'average', 'paths', 'specific', 'single'], default='boxplot',
        help='Analysis mode: boxplot (Box plots and per-experiment error graphs), '
             'average (averaged error graph), paths (drone path visualization), '
             'specific (specific experiments comparison)',
    )
    parser.add_argument(
        '--experiments', type=int, nargs='+', default=[3, 6, 8],
        help='Experiment numbers to use in specific mode',
    )
    parser.add_argument(
        '--target-exp', type=int, default=1,
        help='Experiment number to use in plot path and single experiment',
    )
    parser.add_argument(
        '--num-experiments', type=int, default=10,
        help='Number of experiments to include in average mode',
    )
    args = parser.parse_args()
    
    # Choose the mode based on the arguments
    if args.mode == 'boxplot':
        print("Running Box plot analysis mode...")
        analyze_logs(args.num_experiments)
    elif args.mode == 'average':
        print(f"Running average error graph mode with {args.num_experiments} experiments...")
        average(args.num_experiments)
    elif args.mode == 'paths':
        print("Running drone path visualization mode...")
        drone_paths(args.target_exp)
    elif args.mode == 'specific':
        print(f"Running specific experiments mode with experiments: {args.experiments}...")
        plot_specific_experiments(args.experiments)
    elif args.mode == 'single':
        print(f"Running single experiment mode with experiment: {args.target_exp}...")
        plot_single_experiment(args.target_exp)
    else:
        print(f"Unknown mode: {args.mode}")
        parser.print_help()

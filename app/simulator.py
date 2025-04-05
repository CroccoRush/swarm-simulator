from network import NetworkSimulator
from experiment_runner import ExperimentRunner
from gamepad import DroneControlGUI
import argparse
import sys
import os
import time


def run_experiment_mode(sim):
    """
    Runs the simulator in experiment mode with automated experiments.
    
    Args:
        sim: NetworkSimulator instance
    """
    # Get the number of the parameter set and the number of the experiment from the environment
    param_set = int(os.environ.get("EXPERIMENT_SET", "1"))
    exp_num = int(os.environ.get("EXPERIMENT_NUM", "1"))
    
    print(f"Running experiment {exp_num} with algorithm set {param_set}")

    time_start = time.time()
    
    # Create a runner for experiments instead of GUI
    runner = ExperimentRunner(sim)
    
    # Set the parameters depending on the selected set
    if param_set == 1:
        # LVP parameters
        params = {
            "Swarm_XY_ALGO": "1",  # LVP алгоритм
            "Swarm_LVP_XY_A": "0.3",
        }
    else:
        # ALVP parameters
        params = {
            "Swarm_XY_ALGO": "2",  # ALVP algorithm
            "Swarm_ALVP_XY_A": "0.25",
            "Swarm_ALVP_XY_G": "10",
            "Swarm_ALVP_XY_H": "0.33",
            "Swarm_ALVP_XY_L": "3",
            "Swarm_ALVP_XY_ME": "10",
        }
    
    # Set the parameters
    runner.set_parameters(params)
    time.sleep(60 - (time.time() - time_start))
    runner.run_experiment()
    
    # Save the logs
    runner.save_logs(param_set, exp_num)
    
    print(f"Experiment {exp_num} with set {param_set} completed!")


def run_gui_mode(sim):
    """
    Runs the simulator in interactive GUI mode.
    
    Args:
        sim: NetworkSimulator instance
    """
    print("Starting simulator in GUI mode...")
    gui = DroneControlGUI(sim)
    gui.run()


def main():
    # Set up command-line argument parsing
    parser = argparse.ArgumentParser(description="Drone Swarm Simulator")
    parser.add_argument(
        '--mode', type=str, choices=['experiment', 'gui'], default='experiment',
        help='Simulator mode: experiment (automatic experiment run) or gui (interactive control)'
    )
    args = parser.parse_args()

    # Initialize the network simulator
    sim = NetworkSimulator("./config.json")

    # Choose mode based on command-line argument
    if args.mode == 'experiment':
        run_experiment_mode(sim)
    elif args.mode == 'gui':
        run_gui_mode(sim)
    else:
        print(f"Unknown mode: {args.mode}")
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()

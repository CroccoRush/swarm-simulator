#!/bin/bash

# Number of experiments for each parameter set
NUM_EXPERIMENTS=10

# Function for completing all processes
cleanup() {
    echo "Stopping all processes..."
    
    # Killing all remaining processes in the group
    pkill -f run_simulation.sh
    
    # Give time for completion
    sleep 3
    
    # Force termination if there is anything left
    pgrep -f "arducopter|ArduCopter" > /dev/null && {
        echo "Forcing kill of remaining SITL processes..."
        pkill -9 -f "arducopter"
        pkill -9 -f "ArduCopter"
    }
    
    # Check that everything is completed
    if pgrep -f "arducopter|ArduCopter|mavproxy.py|simulator.py" > /dev/null; then
        echo "WARNING: Some processes still running!"
        ps aux | grep -E "arducopter|ArduCopter|mavproxy.py|simulator.py" | grep -v grep
    else
        echo "All processes stopped successfully."
    fi
}

# Function for running a single experiment
run_experiment() {
    set_num=$1
    exp_num=$2
    
    echo "Running experiment $exp_num with parameter set $set_num..."
    
    # Set environment variables
    export EXPERIMENT_SET=$set_num
    export EXPERIMENT_NUM=$exp_num
    
    # Run simulation in experiment mode
    ./run_simulation.sh experiment
    
    # After the experiment is completed, complete all processes
    cleanup
    
    # Wait a few seconds before the next experiment
    echo "Waiting before next experiment..."
    sleep 5
}

# Interception of signals for correct termination
trap cleanup SIGINT SIGTERM

# Run experiments with the first parameter set (LVP)
echo "Starting experiments with algorithm set 1 (LVP)..."
for ((i=1; i<=$NUM_EXPERIMENTS; i++)); do
    run_experiment 1 $i
done

# Run experiments with the second parameter set (ALVP)
echo "Starting experiments with algorithm set 2 (ALVP)..."
for ((i=1; i<=$NUM_EXPERIMENTS; i++)); do
    run_experiment 2 $i
done

echo "All experiments completed!" 

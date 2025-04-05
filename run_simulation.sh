#!/bin/bash

# Get the simulator mode (default to gui if not specified)
SIM_MODE=${1:-gui}

export PATH=$PATH:/home/kiselyovvld/.local/bin  # Needed for mavproxy

RUN="./sitl/run_in_terminal_window.sh"
SITL_BIN="./sitl/arducopter"

# Loading the configuration
CONFIG_FILE="config.json"
DRONES=$(jq -c '.drones[]' $CONFIG_FILE)

# Running SITL for each drone
SITL_PIDS=()
for DRONE_CFG in $DRONES; do
    ID=$(echo $DRONE_CFG | jq -r '.id')
    UDP_PORT=$(echo $DRONE_CFG | jq -r '.udp_port')
    SERIAL5_PORT=$(echo $DRONE_CFG | jq -r '.serial5_port')
    LAT=$(echo $DRONE_CFG | jq -r '.initial_position.lat')
    LON=$(echo $DRONE_CFG | jq -r '.initial_position.lon')
    ALT=$(echo $DRONE_CFG | jq -r '.initial_position.alt')
    PARAMS_PATH="./params/copter_$ID.parm"
    COMMAND="$RUN ArduCopter $SITL_BIN -S --model + --speedup 1 --slave 0 --serial5=tcp:$SERIAL5_PORT:wait --defaults=$PARAMS_PATH --sim-address=127.0.0.1 --home=$LAT,$LON,$ALT,0 -I$ID"

    echo "Starting drone $ID at ($LAT, $LON, $ALT) on UDP port $UDP_PORT and SERIAL5 TCP port $SERIAL5_PORT ..."
    echo "Run $COMMAND ..."
    xterm -hold -e "$COMMAND 2>&1 | tee /tmp/drone_$ID.log" &
    SITL_PIDS+=($!)  # Saving the SITL Process PIDs
done

# Time for initialization
sleep 10

# Running the proxy
MAVPROXY_COMMAND="mavproxy.py --out 172.28.0.1:14550 --out 172.28.0.1:14551"

for DRONE_CFG in $DRONES; do
    ID=$(echo $DRONE_CFG | jq -r '.id')
    MASTER_PORT=$((5760 + 10 * ID))
    SITL_PORT=$((5501 + ID))
    OUT_PORT=$((14500 + 10 * ID))
    MAVPROXY_COMMAND+=" --master tcp:127.0.0.1:$MASTER_PORT --sitl 127.0.0.1:$SITL_PORT --out udp:0.0.0.0:$OUT_PORT"
done
echo "Run $MAVPROXY_COMMAND ..."

xterm -hold -e "$MAVPROXY_COMMAND 2>&1 | tee /tmp/mavproxy.log" &
MAVPROXY_PID=$!  # Saving the MAVProxy Process PID

# Time for initialization
sleep 5

# Running the simulator with the specified mode
echo "Starting simulator in $SIM_MODE mode..."
python3 app/simulator.py --mode $SIM_MODE &
SIMULATOR_PID=$!  # Saving the simulator Process PID

# Function for completing all processes
cleanup() {
    echo "Stopping all processes..."
    kill -TERM $SIMULATOR_PID 2>/dev/null
    kill -TERM $MAVPROXY_PID 2>/dev/null
    # Terminating SITL processes
    for PID in "${SITL_PIDS[@]}"; do
        kill -TERM $PID 2>/dev/null
    done
    pkill -f "arducopter"
    pkill -f "ArduCopter"

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

    exit 0
}

# Interception of signals for correct termination
trap cleanup SIGINT SIGTERM

# Waiting for the simulator to finish
wait $SIMULATOR_PID

# Termination of all processes after completion of the simulator
cleanup

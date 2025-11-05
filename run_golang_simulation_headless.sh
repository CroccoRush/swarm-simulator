#!/bin/bash

# Usage: ./run_golang_simulation_headless.sh [SIM_MODE] [CONNECTION_MODE] [SCENARIO_FILE]
# SIM_MODE: experiment, gui (default: experiment)
# CONNECTION_MODE: direct, qemu (default: direct)
# SCENARIO_FILE: Path to scenario file (default: scenarios/simple_flight.yaml)
#
# This version runs all processes in background without xterm windows
# Suitable for large number of drones (>128)

# Get the simulator mode (default to experiment if not specified)
SIM_MODE=${1:-experiment}
CONNECTION_MODE=${2:-direct}  # direct, or qemu
SCENARIO_FILE=${3:-"scenarios/simple_flight.yaml"}

# Show usage if help requested
if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    echo "Swarm Simulator Launch Script (Headless Mode)"
    echo "=============================================="
    echo ""
    echo "Usage: $0 [SIM_MODE] [CONNECTION_MODE] [SCENARIO_FILE]"
    echo ""
    echo "SIM_MODE:"
    echo "  experiment  - Run automated experiment (default)"
    echo "  gui         - Run with GUI interface"
    echo ""
    echo "CONNECTION_MODE:"
    echo "  direct      - Direct connection to ArduPilot SITL (default)"
    echo "  qemu        - Use QEMU ESP32 emulation layer"
    echo ""
    echo "SCENARIO_FILE:"
    echo "  Path to the YAML or JSON scenario file to execute in experiment mode."
    echo "  (default: \"scenarios/simple_flight.yaml\")"
    echo ""
    echo "Examples:"
    echo "  $0 experiment direct   # Standard mode with default scenario"
    echo "  $0 experiment qemu     # QEMU ESP32 emulation"
    echo ""
    echo "Requirements:"
    echo "  direct: ArduPilot SITL, MAVProxy"
    echo "  qemu:   ArduPilot SITL, MAVProxy, QEMU, ESP32 firmware"
    echo ""
    echo "Note: This version runs all processes in background without GUI windows"
    echo "      Check log files in logs/ directory for process output"
    echo ""
    exit 0
fi

export PATH=$PATH:/home/kiselyovvld/.local/bin  # Needed for mavproxy
export PATH=$PATH:/usr/local/go/bin  # Needed for Golang

SITL_BIN="./sitl/arducopter.bin"

# Build the Go simulator first
echo "Building Go simulator..."
cd golang_app || exit
if ! go build -o ../bin/golang-simulator ./cmd/simulator; then
    echo "Failed to build Go simulator"
    exit 1
fi
cd ..
echo "Go simulator built successfully"

# Loading the configuration
CONFIG_FILE="config.json"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Configuration file $CONFIG_FILE not found"
    exit 1
fi

DRONES=$(jq -c '.drones[]' $CONFIG_FILE)
if [ $? -ne 0 ]; then
    echo "Failed to parse configuration file"
    exit 1
fi

DRONE_COUNT=$(echo "$DRONES" | wc -l)
echo "Configuration loaded: $DRONE_COUNT drones"

# Validate connection mode
if [[ ! "$CONNECTION_MODE" =~ ^(direct|qemu)$ ]]; then
    echo "Invalid CONNECTION_MODE: $CONNECTION_MODE"
    echo "Valid options: direct, qemu"
    echo "Use --help for more information"
    exit 1
fi

# Check connection mode
case $CONNECTION_MODE in
    "qemu")
        echo "QEMU MODE - Starting with ESP32 emulation layer"
        QEMU_MODE=true
        ;;
    "direct"|*)
        echo "DIRECT MODE - Direct ArduPilot SITL connection"
        QEMU_MODE=false
        ;;
esac

# Real mode - start ArduPilot SITL and MAVProxy
echo "HEADLESS MODE - Starting ArduPilot SITL instances in background..."

# Create directories for logs
mkdir -p logs

# Running SITL for each drone
SITL_PIDS=()
echo "Starting SITL instances..."
for DRONE_CFG in $DRONES; do
    ID=$(echo $DRONE_CFG | jq -r '.id')
    UDP_PORT=$(echo $DRONE_CFG | jq -r '.udp_port')

    # Extract Serial5 configuration (new format)
    SERIAL5_TYPE=$(echo $DRONE_CFG | jq -r '.serial5.type // empty')
    SERIAL5_PORT=$(echo $DRONE_CFG | jq -r '.serial5.port // empty')
    SERIAL5_PATH=$(echo $DRONE_CFG | jq -r '.serial5.path // empty')

    # Backward compatibility: check old serial5_port field
    if [ "$SERIAL5_TYPE" = "" ] || [ "$SERIAL5_TYPE" = "null" ]; then
        OLD_SERIAL5_PORT=$(echo $DRONE_CFG | jq -r '.serial5_port // empty')
        if [ "$OLD_SERIAL5_PORT" != "" ] && [ "$OLD_SERIAL5_PORT" != "null" ]; then
            SERIAL5_TYPE="tcp"
            SERIAL5_PORT=$OLD_SERIAL5_PORT
        else
            # Default fallback
            SERIAL5_TYPE="tcp"
            SERIAL5_PORT=$((5765 + ID))
        fi
    fi

    # Build Serial5 argument based on type
    case $SERIAL5_TYPE in
        "unix")
            SERIAL5_ARG="--serial5=unix:$SERIAL5_PATH:wait"
            SERIAL5_INFO="Unix socket: $SERIAL5_PATH"
            ;;
        "tcp"|*)
            SERIAL5_ARG="--serial5=tcp:$SERIAL5_PORT:wait"
            SERIAL5_INFO="TCP port: $SERIAL5_PORT"
            ;;
    esac

    LAT=$(echo $DRONE_CFG | jq -r '.initial_position.lat')
    LON=$(echo $DRONE_CFG | jq -r '.initial_position.lon')
    ALT=$(echo $DRONE_CFG | jq -r '.initial_position.alt')
    PARAMS_PATH="./params/copter_$ID.parm"

    # Check if params file exists
    if [ ! -f "$PARAMS_PATH" ]; then
        PARAMS_ARG=""
    else
        PARAMS_ARG="--defaults=$PARAMS_PATH"
    fi

    BASE_PORT=$((5760 + 10 * ID))
    SITL_CORE=$(( (ID * 2) % $(nproc) ))
    COMMAND="taskset -c $SITL_CORE $SITL_BIN -S --model + --speedup 1 --slave 0 --base-port $BASE_PORT $SERIAL5_ARG $PARAMS_ARG --sim-address=127.0.0.1 --home=$LAT,$LON,$ALT,0 -I$ID --disable-fgview"

    if [ $((ID % 20)) -eq 0 ]; then
        echo "   Starting drone $ID/$DRONE_COUNT at ($LAT, $LON, $ALT) on UDP port $UDP_PORT and Serial5 $SERIAL5_INFO"
    fi

    # Run SITL in background, redirect output to log file
    $COMMAND > logs/drone_$ID.log 2>&1 &
    SITL_PIDS+=($!)
done

echo "Started ${#SITL_PIDS[@]} ArduPilot SITL instances"

# Time for initialization
echo "Waiting for SITL initialization..."
sleep 5

# Running MAVProxy for each drone
MAVPROXY_PIDS=()
echo "Starting MAVProxy instances..."

for DRONE_CFG in $DRONES; do
    ID=$(echo $DRONE_CFG | jq -r '.id')
    UDP_PORT=$(echo $DRONE_CFG | jq -r '.udp_port')
    MASTER_PORT=$((5760 + 10 * ID))
    SITL_PORT=$((5501 + ID))

    MAVPROXY_CORE=$(( (ID * 2 + 1) % $(nproc) ))
    MAVPROXY_COMMAND="taskset -c $MAVPROXY_CORE mavproxy.py --daemon --master tcp:127.0.0.1:$MASTER_PORT --sitl 127.0.0.1:$SITL_PORT --out udp:0.0.0.0:$UDP_PORT --out udp:0.0.0.0:$((15000 + ID))"

    if [ $((ID % 20)) -eq 0 ]; then
        echo "   Starting MAVProxy $ID/$DRONE_COUNT (Go Simulator UDP:$UDP_PORT, Mission Planner UDP:$((15000 + ID)))"
    fi

    # Run MAVProxy in background, redirect output to log file
    $MAVPROXY_COMMAND > logs/mavproxy_$ID.log 2>&1 &
    MAVPROXY_PIDS+=($!)
done

echo "Started ${#MAVPROXY_PIDS[@]} MAVProxy instances"

# Time for MAVProxy initialization
echo "Waiting for MAVProxy initialization..."
sleep 5

# Start QEMU ESP32 emulators if in QEMU mode
QEMU_PIDS=()
if [ "$QEMU_MODE" = "true" ]; then
    echo "Starting QEMU ESP32 emulators..."

    # Check if firmware exists
    FIRMWARE_PATH="./qemu/firmware.bin"
    if [ ! -f "$FIRMWARE_PATH" ]; then
        echo "ESP32 firmware not found at $FIRMWARE_PATH"
        echo "Place your ESP32 firmware at $FIRMWARE_PATH"
        echo "Creating placeholder firmware for testing..."
        mkdir -p qemu
        # Create a placeholder firmware file for testing
        dd if=/dev/zero of="$FIRMWARE_PATH" bs=1M count=1 2>/dev/null
        echo "Using placeholder firmware - QEMU may not work correctly"
    else
        echo "ESP32 firmware found at $FIRMWARE_PATH"
    fi

    for DRONE_CFG in $DRONES; do
        ID=$(echo $DRONE_CFG | jq -r '.id')

        # Extract Serial5 configuration for QEMU connection to SITL
        SERIAL5_TYPE=$(echo $DRONE_CFG | jq -r '.serial5.type // empty')
        SERIAL5_PORT=$(echo $DRONE_CFG | jq -r '.serial5.port // empty')
        SERIAL5_PATH=$(echo $DRONE_CFG | jq -r '.serial5.path // empty')

        # Backward compatibility
        if [ "$SERIAL5_TYPE" = "" ] || [ "$SERIAL5_TYPE" = "null" ]; then
            OLD_SERIAL5_PORT=$(echo $DRONE_CFG | jq -r '.serial5_port // empty')
            if [ "$OLD_SERIAL5_PORT" != "" ] && [ "$OLD_SERIAL5_PORT" != "null" ]; then
                SERIAL5_TYPE="tcp"
                SERIAL5_PORT=$OLD_SERIAL5_PORT
            else
                SERIAL5_TYPE="tcp"
                SERIAL5_PORT=$((5765 + ID))
            fi
        fi

        # QEMU network port for Go-simulator communication
        QEMU_NET_PORT=$((6000 + ID))

        # Build QEMU command with network configuration
        case $SERIAL5_TYPE in
            "unix")
                # For Unix sockets, we need to set up QEMU to connect to the socket
                QEMU_SERIAL_ARG="-chardev socket,id=serial5,path=$SERIAL5_PATH -device esp32-uart,chardev=serial5"
                ;;
            "tcp"|*)
                # For TCP, QEMU connects to SITL's TCP server
                QEMU_SERIAL_ARG="-chardev socket,id=serial5,host=127.0.0.1,port=$SERIAL5_PORT -device esp32-uart,chardev=serial5"
                ;;
        esac

        # QEMU command with network forwarding for Go-simulator communication
        CPU_CORE=$((ID % $(nproc)))
        QEMU_COMMAND="taskset -c $CPU_CORE qemu-system-xtensa -nographic -machine esp32 \
-drive file=$FIRMWARE_PATH,if=mtd,format=raw -m 4M \
$QEMU_SERIAL_ARG \
-netdev user,id=net0,hostfwd=tcp::$QEMU_NET_PORT-:80 \
-serial null \
-parallel none \
-monitor none \
-display none \
-vga none \
-soundhw none \
-no-reboot \
-no-shutdown"

        if [ $((ID % 20)) -eq 0 ]; then
            echo "   Starting QEMU ESP32 emulator $ID/$DRONE_COUNT (Network: TCP port $QEMU_NET_PORT)"
        fi

        # Run QEMU in background, redirect output to log file
        $QEMU_COMMAND > logs/qemu_$ID.log 2>&1 &
        QEMU_PIDS+=($!)
    done

    echo "Started ${#QEMU_PIDS[@]} QEMU ESP32 emulators"

    # Time for QEMU initialization
    echo "Waiting for QEMU ESP32 initialization..."
    sleep 10
fi

# Running the Go simulator with the specified mode
echo "Starting Go simulator in $SIM_MODE mode..."
if [ "$QEMU_MODE" = "true" ]; then
    echo "   QEMU ESP32 emulators will handle data routing"
    # In QEMU mode, we need to create a temporary config that points to QEMU ports
    TEMP_CONFIG="config_qemu_temp.json"
    echo "   Creating temporary QEMU config: $TEMP_CONFIG"

    # Create QEMU config by modifying ports to point to QEMU instead of SITL
    jq '.drones |= map(.serial5.port = (6000 + .id))' $CONFIG_FILE > $TEMP_CONFIG
    CONFIG_TO_USE=$TEMP_CONFIG
else
    echo "   Direct connection to SITL instances"
    CONFIG_TO_USE=$CONFIG_FILE
fi

SIMULATOR_CMD="./bin/golang-simulator -config $CONFIG_TO_USE -mode $SIM_MODE"
if [ "$SIM_MODE" = "experiment" ]; then
    echo "   Using scenario: $SCENARIO_FILE"
    SIMULATOR_CMD+=" -scenario $SCENARIO_FILE"
fi

$SIMULATOR_CMD &
SIMULATOR_PID=$!  # Saving the simulator Process PID

echo ""
echo "All systems started in headless mode!"
echo "Go Simulator PID: $SIMULATOR_PID"
echo "MAVProxy PIDs: ${#MAVPROXY_PIDS[@]} instances"
echo "SITL PIDs: ${#SITL_PIDS[@]} instances"
if [ "$QEMU_MODE" = "true" ]; then
    echo "QEMU ESP32 PIDs: ${#QEMU_PIDS[@]} instances"
fi
echo ""
echo "Log files are available in logs/ directory:"
echo "   - drone_*.log (SITL output)"
echo "   - mavproxy_*.log (MAVProxy output)"
if [ "$QEMU_MODE" = "true" ]; then
    echo "   - qemu_*.log (QEMU ESP32 output)"
fi
echo ""
echo "To monitor specific drone, use: tail -f logs/drone_0.log"
echo "To see Go simulator output: check console or redirect to file"
echo ""
echo "Press Ctrl+C to stop all processes..."

# Function for completing all processes
function cleanup() {
    echo ""
    echo "Stopping all processes..."

    # Stop Go simulator first
    if [ ! -z "$SIMULATOR_PID" ]; then
        echo "   Stopping Go simulator..."
        kill -TERM $SIMULATOR_PID 2>/dev/null
        wait $SIMULATOR_PID 2>/dev/null
    fi

    # Stop MAVProxy instances
    echo "   Stopping ${#MAVPROXY_PIDS[@]} MAVProxy instances..."
    for PID in "${MAVPROXY_PIDS[@]}"; do
        kill -TERM $PID 2>/dev/null
    done

    # Stop QEMU ESP32 instances
    if [ "$QEMU_MODE" = "true" ]; then
        echo "   Stopping ${#QEMU_PIDS[@]} QEMU ESP32 instances..."
        for PID in "${QEMU_PIDS[@]}"; do
            kill -TERM $PID 2>/dev/null
        done
    fi

    # Terminating SITL processes
    echo "   Stopping ${#SITL_PIDS[@]} SITL instances..."
    for PID in "${SITL_PIDS[@]}"; do
        kill -TERM $PID 2>/dev/null
    done

    # Kill by process name as backup
    pkill -f "arducopter" 2>/dev/null
    pkill -f "ArduCopter" 2>/dev/null
    pkill -f "mavproxy.py" 2>/dev/null
    if [ "$QEMU_MODE" = "true" ]; then
        pkill -f "qemu-system-xtensa" 2>/dev/null
    fi

    # Give time for graceful termination
    sleep 5

    # Force termination if there is anything left
    if pgrep -f "arducopter|ArduCopter" > /dev/null; then
        echo "   Force killing remaining SITL processes..."
        pkill -9 -f "arducopter" 2>/dev/null
        pkill -9 -f "ArduCopter" 2>/dev/null
    fi

    if pgrep -f "mavproxy.py" > /dev/null; then
        echo "   Force killing MAVProxy..."
        pkill -9 -f "mavproxy.py" 2>/dev/null
    fi

    if [ "$QEMU_MODE" = "true" ] && pgrep -f "qemu-system-xtensa" > /dev/null; then
        echo "   Force killing QEMU ESP32..."
        pkill -9 -f "qemu-system-xtensa" 2>/dev/null
    fi

    # Check that everything is completed
    PROCESS_PATTERN="arducopter|ArduCopter|mavproxy.py|golang-simulator"
    if [ "$QEMU_MODE" = "true" ]; then
        PROCESS_PATTERN="$PROCESS_PATTERN|qemu-system-xtensa"
    fi

    if pgrep -f "$PROCESS_PATTERN" > /dev/null; then
        echo "WARNING: Some processes still running!"
        ps aux | grep -E "$PROCESS_PATTERN" | grep -v grep
    else
        echo "All processes stopped successfully"
    fi

    # Clean up temporary config file
    if [ "$QEMU_MODE" = "true" ] && [ -f "config_qemu_temp.json" ]; then
        rm -f config_qemu_temp.json
        echo "Cleaned up temporary QEMU config"
    fi

    exit 0
}

# Interception of signals for correct termination
trap cleanup SIGINT SIGTERM

# Waiting for the simulator to finish
wait $SIMULATOR_PID

# Termination of all processes after completion of the simulator
cleanup

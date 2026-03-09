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
CONFIG_FILE=${CONFIG_FILE:-"config.json"}
MAVPROXY_IMPL=${MAVPROXY_IMPL:-go}  # python, go
MAVPROXY_TOPOLOGY=${MAVPROXY_TOPOLOGY:-per-instance}  # per-instance, single-process

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

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
    echo "Environment:"
    echo "  CONFIG_FILE=path"
    echo "  MAVPROXY_IMPL=python|go"
    echo "  MAVPROXY_TOPOLOGY=per-instance|single-process"
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
MAVPROXY_SESSION_CONFIG=""

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
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Configuration file $CONFIG_FILE not found"
    exit 1
fi

DRONES=$(jq -c '.drones[]' "$CONFIG_FILE")
if [ $? -ne 0 ]; then
    echo "Failed to parse configuration file"
    exit 1
fi

DRONE_COUNT=$(echo "$DRONES" | wc -l)
echo "Configuration loaded: $DRONE_COUNT drones"
echo "Bridge implementation: $MAVPROXY_IMPL"
echo "Bridge topology: $MAVPROXY_TOPOLOGY"

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

# Real mode - start ArduPilot SITL and bridge
echo "HEADLESS MODE - Starting ArduPilot SITL instances in background..."

# Create directories for logs
mkdir -p logs

# Running SITL for each drone
SITL_PIDS=()
echo "Starting SITL instances..."
for DRONE_CFG in $DRONES; do
    ID=$(echo $DRONE_CFG | jq -r '.id')
    UDP_PORT=$(echo $DRONE_CFG | jq -r '.udp_port')

    SERIAL5_TYPE=$(echo $DRONE_CFG | jq -r '.serial5.type // empty')
    SERIAL5_PORT=$(echo $DRONE_CFG | jq -r '.serial5.port // empty')
    SERIAL5_PATH=$(echo $DRONE_CFG | jq -r '.serial5.path // empty')

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

    $COMMAND > logs/drone_$ID.log 2>&1 &
    SITL_PIDS+=($!)
done

echo "Started ${#SITL_PIDS[@]} ArduPilot SITL instances"

# Time for initialization
echo "Waiting for SITL initialization..."
sleep 5

# Running bridge instances
MAVPROXY_PIDS=()

if [ "$MAVPROXY_IMPL" = "go" ] && [ "$MAVPROXY_TOPOLOGY" = "single-process" ]; then
    echo "Starting single go-mavproxy instance for all drones..."
    MAVPROXY_SESSION_CONFIG="logs/go_mavproxy_sessions.json"
    printf '[\n' > "$MAVPROXY_SESSION_CONFIG"
    FIRST=1

    for DRONE_CFG in $DRONES; do
        ID=$(echo $DRONE_CFG | jq -r '.id')
        UDP_PORT=$(echo $DRONE_CFG | jq -r '.udp_port')
        MASTER_PORT=$((5760 + 10 * ID))
        SITL_PORT=$((5501 + ID))

        if [ $FIRST -eq 0 ]; then
            printf ',\n' >> "$MAVPROXY_SESSION_CONFIG"
        fi
        FIRST=0

        cat >> "$MAVPROXY_SESSION_CONFIG" <<JSON
  {
    "name": "drone_$ID",
    "dialect": "ardupilotmega",
    "streamrate": 10,
    "continue_on_disconnect": true,
    "masters": ["tcp://127.0.0.1:$MASTER_PORT"],
    "outs": ["udp:0.0.0.0:$UDP_PORT", "udp:0.0.0.0:$((15000 + ID))"],
    "sitl": "127.0.0.1:$SITL_PORT",
    "app_logfile": "logs/mavproxy_$ID.app.log",
    "message_logfile": "logs/mavproxy_$ID.messages.log",
    "logfile": "logs/mavproxy_$ID.tlog",
    "loglevel": "debug"
  }
JSON
    done
    printf '\n]\n' >> "$MAVPROXY_SESSION_CONFIG"

    MAVPROXY_CORE=1
    MAVPROXY_COMMAND="taskset -c $MAVPROXY_CORE ./bin/go-mavproxy --session-config $MAVPROXY_SESSION_CONFIG --loglevel debug"
    echo "   Command: $MAVPROXY_COMMAND"
    $MAVPROXY_COMMAND > logs/mavproxy_single.log 2>&1 &
    MAVPROXY_PIDS+=($!)
else
    echo "Starting bridge instances..."
    for DRONE_CFG in $DRONES; do
        ID=$(echo $DRONE_CFG | jq -r '.id')
        UDP_PORT=$(echo $DRONE_CFG | jq -r '.udp_port')
        MASTER_PORT=$((5760 + 10 * ID))
        SITL_PORT=$((5501 + ID))

        MAVPROXY_CORE=$(( (ID * 2 + 1) % $(nproc) ))
        if [ "$MAVPROXY_IMPL" = "go" ]; then
            MAVPROXY_COMMAND="taskset -c $MAVPROXY_CORE ./bin/go-mavproxy --master tcp://127.0.0.1:$MASTER_PORT --sitl 127.0.0.1:$SITL_PORT --out udp:0.0.0.0:$UDP_PORT --out udp:0.0.0.0:$((15000 + ID)) --dialect ardupilotmega --streamrate 10 --loglevel debug --app-logfile logs/mavproxy_$ID.app.log --message-logfile logs/mavproxy_$ID.messages.log --logfile logs/mavproxy_$ID.tlog"
        else
            MAVPROXY_COMMAND="taskset -c $MAVPROXY_CORE mavproxy.py --daemon --master tcp:127.0.0.1:$MASTER_PORT --sitl 127.0.0.1:$SITL_PORT --out udp:0.0.0.0:$UDP_PORT --out udp:0.0.0.0:$((15000 + ID))"
        fi

        if [ $((ID % 20)) -eq 0 ]; then
            echo "   Starting bridge $ID/$DRONE_COUNT ($MAVPROXY_IMPL/$MAVPROXY_TOPOLOGY)"
        fi

        $MAVPROXY_COMMAND > logs/mavproxy_$ID.log 2>&1 &
        MAVPROXY_PIDS+=($!)
    done
fi

echo "Started ${#MAVPROXY_PIDS[@]} bridge instances"

# Time for bridge initialization
echo "Waiting for bridge initialization..."
sleep 5

# Start QEMU ESP32 emulators if in QEMU mode
QEMU_PIDS=()
if [ "$QEMU_MODE" = "true" ]; then
    echo "Starting QEMU ESP32 emulators..."

    FIRMWARE_PATH="./qemu/firmware.bin"
    if [ ! -f "$FIRMWARE_PATH" ]; then
        echo "ESP32 firmware not found at $FIRMWARE_PATH"
        echo "Place your ESP32 firmware at $FIRMWARE_PATH"
        echo "Creating placeholder firmware for testing..."
        mkdir -p qemu
        dd if=/dev/zero of="$FIRMWARE_PATH" bs=1M count=1 2>/dev/null
        echo "Using placeholder firmware - QEMU may not work correctly"
    else
        echo "ESP32 firmware found at $FIRMWARE_PATH"
    fi

    for DRONE_CFG in $DRONES; do
        ID=$(echo $DRONE_CFG | jq -r '.id')
        SERIAL5_TYPE=$(echo $DRONE_CFG | jq -r '.serial5.type // empty')
        SERIAL5_PORT=$(echo $DRONE_CFG | jq -r '.serial5.port // empty')
        SERIAL5_PATH=$(echo $DRONE_CFG | jq -r '.serial5.path // empty')

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

        QEMU_NET_PORT=$((6000 + ID))

        case $SERIAL5_TYPE in
            "unix")
                QEMU_SERIAL_ARG="-chardev socket,id=serial5,path=$SERIAL5_PATH -device esp32-uart,chardev=serial5"
                ;;
            "tcp"|*)
                QEMU_SERIAL_ARG="-chardev socket,id=serial5,host=127.0.0.1,port=$SERIAL5_PORT -device esp32-uart,chardev=serial5"
                ;;
        esac

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

        $QEMU_COMMAND > logs/qemu_$ID.log 2>&1 &
        QEMU_PIDS+=($!)
    done

    echo "Started ${#QEMU_PIDS[@]} QEMU ESP32 emulators"
    echo "Waiting for QEMU ESP32 initialization..."
    sleep 10
fi

# Running the Go simulator with the specified mode
echo "Starting Go simulator in $SIM_MODE mode..."
if [ "$QEMU_MODE" = "true" ]; then
    echo "   QEMU ESP32 emulators will handle data routing"
    TEMP_CONFIG="config_qemu_temp.json"
    echo "   Creating temporary QEMU config: $TEMP_CONFIG"
    jq '.drones |= map(.serial5.port = (6000 + .id))' "$CONFIG_FILE" > "$TEMP_CONFIG"
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
SIMULATOR_PID=$!

echo ""
echo "All systems started in headless mode!"
echo "Go Simulator PID: $SIMULATOR_PID"
echo "Bridge PIDs: ${#MAVPROXY_PIDS[@]} instances"
echo "SITL PIDs: ${#SITL_PIDS[@]} instances"
if [ "$QEMU_MODE" = "true" ]; then
    echo "QEMU ESP32 PIDs: ${#QEMU_PIDS[@]} instances"
fi
echo ""
echo "Log files are available in logs/ directory:"
echo "   - drone_*.log (SITL output)"
echo "   - mavproxy_*.log (bridge output)"
if [ "$QEMU_MODE" = "true" ]; then
    echo "   - qemu_*.log (QEMU ESP32 output)"
fi
if [ -n "$MAVPROXY_SESSION_CONFIG" ]; then
    echo "   - mavproxy_single.log (single-process bridge output)"
fi
echo ""
echo "To monitor specific drone, use: tail -f logs/drone_0.log"
echo "Press Ctrl+C to stop all processes..."

function cleanup() {
    echo ""
    echo "Stopping all processes..."

    if [ ! -z "$SIMULATOR_PID" ]; then
        echo "   Stopping Go simulator..."
        kill -TERM $SIMULATOR_PID 2>/dev/null
        wait $SIMULATOR_PID 2>/dev/null
    fi

    echo "   Stopping ${#MAVPROXY_PIDS[@]} bridge instances..."
    for PID in "${MAVPROXY_PIDS[@]}"; do
        kill -TERM $PID 2>/dev/null
    done

    if [ "$QEMU_MODE" = "true" ]; then
        echo "   Stopping ${#QEMU_PIDS[@]} QEMU ESP32 instances..."
        for PID in "${QEMU_PIDS[@]}"; do
            kill -TERM $PID 2>/dev/null
        done
    fi

    echo "   Stopping ${#SITL_PIDS[@]} SITL instances..."
    for PID in "${SITL_PIDS[@]}"; do
        kill -TERM $PID 2>/dev/null
    done

    pkill -f "arducopter" 2>/dev/null
    pkill -f "ArduCopter" 2>/dev/null
    pkill -f "mavproxy.py" 2>/dev/null
    pkill -f "go-mavproxy" 2>/dev/null
    if [ "$QEMU_MODE" = "true" ]; then
        pkill -f "qemu-system-xtensa" 2>/dev/null
    fi

    sleep 5

    if pgrep -f "arducopter|ArduCopter" > /dev/null; then
        echo "   Force killing remaining SITL processes..."
        pkill -9 -f "arducopter" 2>/dev/null
        pkill -9 -f "ArduCopter" 2>/dev/null
    fi

    if pgrep -f "mavproxy.py|go-mavproxy" > /dev/null; then
        echo "   Force killing bridge processes..."
        pkill -9 -f "mavproxy.py" 2>/dev/null
        pkill -9 -f "go-mavproxy" 2>/dev/null
    fi

    if [ "$QEMU_MODE" = "true" ] && pgrep -f "qemu-system-xtensa" > /dev/null; then
        echo "   Force killing QEMU ESP32..."
        pkill -9 -f "qemu-system-xtensa" 2>/dev/null
    fi

    PROCESS_PATTERN="arducopter|ArduCopter|mavproxy.py|go-mavproxy|golang-simulator"
    if [ "$QEMU_MODE" = "true" ]; then
        PROCESS_PATTERN="$PROCESS_PATTERN|qemu-system-xtensa"
    fi

    if pgrep -f "$PROCESS_PATTERN" > /dev/null; then
        echo "WARNING: Some processes still running!"
        ps aux | grep -E "$PROCESS_PATTERN" | grep -v grep
    else
        echo "All processes stopped successfully"
    fi

    if [ "$QEMU_MODE" = "true" ] && [ -f "config_qemu_temp.json" ]; then
        rm -f config_qemu_temp.json
        echo "Cleaned up temporary QEMU config"
    fi

    if [ -n "$MAVPROXY_SESSION_CONFIG" ] && [ -f "$MAVPROXY_SESSION_CONFIG" ]; then
        rm -f "$MAVPROXY_SESSION_CONFIG"
    fi

    exit 0
}

trap cleanup SIGINT SIGTERM
wait $SIMULATOR_PID
cleanup

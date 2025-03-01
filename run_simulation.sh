#!/bin/bash

export PATH=$PATH:/home/kiselyovvld/.local/bin
export PATH=$PATH:/mnt/c/Users/kisel/CLionProjects/ardupilot-swarming/Tools/autotest
export PATH=$PATH:/mnt/c/Users/kisel/PycharmProjects/swarm-simulation

SIM_VEHICLE="sim_vehicle.py"
RUN="run_in_terminal_window.sh"
SITL_BIN="/mnt/c/Users/kisel/CLionProjects/ardupilot-swarming/build/sitl/bin/arducopter"

SITL_PIDS=()

# Loading the configuration
CONFIG_FILE="config.json"
DRONES=$(jq -c '.drones[]' $CONFIG_FILE)

# Running SITL for each drone
for DRONE_CFG in $DRONES; do
    ID=$(echo $DRONE_CFG | jq -r '.id')
    SIM_PORT=$((5700 + 10 * ID))
    UDP_PORT=$(echo $DRONE_CFG | jq -r '.udp_port')
    SERIAL5_PORT=$(echo $DRONE_CFG | jq -r '.serial5_port')
    LAT=$(echo $DRONE_CFG | jq -r '.initial_position.lat')
    LON=$(echo $DRONE_CFG | jq -r '.initial_position.lon')
    ALT=$(echo $DRONE_CFG | jq -r '.initial_position.alt')
    PARAMS_PATH="/mnt/c/Users/kisel/PycharmProjects/swarm-simulation/params/copter_$ID.parm"
    COMMAND="$RUN ArduCopter $SITL_BIN -S --model + --speedup 1 --slave 0 --serial5=tcp:$SERIAL5_PORT:wait --defaults=$PARAMS_PATH --sim-address=127.0.0.1 --home=$LAT,$LON,$ALT,0 -I$ID"
#    COMMAND="$SIM_VEHICLE -N -v ArduCopter -I $ID --out=udp:0.0.0.0:$UDP_PORT --console -l $LAT,$LON,$ALT,0 --add-param-file=$PARAMS_PATH -A --serial5=tcp:$SERIAL5_PORT:wait"

    echo "Starting drone $ID at ($LAT, $LON, $ALT) on UDP port $UDP_PORT and SERIAL5 TCP port $SERIAL5_PORT ..."
    echo "Run $COMMAND ..."
    xterm -hold -e "$COMMAND 2>&1 | tee /tmp/drone_$ID.log" &
    SITL_PIDS+=($!)  # Сохранение PID процесса
done

# Time for initialization
sleep 10

# Running the proxy
#MAVPROXY_COMMAND="mavproxy.py  --out 172.28.0.1:14550 --out 172.28.0.1:14551    --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out udp:0.0.0.0:14550    --master tcp:127.0.0.1:5770   --sitl 127.0.0.1:5502   --out udp:0.0.0.0:14560    --console"
# Запуск прокси с динамическими параметрами для всех дронов
MAVPROXY_COMMAND="mavproxy.py --out 172.28.0.1:14550 --out 172.28.0.1:14551"

for DRONE_CFG in $DRONES; do
    ID=$(echo $DRONE_CFG | jq -r '.id')
    MASTER_PORT=$((5760 + 10 * ID))
    SITL_PORT=$((5501 + ID))
    OUT_PORT=$((14500 + 10 * ID))
    MAVPROXY_COMMAND+=" --master tcp:127.0.0.1:$MASTER_PORT --sitl 127.0.0.1:$SITL_PORT --out udp:0.0.0.0:$OUT_PORT"
done
echo "Run $MAVPROXY_COMMAND ..."
#MAVPROXY_COMMAND+=" --out udp:0.0.0.0:14550 --out udp:0.0.0.0:14560 --console"

xterm -hold -e "$MAVPROXY_COMMAND 2>&1 | tee /tmp/mavproxy.log" &
MAVPROXY_PID=$!  # Сохранение PID процесса MAVProxy

# Time for initialization
sleep 5

# Running the simulator
python3 app/simulator.py &
SIMULATOR_PID=$!  # Сохранение PID процесса симулятора

# Функция для завершения всех процессов
cleanup() {
    echo "Stopping all processes..."
    # Завершение процессов SITL
    for PID in "${SITL_PIDS[@]}"; do
        kill -TERM $PID 2>/dev/null
    done
    kill -TERM $MAVPROXY_PID 2>/dev/null
    kill -TERM $SIMULATOR_PID 2>/dev/null
    echo "All processes stopped."
    exit 0
}

# Перехват сигналов для корректного завершения
trap cleanup SIGINT SIGTERM

# Ожидание завершения симулятора
wait $SIMULATOR_PID

# Завершение всех процессов после завершения симулятора
cleanup

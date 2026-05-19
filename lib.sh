# shellcheck shell=bash
# Swarm simulation launcher — shared library.
# Sourced by run_swarm_simulation.sh; requires SWARM_SIM_ROOT and cwd = repo root.
#
# Behavioral parity (see also run_golang_simulation*.sh, run_golang_single.sh):
# - terminal: xterm for processes; headless: logs under logs/
# - MAVPROXY_IMPL=go|python, MAVPROXY_TOPOLOGY=per-instance|single-process
# - python + single-process ~= one mavproxy.py with all masters (old run_golang_single bridge);
#   SITL here always uses --base-port (run_golang_single.sh used RITW without --base-port — use that script if you need an exact match)

if [[ -n "${_SWARM_SIM_LIB_LOADED:-}" ]]; then
  return 0 2>/dev/null || exit 0
fi
_SWARM_SIM_LIB_LOADED=1

ssim_usage() {
  local me
  me="${SSIM_ME:-run_swarm_simulation.sh}"
  cat <<EOF
Swarm simulator (unified launcher)

Usage:
  $me [options] [SIM_MODE [CONNECTION_MODE [SCENARIO_FILE]]]

Positional:
  SIM_MODE          experiment | gui   (default: experiment)
  CONNECTION_MODE   direct | qemu      (default: direct)
  SCENARIO_FILE     scenario path      (default: scenarios/simple_flight.yaml)

Options:
  -h, --help              Show this help
      --headless          Run SITL/MAVProxy/QEMU/simulator without xterm (logs in logs/)
      --terminal          Force xterm windows (default)

Environment:
  CONFIG_FILE           Path to drone JSON (default: config.json)
  MAVPROXY_IMPL         python | go (default: go)
  MAVPROXY_TOPOLOGY     per-instance | single-process (default: per-instance)
                          go + single-process     -> one go-mavproxy (session JSON)
                          python + single-process -> one mavproxy.py, all masters
  SITL_BIN              Override SITL binary (default: ./sitl/arducopter.bin)
  RUN_SWARM_APPEND_PATH If set to 0, skip PATH additions for ~/.local/bin and /usr/local/go/bin
  MAVPROXY_NO_STATE     If 1, pass --no-state to mavproxy.py (no mav.tlog / .raw on disk; less state)
  MAVPROXY_GO_NO_LOG    If 1, disable go-mavproxy file logs (no .tlog / .app.log / .messages.log;
                          loglevel error; headless stdout -> /dev/null)
  MAVPROXY_PY_EXTRA     Extra arguments appended to every mavproxy.py command (quoted string)

Python MAVProxy writes mav.tlog, mav.tlog.raw, mav*.parm in cwd unless redirected; this script
defaults to --state-basedir=logs and per-process --logfile.
EOF
}

ssim_append_path_defaults() {
  if [[ "${RUN_SWARM_APPEND_PATH:-1}" == "0" ]]; then
    return 0
  fi
  if [[ -n "${HOME:-}" && -d "${HOME}/.local/bin" ]]; then
    export PATH="${PATH}:${HOME}/.local/bin"
  fi
  if [[ -d "/usr/local/go/bin" ]]; then
    export PATH="${PATH}:/usr/local/go/bin"
  fi
}

ssim_build_go_simulator() {
  echo "Building Go simulator..."
  (cd golang_app || exit 1
  if ! go build -o ../bin/golang-simulator ./cmd/simulator; then
    echo "Failed to build Go simulator"
    return 1
  fi) || return 1
  echo "Go simulator built successfully"
  return 0
}

ssim_load_drones_from_config() {
  local cfg="$1"
  if [[ ! -f "$cfg" ]]; then
    echo "Configuration file $cfg not found"
    return 1
  fi
  DRONES=$(jq -c '.drones[]' "$cfg") || {
    echo "Failed to parse configuration file"
    return 1
  }
  DRONE_COUNT=$(echo "$DRONES" | wc -l)
  return 0
}

ssim_validate_connection_mode() {
  if [[ ! "$CONNECTION_MODE" =~ ^(direct|qemu)$ ]]; then
    echo "Invalid CONNECTION_MODE: $CONNECTION_MODE"
    echo "Valid options: direct, qemu"
    echo "Use --help for more information"
    return 1
  fi
  case "$CONNECTION_MODE" in
    qemu)
      echo "QEMU MODE - Starting with ESP32 emulation layer"
      QEMU_MODE=true
      ;;
    *)
      echo "DIRECT MODE - Direct ArduPilot SITL connection"
      QEMU_MODE=false
      ;;
  esac
  return 0
}

# Resolve SERIAL5_* from one drone JSON line (mutates globals SERIAL5_TYPE, SERIAL5_PORT, SERIAL5_PATH)
ssim_drone_serial5_from_cfg() {
  local DRONE_CFG="$1"
  SERIAL5_TYPE=$(echo "$DRONE_CFG" | jq -r '.serial5.type // empty')
  SERIAL5_PORT=$(echo "$DRONE_CFG" | jq -r '.serial5.port // empty')
  SERIAL5_PATH=$(echo "$DRONE_CFG" | jq -r '.serial5.path // empty')
  if [[ "$SERIAL5_TYPE" == "" || "$SERIAL5_TYPE" == "null" ]]; then
    local OLD_SERIAL5_PORT
    OLD_SERIAL5_PORT=$(echo "$DRONE_CFG" | jq -r '.serial5_port // empty')
    if [[ "$OLD_SERIAL5_PORT" != "" && "$OLD_SERIAL5_PORT" != "null" ]]; then
      SERIAL5_TYPE="tcp"
      SERIAL5_PORT=$OLD_SERIAL5_PORT
    else
      SERIAL5_TYPE="tcp"
      SERIAL5_PORT=$((5765 + ID))
    fi
  fi
}

ssim_serial5_args() {
  case "$SERIAL5_TYPE" in
    unix)
      SERIAL5_ARG="--serial5=unix:$SERIAL5_PATH:wait"
      SERIAL5_INFO="Unix socket: $SERIAL5_PATH"
      ;;
    *)
      SERIAL5_ARG="--serial5=tcp:$SERIAL5_PORT:wait"
      SERIAL5_INFO="TCP port: $SERIAL5_PORT"
      ;;
  esac
}

# MAVProxy (Python) defaults to mav.tlog + mav*.parm in cwd. Use logs/ unless MAVPROXY_NO_STATE=1.
ssim_mavproxy_python_disk_flags() {
  local log_basename="$1"
  if [[ "${MAVPROXY_NO_STATE:-0}" == "1" ]]; then
    printf '%s' "--no-state"
  else
    printf '%s' "--state-basedir=logs --logfile ${log_basename}.tlog"
  fi
}

ssim_go_mavproxy_no_log() {
  [[ "${MAVPROXY_GO_NO_LOG:-0}" == "1" ]]
}

ssim_go_mavproxy_loglevel() {
  if ssim_go_mavproxy_no_log; then
    echo "error"
  else
    echo "debug"
  fi
}

# Extra CLI flags for per-instance go-mavproxy (empty when logging disabled).
ssim_go_mavproxy_file_log_flags() {
  local id=$1
  if ssim_go_mavproxy_no_log; then
    return 0
  fi
  printf '%s' \
    "--app-logfile logs/mavproxy_${id}.app.log --message-logfile logs/mavproxy_${id}.messages.log --logfile logs/mavproxy_${id}.tlog"
}

# Where to redirect go-mavproxy stdout/stderr in headless mode.
ssim_go_mavproxy_stdout_target() {
  local id=$1
  if ssim_go_mavproxy_no_log; then
    echo "/dev/null"
  elif [[ "$id" == "single" ]]; then
    echo "logs/mavproxy_single.log"
  else
    echo "logs/mavproxy_${id}.log"
  fi
}

# Second MAVLink out for Mission Planner / GCS
ssim_mp_second_out() {
  local id="$1"
  if [[ "$SSIM_HEADLESS" -eq 1 ]]; then
    echo "udp:0.0.0.0:$((15000 + id))"
  else
    echo "udp:172.28.0.1:$((15000 + id))"
  fi
}

ssim_resolve_sitl_bin() {
  if [[ -n "${SITL_BIN:-}" ]]; then
    return 0
  fi
  SITL_BIN="./sitl/arducopter.bin"
}

# --- SITL ---
ssim_start_sitl_instances() {
  SITL_PIDS=()
  echo "Starting ArduPilot SITL instances..."
  mkdir -p logs

  for DRONE_CFG in $DRONES; do
    ID=$(echo "$DRONE_CFG" | jq -r '.id')
    UDP_PORT=$(echo "$DRONE_CFG" | jq -r '.udp_port')
    ssim_drone_serial5_from_cfg "$DRONE_CFG"
    ssim_serial5_args

    LAT=$(echo "$DRONE_CFG" | jq -r '.initial_position.lat')
    LON=$(echo "$DRONE_CFG" | jq -r '.initial_position.lon')
    ALT=$(echo "$DRONE_CFG" | jq -r '.initial_position.alt')
    PARAMS_PATH="./params/copter_${ID}.parm"
    if [[ ! -f "$PARAMS_PATH" ]]; then
      echo "Params file $PARAMS_PATH not found, using default"
      PARAMS_ARG=""
    else
      PARAMS_ARG="--defaults=$PARAMS_PATH"
    fi

    SITL_CORE=$(( (ID * 2) % $(nproc) ))
    BASE_PORT=$((5760 + 10 * ID))
    COMMAND="taskset -c $SITL_CORE $SITL_BIN -S --model + --speedup 1 --slave 0 --base-port $BASE_PORT $SERIAL5_ARG $PARAMS_ARG --sim-address=127.0.0.1 --home=$LAT,$LON,$ALT,0 -I$ID --disable-fgview"

    if [[ $((ID % 20)) -eq 0 ]]; then
      echo "   Starting drone $ID/$DRONE_COUNT at ($LAT, $LON, $ALT) on UDP port $UDP_PORT and Serial5 $SERIAL5_INFO ..."
      echo "   Command: $COMMAND"
    fi

    if [[ "$SSIM_HEADLESS" -eq 1 ]]; then
      # shellcheck disable=SC2086
      $COMMAND >"logs/drone_${ID}.log" 2>&1 &
      SITL_PIDS+=($!)
    else
      xterm -title "Drone $ID SITL" -hold -e bash -lc "$COMMAND 2>&1 | tee logs/drone_${ID}.log" &
      SITL_PIDS+=($!)
    fi

  done
  echo "Started ${#SITL_PIDS[@]} ArduPilot SITL instances"
  echo "Waiting 5 seconds for SITL initialization..."
  sleep 5
}

# --- MAVProxy / go-mavproxy ---
ssim_start_mavproxy_default() {
  MAVPROXY_PIDS=()
  MAVPROXY_SESSION_CONFIG=""

  if [[ "$MAVPROXY_IMPL" == "go" && "$MAVPROXY_TOPOLOGY" == "single-process" ]]; then
    echo "Starting single go-mavproxy instance for all drones..."
    MAVPROXY_SESSION_CONFIG="logs/go_mavproxy_sessions.json"
    printf '[\n' >"$MAVPROXY_SESSION_CONFIG"
    local FIRST=1
    for DRONE_CFG in $DRONES; do
      ID=$(echo "$DRONE_CFG" | jq -r '.id')
      UDP_PORT=$(echo "$DRONE_CFG" | jq -r '.udp_port')
      MASTER_PORT=$((5760 + 10 * ID))
      SITL_PORT=$((5501 + ID))
      local OUT_SECOND
      OUT_SECOND=$(ssim_mp_second_out "$ID")
      if [[ $FIRST -eq 0 ]]; then
        printf ',\n' >>"$MAVPROXY_SESSION_CONFIG"
      fi
      FIRST=0
      local GO_LOG_LEVEL
      GO_LOG_LEVEL=$(ssim_go_mavproxy_loglevel)
      if ssim_go_mavproxy_no_log; then
        cat >>"$MAVPROXY_SESSION_CONFIG" <<JSON
  {
    "name": "drone_$ID",
    "dialect": "ardupilotmega",
    "streamrate": 10,
    "continue_on_disconnect": true,
    "masters": ["tcp://127.0.0.1:$MASTER_PORT"],
    "outs": ["udp:0.0.0.0:$UDP_PORT", "$OUT_SECOND"],
    "sitl": "127.0.0.1:$SITL_PORT",
    "loglevel": "$GO_LOG_LEVEL"
  }
JSON
      else
        cat >>"$MAVPROXY_SESSION_CONFIG" <<JSON
  {
    "name": "drone_$ID",
    "dialect": "ardupilotmega",
    "streamrate": 10,
    "continue_on_disconnect": true,
    "masters": ["tcp://127.0.0.1:$MASTER_PORT"],
    "outs": ["udp:0.0.0.0:$UDP_PORT", "$OUT_SECOND"],
    "sitl": "127.0.0.1:$SITL_PORT",
    "app_logfile": "logs/mavproxy_${ID}.app.log",
    "message_logfile": "logs/mavproxy_${ID}.messages.log",
    "logfile": "logs/mavproxy_${ID}.tlog",
    "loglevel": "$GO_LOG_LEVEL"
  }
JSON
      fi
    done
    printf '\n]\n' >>"$MAVPROXY_SESSION_CONFIG"

    MAVPROXY_CORE=1
    local GO_STDOUT
    GO_STDOUT=$(ssim_go_mavproxy_stdout_target single)
    MAVPROXY_COMMAND="taskset -c $MAVPROXY_CORE ./bin/go-mavproxy --session-config $MAVPROXY_SESSION_CONFIG --loglevel $GO_LOG_LEVEL"
    if ssim_go_mavproxy_no_log; then
      echo "   go-mavproxy file logging disabled (MAVPROXY_GO_NO_LOG=1)"
    fi
    echo "   Command: $MAVPROXY_COMMAND"
    if [[ "$SSIM_HEADLESS" -eq 1 ]]; then
      # shellcheck disable=SC2086
      $MAVPROXY_COMMAND >"$GO_STDOUT" 2>&1 &
      MAVPROXY_PIDS+=($!)
    else
      if ssim_go_mavproxy_no_log; then
        xterm -title "go-mavproxy (single)" -hold -e "$MAVPROXY_COMMAND" &
      else
        xterm -title "go-mavproxy (single)" -hold -e "$MAVPROXY_COMMAND 2>&1 | tee logs/mavproxy_single.log" &
      fi
      MAVPROXY_PIDS+=($!)
    fi
  elif [[ "$MAVPROXY_IMPL" == "python" && "$MAVPROXY_TOPOLOGY" == "single-process" ]]; then
    echo "Starting single mavproxy.py instance for all drones..."
    MAVPROXY_SESSION_CONFIG=""
    local MAVP_DISK
    MAVP_DISK=$(ssim_mavproxy_python_disk_flags mavproxy_single)
    local MAVPROXY_BASE_COMMAND="mavproxy.py ${MAVP_DISK} ${MAVPROXY_PY_EXTRA:-} --out 172.28.0.1:14550 --out 172.28.0.1:14551"
    local MAVPROXY_LINKS=""
    for DRONE_CFG in $DRONES; do
      ID=$(echo "$DRONE_CFG" | jq -r '.id')
      UDP_PORT=$(echo "$DRONE_CFG" | jq -r '.udp_port')
      MASTER_PORT=$((5760 + 10 * ID))
      SITL_PORT=$((5501 + ID))
      MAVPROXY_LINKS+=" --master tcp:127.0.0.1:$MASTER_PORT --sitl 127.0.0.1:$SITL_PORT --out udp:0.0.0.0:$UDP_PORT"
    done
    MAVPROXY_CORE=1
    if [[ "$SSIM_HEADLESS" -eq 1 ]]; then
      FULL_MAVPROXY_COMMAND="taskset -c $MAVPROXY_CORE mavproxy.py ${MAVP_DISK} ${MAVPROXY_PY_EXTRA:-} --daemon --out 172.28.0.1:14550 --out 172.28.0.1:14551$MAVPROXY_LINKS"
    else
      FULL_MAVPROXY_COMMAND="taskset -c $MAVPROXY_CORE $MAVPROXY_BASE_COMMAND$MAVPROXY_LINKS"
    fi
    echo "   Command: $FULL_MAVPROXY_COMMAND"
    echo "   Go simulator uses UDP ports from $CONFIG_FILE; Mission Planner: UDP 172.28.0.1:14550 or 14551"
    if [[ "$SSIM_HEADLESS" -eq 1 ]]; then
      # shellcheck disable=SC2086
      $FULL_MAVPROXY_COMMAND >logs/mavproxy_single.log 2>&1 &
      MAVPROXY_PIDS+=($!)
    else
      xterm -title "MAVProxy (single)" -hold -e "$FULL_MAVPROXY_COMMAND 2>&1 | tee logs/mavproxy_single.log" &
      MAVPROXY_PIDS+=($!)
    fi
  else
    echo "Starting bridge instances..."
    for DRONE_CFG in $DRONES; do
      ID=$(echo "$DRONE_CFG" | jq -r '.id')
      UDP_PORT=$(echo "$DRONE_CFG" | jq -r '.udp_port')
      MASTER_PORT=$((5760 + 10 * ID))
      SITL_PORT=$((5501 + ID))
      MP_PORT=$((15000 + ID))
      MAVPROXY_CORE=$(( (ID * 2 + 1) % $(nproc) ))
      local OUT_SECOND
      OUT_SECOND=$(ssim_mp_second_out "$ID")

      if [[ "$MAVPROXY_IMPL" == "go" ]]; then
        local GO_LOG_LEVEL GO_FILE_LOGS GO_STDOUT
        GO_LOG_LEVEL=$(ssim_go_mavproxy_loglevel)
        GO_FILE_LOGS=$(ssim_go_mavproxy_file_log_flags "$ID")
        GO_STDOUT=$(ssim_go_mavproxy_stdout_target "$ID")
        MAVPROXY_COMMAND="taskset -c $MAVPROXY_CORE ./bin/go-mavproxy --master tcp://127.0.0.1:$MASTER_PORT --sitl 127.0.0.1:$SITL_PORT --out udp:0.0.0.0:$UDP_PORT --out $OUT_SECOND --dialect ardupilotmega --streamrate 10 --loglevel $GO_LOG_LEVEL"
        if [[ -n "$GO_FILE_LOGS" ]]; then
          MAVPROXY_COMMAND+=" $GO_FILE_LOGS"
        fi
        BRIDGE_TITLE="go-mavproxy $ID"
      else
        local MAVP_DISK
        MAVP_DISK=$(ssim_mavproxy_python_disk_flags "mavproxy_${ID}")
        MAVPROXY_COMMAND="taskset -c $MAVPROXY_CORE mavproxy.py ${MAVP_DISK} ${MAVPROXY_PY_EXTRA:-} --master tcp:127.0.0.1:$MASTER_PORT --sitl 127.0.0.1:$SITL_PORT --out udp:0.0.0.0:$UDP_PORT"
        if [[ "$SSIM_HEADLESS" -eq 1 ]]; then
          MAVPROXY_COMMAND+=" --daemon --out udp:0.0.0.0:$MP_PORT"
        else
          MAVPROXY_COMMAND+=" --out 172.28.0.1:$MP_PORT"
        fi
        BRIDGE_TITLE="MAVProxy $ID"
      fi

      echo "   Command: $MAVPROXY_COMMAND"
      if [[ "$SSIM_HEADLESS" -eq 1 ]]; then
        if [[ $((ID % 20)) -eq 0 ]]; then
          echo "   Starting bridge $ID/$DRONE_COUNT ($MAVPROXY_IMPL/$MAVPROXY_TOPOLOGY)"
          echo "   Command: $MAVPROXY_COMMAND"
        fi
        if [[ "$MAVPROXY_IMPL" == "go" ]]; then
          # shellcheck disable=SC2086
          $MAVPROXY_COMMAND >"$GO_STDOUT" 2>&1 &
        else
          # shellcheck disable=SC2086
          $MAVPROXY_COMMAND >"logs/mavproxy_${ID}.log" 2>&1 &
        fi
        MAVPROXY_PIDS+=($!)
      else
        echo "   Go simulator uses UDP ports from $CONFIG_FILE; Mission Planner: UDP 172.28.0.1:$MP_PORT"
        if [[ "$MAVPROXY_IMPL" == "go" ]] && ssim_go_mavproxy_no_log; then
          xterm -title "$BRIDGE_TITLE" -hold -e "$MAVPROXY_COMMAND" &
        else
          xterm -title "$BRIDGE_TITLE" -hold -e "$MAVPROXY_COMMAND 2>&1 | tee logs/mavproxy_${ID}.log" &
        fi
        MAVPROXY_PIDS+=($!)
      fi
    done
  fi
  echo "Started ${#MAVPROXY_PIDS[@]} bridge instances"
  echo "Waiting 5 seconds for bridge initialization..."
  sleep 5
}

# --- QEMU ---
ssim_start_qemu_if_needed() {
  QEMU_PIDS=()
  if [[ "$QEMU_MODE" != "true" ]]; then
    return 0
  fi
  echo "Starting QEMU ESP32 emulators..."
  local FIRMWARE_PATH="./qemu/firmware.bin"
  if [[ ! -f "$FIRMWARE_PATH" ]]; then
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
    ID=$(echo "$DRONE_CFG" | jq -r '.id')
    ssim_drone_serial5_from_cfg "$DRONE_CFG"
    QEMU_NET_PORT=$((6000 + ID))
    case "$SERIAL5_TYPE" in
      unix)
        QEMU_SERIAL_ARG="-chardev socket,id=serial5,path=$SERIAL5_PATH -device esp32-uart,chardev=serial5"
        SERIAL5_INFO="Unix socket: $SERIAL5_PATH"
        ;;
      *)
        QEMU_SERIAL_ARG="-chardev socket,id=serial5,host=127.0.0.1,port=$SERIAL5_PORT -device esp32-uart,chardev=serial5"
        SERIAL5_INFO="TCP port: $SERIAL5_PORT"
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

    if [[ "$SSIM_HEADLESS" -eq 1 ]]; then
      if [[ $((ID % 20)) -eq 0 ]]; then
        echo "   Starting QEMU ESP32 emulator $ID/$DRONE_COUNT (Network: TCP port $QEMU_NET_PORT)"
      fi
      # shellcheck disable=SC2086
      $QEMU_COMMAND >"logs/qemu_${ID}.log" 2>&1 &
      QEMU_PIDS+=($!)
    else
      echo "Starting QEMU ESP32 emulator for drone $ID..."
      echo "   Serial5: $SERIAL5_INFO"
      echo "   Network: TCP port $QEMU_NET_PORT (for Go-simulator)"
      echo "   Command: $QEMU_COMMAND"
      xterm -title "QEMU ESP32 $ID" -hold -e "$QEMU_COMMAND 2>&1 | tee logs/qemu_${ID}.log" &
      QEMU_PIDS+=($!)
    fi
  done
  echo "Started ${#QEMU_PIDS[@]} QEMU ESP32 emulators"
  echo "Waiting for QEMU ESP32 initialization..."
  sleep 10
}

ssim_prepare_simulator_config() {
  if [[ "$QEMU_MODE" == "true" ]]; then
    echo "   QEMU ESP32 emulators will handle data routing"
    TEMP_CONFIG="config_qemu_temp.json"
    echo "   Creating temporary QEMU config: $TEMP_CONFIG"
    jq '.drones |= map(.serial5.port = (6000 + .id))' "$CONFIG_FILE" >"$TEMP_CONFIG"
    CONFIG_TO_USE=$TEMP_CONFIG
  else
    echo "   Direct connection to SITL instances"
    CONFIG_TO_USE=$CONFIG_FILE
  fi
}

ssim_start_simulator_background() {
  echo "Starting Go simulator in $SIM_MODE mode..."
  ssim_prepare_simulator_config
  SIMULATOR_CMD="./bin/golang-simulator -config $CONFIG_TO_USE -mode $SIM_MODE"
  if [[ "$SIM_MODE" == "experiment" ]]; then
    echo "   Using scenario: $SCENARIO_FILE"
    SIMULATOR_CMD+=" -scenario $SCENARIO_FILE"
  fi
  # shellcheck disable=SC2086
  $SIMULATOR_CMD &
  SIMULATOR_PID=$!
}

ssim_print_banner() {
  echo ""
  if [[ "$SSIM_HEADLESS" -eq 1 ]]; then
    echo "All systems started in headless mode!"
  else
    echo "All systems started!"
  fi
  echo "Go Simulator PID: $SIMULATOR_PID"
  echo "Bridge PIDs: ${MAVPROXY_PIDS[*]}"
  echo "SITL PIDs: ${SITL_PIDS[*]}"
  if [[ "$QEMU_MODE" == "true" ]]; then
    echo "QEMU ESP32 PIDs: ${QEMU_PIDS[*]}"
  fi
  echo ""
  echo "Log files:"
  for DRONE_CFG in $DRONES; do
    ID=$(echo "$DRONE_CFG" | jq -r '.id')
    echo "   Drone $ID SITL: logs/drone_${ID}.log"
    if [[ "$MAVPROXY_IMPL" == "go" ]] && ! ssim_go_mavproxy_no_log; then
      echo "   Drone $ID bridge app: logs/mavproxy_${ID}.app.log"
      echo "   Drone $ID bridge messages: logs/mavproxy_${ID}.messages.log"
    elif [[ "$MAVPROXY_IMPL" == "python" && "$MAVPROXY_TOPOLOGY" == "per-instance" ]]; then
      echo "   Drone $ID MAVProxy: logs/mavproxy_${ID}.log"
    fi
    if [[ "$QEMU_MODE" == "true" ]]; then
      echo "   Drone $ID QEMU ESP32: logs/qemu_${ID}.log"
    fi
  done
  if [[ -n "$MAVPROXY_SESSION_CONFIG" ]]; then
    echo "   Session config: $MAVPROXY_SESSION_CONFIG"
  fi
  if [[ "$MAVPROXY_IMPL" == "go" && "$MAVPROXY_TOPOLOGY" == "single-process" ]]; then
    if ssim_go_mavproxy_no_log; then
      echo "   Single go-mavproxy: logging disabled (MAVPROXY_GO_NO_LOG=1)"
    else
      echo "   Single go-mavproxy console: logs/mavproxy_single.log"
    fi
  fi
  if [[ "$MAVPROXY_IMPL" == "python" && "$MAVPROXY_TOPOLOGY" == "single-process" ]]; then
    echo "   Single mavproxy.py: logs/mavproxy_single.log"
  fi
  echo ""
  if [[ "$SSIM_HEADLESS" -eq 1 ]]; then
    echo "To monitor a drone: tail -f logs/drone_0.log"
  fi
  echo "Press Ctrl+C to stop all processes..."
}

ssim_cleanup() {
  echo ""
  echo "Stopping all processes..."

  if [[ -n "${SIMULATOR_PID:-}" ]]; then
    echo "   Stopping Go simulator..."
    kill -TERM "$SIMULATOR_PID" 2>/dev/null || true
    wait "$SIMULATOR_PID" 2>/dev/null || true
  fi

  echo "   Stopping bridge instances..."
  for PID in "${MAVPROXY_PIDS[@]}"; do
    kill -TERM "$PID" 2>/dev/null || true
  done

  if [[ "$QEMU_MODE" == "true" ]]; then
    echo "   Stopping QEMU ESP32 instances..."
    for PID in "${QEMU_PIDS[@]}"; do
      kill -TERM "$PID" 2>/dev/null || true
    done
  fi

  echo "   Stopping SITL instances..."
  for PID in "${SITL_PIDS[@]}"; do
    kill -TERM "$PID" 2>/dev/null || true
  done

  pkill -f "arducopter" 2>/dev/null || true
  pkill -f "ArduCopter" 2>/dev/null || true
  pkill -f "mavproxy.py" 2>/dev/null || true
  pkill -f "go-mavproxy" 2>/dev/null || true
  if [[ "$QEMU_MODE" == "true" ]]; then
    pkill -f "qemu-system-xtensa" 2>/dev/null || true
  fi

  local nap=3
  if [[ "$SSIM_HEADLESS" -eq 1 ]]; then
    nap=5
  fi
  sleep "$nap"

  if pgrep -f "arducopter|ArduCopter" >/dev/null; then
    echo "   Force killing remaining SITL processes..."
    pkill -9 -f "arducopter" 2>/dev/null || true
    pkill -9 -f "ArduCopter" 2>/dev/null || true
  fi

  if pgrep -f "mavproxy.py|go-mavproxy" >/dev/null; then
    echo "   Force killing bridge processes..."
    pkill -9 -f "mavproxy.py" 2>/dev/null || true
    pkill -9 -f "go-mavproxy" 2>/dev/null || true
  fi

  if [[ "$QEMU_MODE" == "true" ]] && pgrep -f "qemu-system-xtensa" >/dev/null; then
    echo "   Force killing QEMU ESP32..."
    pkill -9 -f "qemu-system-xtensa" 2>/dev/null || true
  fi

  local PROCESS_PATTERN="arducopter|ArduCopter|mavproxy.py|go-mavproxy|golang-simulator"
  if [[ "$QEMU_MODE" == "true" ]]; then
    PROCESS_PATTERN="$PROCESS_PATTERN|qemu-system-xtensa"
  fi

  if pgrep -f "$PROCESS_PATTERN" >/dev/null; then
    echo "WARNING: Some processes still running!"
    ps aux | grep -E "$PROCESS_PATTERN" | grep -v grep || true
  else
    echo "All processes stopped successfully"
  fi

  if [[ "$QEMU_MODE" == "true" && -f "config_qemu_temp.json" ]]; then
    rm -f config_qemu_temp.json
    echo "Cleaned up temporary QEMU config"
  fi

  if [[ -n "${MAVPROXY_SESSION_CONFIG:-}" && -f "$MAVPROXY_SESSION_CONFIG" ]]; then
    rm -f "$MAVPROXY_SESSION_CONFIG"
  fi

  exit 0
}

ssim_main() {
  SSIM_ME="${SSIM_ME:-run_swarm_simulation.sh}"
  SSIM_HEADLESS=0
  local positional=()

  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h | --help)
        ssim_usage
        exit 0
        ;;
      --headless)
        SSIM_HEADLESS=1
        shift
        ;;
      --terminal)
        SSIM_HEADLESS=0
        shift
        ;;
      --)
        shift
        positional+=("$@")
        break
        ;;
      *)
        positional+=("$1")
        shift
        ;;
    esac
  done

  SIM_MODE="${positional[0]:-experiment}"
  CONNECTION_MODE="${positional[1]:-direct}"
  SCENARIO_FILE="${positional[2]:-scenarios/simple_flight.yaml}"

  CONFIG_FILE="${CONFIG_FILE:-config.json}"
  MAVPROXY_IMPL="${MAVPROXY_IMPL:-go}"
  MAVPROXY_TOPOLOGY="${MAVPROXY_TOPOLOGY:-per-instance}"

  if [[ "$MAVPROXY_IMPL" != "go" && "$MAVPROXY_IMPL" != "python" ]]; then
    echo "Invalid MAVPROXY_IMPL: $MAVPROXY_IMPL (use go or python)" >&2
    exit 1
  fi
  if [[ "$MAVPROXY_TOPOLOGY" != "per-instance" && "$MAVPROXY_TOPOLOGY" != "single-process" ]]; then
    echo "Invalid MAVPROXY_TOPOLOGY: $MAVPROXY_TOPOLOGY (use per-instance or single-process)" >&2
    exit 1
  fi

  if [[ -z "${SWARM_SIM_ROOT:-}" ]]; then
    echo "ERROR: SWARM_SIM_ROOT is not set. Launch via run_swarm_simulation.sh." >&2
    exit 1
  fi
  cd "$SWARM_SIM_ROOT" || exit 1

  ssim_append_path_defaults
  ssim_resolve_sitl_bin

  echo "Unified launcher: MAVPROXY_IMPL=$MAVPROXY_IMPL MAVPROXY_TOPOLOGY=$MAVPROXY_TOPOLOGY headless=$SSIM_HEADLESS MAVPROXY_GO_NO_LOG=${MAVPROXY_GO_NO_LOG:-0}"
  echo "SITL_BIN=$SITL_BIN CONFIG_FILE=$CONFIG_FILE"

  ssim_build_go_simulator || exit 1
  ssim_load_drones_from_config "$CONFIG_FILE" || exit 1
  echo "Configuration loaded: $DRONE_COUNT drones"

  ssim_validate_connection_mode || exit 1

  if [[ "$SSIM_HEADLESS" -eq 1 ]]; then
    echo "HEADLESS MODE - background processes, logs in logs/"
  else
    echo "TERMINAL MODE - xterm windows (see run_golang_simulation.sh)"
  fi

  ssim_start_sitl_instances
  ssim_start_mavproxy_default

  ssim_start_qemu_if_needed
  ssim_start_simulator_background
  ssim_print_banner

  trap ssim_cleanup SIGINT SIGTERM
  wait "$SIMULATOR_PID"
  ssim_cleanup
}

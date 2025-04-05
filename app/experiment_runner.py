from pymavlink import mavutil
import time
import os


class ExperimentRunner:
    def __init__(self, network_simulator):
        self.network_simulator = network_simulator
        self.script_running = False
    
    @staticmethod
    def _set_single_param(drone, param_name, value):
        """Sets a single parameter for the drone"""
        # If the value is a string with a number, convert it to float
        if isinstance(value, str) and value.replace(".", "", 1).isdigit():
            value = float(value)
        
        # Send the command to set the parameter
        drone.conn.mav.param_set_send(
            drone.conn.target_system, drone.conn.target_component,
            param_name.encode('utf-8'),
            value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

    def set_parameters(self, params):
        """Sets parameters for all drones"""
        for drone in self.network_simulator.drones:
            if drone.connected:
                for param_name, value in params.items():
                    self._set_single_param(drone, param_name, value)
                    time.sleep(0.1)  # Small delay between setting parameters
    
    def send_script_command(self, pitch, roll, yaw, throttle, duration):
        """Executes a command for a specified duration"""
        start_time = time.time()

        while self.script_running and time.time() - start_time < duration:
            for drone in self.network_simulator.drones:
                drone.send_rc_override(roll, pitch, throttle, yaw)
            time.sleep(0.01)  # Small delay between sending commands
    
    def run_experiment(self):
        """Runs a single experiment with the current parameters"""
        if self.script_running:
            print("Warning: Experiment is already running")
            return

        self.script_running = True
        try:
            # Make a pause of 5 seconds before starting the experiment
            print("Waiting 5 seconds before starting experiment...")
            time.sleep(5)
            
            selected_drones = self.network_simulator.drones
            
            # 1. Set drones to GUIDED mode
            print("Setting GUIDED mode...")
            for drone in selected_drones:
                if not drone.connected:
                    continue
                mode_id = drone.conn.mode_mapping()["GUIDED"]
                drone.conn.mav.set_mode_send(
                    drone.conn.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id
                )
            time.sleep(1)
            
            # 2. Arm drones
            print("Arming drones...")
            for drone in selected_drones:
                if not drone.connected:
                    continue
                drone.arm()
            time.sleep(2)
            
            # 3. Send the takeoff command
            takeoff_altitude = 10.0  # in meters
            print(f"Taking off to {takeoff_altitude}m...")
            for drone in selected_drones:
                if not drone.connected:
                    continue
                drone.conn.mav.command_long_send(
                    drone.conn.target_system, drone.conn.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, takeoff_altitude
                )
            time.sleep(10)  # Wait for drones to take off
            
            # 4. Set drones to POSHOLD mode for further control
            print("Setting POSHOLD mode...")
            for drone in selected_drones:
                if not drone.connected:
                    continue
                mode_id = drone.conn.mode_mapping()["POSHOLD"]
                drone.conn.mav.set_mode_send(
                    drone.conn.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id
                )
            time.sleep(1)

            # 5. Fly forward for 20 seconds
            print("Flying forward for 20 seconds...")
            self.send_script_command(pitch=1200, roll=1500, yaw=1500, throttle=1500, duration=20)

            # 6. Fly forward and slightly turn for 15 seconds
            print("Flying forward with slight turn for 15 seconds...")
            self.send_script_command(pitch=1125, roll=1500, yaw=1550, throttle=1500, duration=15)

            # 7. Fly forward for another 20 seconds
            print("Flying forward for 20 more seconds...")
            self.send_script_command(pitch=1200, roll=1500, yaw=1500, throttle=1500, duration=20)

            # 8. Return the joysticks to the center
            print("Experiment completed, centering sticks...")
            self.send_script_command(pitch=1500, roll=1500, yaw=1500, throttle=1500, duration=10)

            print("Script completed successfully")
        except Exception as e:
            print(f"Error during script execution: {str(e)}")
        finally:
            self.script_running = False
            
    def save_logs(self, param_set, exp_num):
        """Renames logs according to the experiment number"""
        time.sleep(1)  # Give time to save logs
        for drone_id in range(len(self.network_simulator.drones)):
            old_name = f"./logs/drone_{drone_id}_position.csv"
            new_name = f"./logs/drone_{drone_id}_position_set{param_set}_exp{exp_num}.csv"
            try:
                if os.path.exists(old_name):
                    os.rename(old_name, new_name)
                    print(f"Saved log as {new_name}")
                else:
                    print(f"Log file {old_name} does not exist")
            except Exception as e:
                print(f"Error renaming log file: {e}") 
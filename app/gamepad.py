from pymavlink import mavutil
import tkinter as tk
from tkinter import messagebox, ttk, simpledialog
import threading
import time

from network import NetworkSimulator
from drone import Drone


class DroneControlGUI:
    def __init__(self, network_simulator: NetworkSimulator):
        self.network_simulator = network_simulator
        self.root = tk.Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.title("Drone Control Panel")
        self.create_widgets()

        # Flags for joystick activity
        self.joystick1_active = False  # For Roll and Pitch
        self.joystick2_active = False  # For Throttle and Yaw

        # Selected drone for control
        self.selected_drone = None

        # Current RC channel values
        self.current_roll = 1500
        self.current_pitch = 1500
        self.current_throttle = 1500
        self.current_yaw = 1500

        # Flag for script execution
        self.script_running = False

        # Start the loop to send default values if joysticks are not active
        self.update_rc_values()

    def create_widgets(self):
        # Switch for selecting a drone
        self.drone_selection_frame = tk.Frame(self.root)
        self.drone_selection_frame.pack(padx=10, pady=10)

        self.drone_label = tk.Label(self.drone_selection_frame, text="Select Drone:")
        self.drone_label.pack(side=tk.LEFT)

        self.drone_var = tk.StringVar()
        self.drone_combobox = ttk.Combobox(
            self.drone_selection_frame,
            textvariable=self.drone_var,
            values=[f"Drone {drone.id}" for drone in self.network_simulator.drones] + ["All"]
        )
        self.drone_combobox.set("All")  # Значение по умолчанию
        self.drone_combobox.pack(side=tk.LEFT)

        # Single control block for all drones
        control_frame = tk.Frame(self.root)
        control_frame.pack(padx=10, pady=10)

        # Control buttons
        self.arm_button = tk.Button(control_frame, text="ARM", command=self.arm_selected)
        self.arm_button.pack(side=tk.LEFT, padx=5)

        self.disarm_button = tk.Button(control_frame, text="DISARM", command=self.disarm_selected)
        self.disarm_button.pack(side=tk.LEFT, padx=5)

        self.takeoff_button = tk.Button(control_frame, text="TAKE OFF", command=self.takeoff_selected)
        self.takeoff_button.pack(side=tk.LEFT, padx=5)

        self.mode_button = tk.Button(control_frame, text="SET MODE", command=self.set_mode_selected)
        self.mode_button.pack(side=tk.LEFT, padx=5)

        # Button for running a script
        self.script_button = tk.Button(control_frame, text="RUN SCRIPT", command=self.run_script)
        self.script_button.pack(side=tk.LEFT, padx=5)

        # Button for setting parameters
        self.param_button = tk.Button(control_frame, text="SET PARAM", command=self.set_parameters)
        self.param_button.pack(side=tk.LEFT, padx=5)

        # Horizontal placement of joysticks
        self.joysticks_frame = tk.Frame(self.root)
        self.joysticks_frame.pack(padx=10, pady=10)

        # First joystick (Roll and Pitch)
        self.joystick1_frame = tk.Frame(self.joysticks_frame)
        self.joystick1_frame.pack(side=tk.LEFT, padx=10)

        self.joystick1_label = tk.Label(self.joystick1_frame, text="Joystick 1 (Roll/Pitch)")
        self.joystick1_label.pack()

        self.joystick1_canvas = tk.Canvas(self.joystick1_frame, width=200, height=200, bg="white")
        self.joystick1_canvas.pack()

        # Drawing the center of the first joystick
        self.joystick1_center = (100, 100)
        self.joystick1_radius = 50
        self.joystick1_canvas.create_oval(
            self.joystick1_center[0] - self.joystick1_radius,
            self.joystick1_center[1] - self.joystick1_radius,
            self.joystick1_center[0] + self.joystick1_radius,
            self.joystick1_center[1] + self.joystick1_radius,
            fill="gray"
        )

        # Current position of the first joystick
        self.joystick1_position = self.joystick1_center

        # Binding mouse events for the first joystick
        self.joystick1_canvas.bind("<Button-1>", lambda e: self.on_joystick_press(e, joystick_id=1))
        self.joystick1_canvas.bind("<B1-Motion>", lambda e: self.on_joystick_drag(e, joystick_id=1))
        self.joystick1_canvas.bind("<ButtonRelease-1>", lambda e: self.on_joystick_release(joystick_id=1))

        # Second joystick (Throttle and Yaw)
        self.joystick2_frame = tk.Frame(self.joysticks_frame)
        self.joystick2_frame.pack(side=tk.LEFT, padx=10)

        self.joystick2_label = tk.Label(self.joystick2_frame, text="Joystick 2 (Throttle/Yaw)")
        self.joystick2_label.pack()

        self.joystick2_canvas = tk.Canvas(self.joystick2_frame, width=200, height=200, bg="white")
        self.joystick2_canvas.pack()

        # Drawing the center of the second joystick
        self.joystick2_center = (100, 100)
        self.joystick2_radius = 50
        self.joystick2_canvas.create_oval(
            self.joystick2_center[0] - self.joystick2_radius,
            self.joystick2_center[1] - self.joystick2_radius,
            self.joystick2_center[0] + self.joystick2_radius,
            self.joystick2_center[1] + self.joystick2_radius,
            fill="gray"
        )

        # Current position of the second joystick
        self.joystick2_position = self.joystick2_center

        # Binding mouse events for the second joystick
        self.joystick2_canvas.bind("<Button-1>", lambda e: self.on_joystick_press(e, joystick_id=2))
        self.joystick2_canvas.bind("<B1-Motion>", lambda e: self.on_joystick_drag(e, joystick_id=2))
        self.joystick2_canvas.bind("<ButtonRelease-1>", lambda e: self.on_joystick_release(joystick_id=2))

    def set_parameters(self):
        """Sets parameters for all selected drones with the ability to enter arbitrary parameters"""
        param_window = tk.Toplevel(self.root)
        param_window.title("Set Parameters")
        param_window.geometry("600x400")

        # Frame for frequently used parameters
        common_frame = tk.LabelFrame(param_window, text="Common Parameters")
        common_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # List of frequently used parameters
        common_params = [
            ("Swarm_XY_ALGO",    "xy algorithm",  "2"),
            ("Swarm_H_ALGO",     "dir algorithm", "1"),
            ("Swarm_LVP_XY_A",   "LVP xy: A",     "0.3"),   # (0, 1)
            ("Swarm_LVP_H_A",    "LVP h: A",      "0.3"),   # (0, 1)
            ("Swarm_ALVP_XY_A",  "ALVP xy: A",    "0.3"),   # (0, 1)
            ("Swarm_ALVP_XY_G",  "ALVP xy: G",    "10"),    # > 0
            ("Swarm_ALVP_XY_H",  "ALVP xy: H",    "0.33"),  # > 0
            ("Swarm_ALVP_XY_L",  "ALVP xy: L",    "3"),     # >= 0
            ("Swarm_ALVP_XY_ME", "ALVP xy: ME",   "10"),    # > 0
        ]

        # Variables for storing values
        param_vars = {}

        # Create input fields for each parameter
        for i, (param_name, param_label, default_val) in enumerate(common_params):
            tk.Label(common_frame, text=param_label).grid(row=i, column=0, padx=5, pady=2, sticky="e")
            var = tk.StringVar(value=default_val)
            param_vars[param_name] = var
            tk.Entry(common_frame, textvariable=var).grid(row=i, column=1, padx=5, pady=2)

        # Frame for arbitrary parameters
        custom_frame = tk.LabelFrame(param_window, text="Custom Parameters")
        custom_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # List for storing arbitrary parameters
        self.custom_params = []

        # Function for adding a new parameter
        def add_custom_param():
            row = len(self.custom_params)

            # Field for parameter name
            name_var = tk.StringVar()
            tk.Label(custom_frame, text="Parameter name:").grid(row=row, column=0, padx=5, pady=2, sticky="e")
            name_entry = tk.Entry(custom_frame, textvariable=name_var)
            name_entry.grid(row=row, column=1, padx=5, pady=2)

            # Field for parameter value
            value_var = tk.StringVar()
            tk.Label(custom_frame, text="Value:").grid(row=row, column=2, padx=5, pady=2, sticky="e")
            value_entry = tk.Entry(custom_frame, textvariable=value_var)
            value_entry.grid(row=row, column=3, padx=5, pady=2)

            # Button for removing a parameter
            def remove_param():
                for widget in custom_frame.grid_slaves():
                    if int(widget.grid_info()["row"]) == row:
                        widget.destroy()
                self.custom_params.remove((name_var, value_var))
                # Renumber the remaining parameters
                for i, (_, _) in enumerate(self.custom_params):
                    for widget in custom_frame.grid_slaves():
                        if int(widget.grid_info()["row"]) == i:
                            widget.grid(row=i)

            remove_btn = tk.Button(custom_frame, text="×", command=remove_param)
            remove_btn.grid(row=row, column=4, padx=5, pady=2)

            self.custom_params.append((name_var, value_var))

        # Button for adding a new parameter
        add_param_btn = tk.Button(custom_frame, text="+ Add Parameter", command=add_custom_param)
        add_param_btn.grid(row=0, column=0, columnspan=5, pady=5)

        # Function for applying parameters
        def apply_params():
            try:
                selected_drones = self.get_selected_drones()
                if not selected_drones:
                    messagebox.showerror("Error", "No drones selected")
                    return

                # Apply standard parameters
                for param_name, var in param_vars.items():
                    value = var.get()
                    if value:
                        for drone in selected_drones:
                            if drone.connected:
                                self._set_single_param(drone, param_name, value)

                # Apply arbitrary parameters
                for name_var, value_var in self.custom_params:
                    param_name = name_var.get()
                    value = value_var.get()
                    if param_name and value:
                        for drone in selected_drones:
                            if drone.connected:
                                self._set_single_param(drone, param_name, value)

                messagebox.showinfo("Success", f"Parameters set for {len(selected_drones)} drones")
                param_window.destroy()
            except Exception as e:
                messagebox.showerror("Error", f"Failed to set parameters: {str(e)}")

        # Frame for buttons
        button_frame = tk.Frame(param_window)
        button_frame.pack(fill=tk.X, padx=5, pady=5)

        # Confirm button
        tk.Button(button_frame, text="Apply", command=apply_params).pack(side=tk.RIGHT, padx=5)

        # Cancel button
        tk.Button(button_frame, text="Cancel", command=param_window.destroy).pack(side=tk.RIGHT, padx=5)

    def _set_single_param(self, drone, param_name, value):
        """Sets a single parameter for a single drone"""
        try:
            # Try to convert the value to a number
            try:
                param_value = float(value)
                param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            except ValueError:
                # If not a number, send as a string
                param_value = value
                param_type = mavutil.mavlink.MAV_PARAM_TYPE_STRING

            drone.conn.mav.param_set_send(
                drone.conn.target_system,
                drone.conn.target_component,
                param_name.encode('utf-8'),
                param_value,
                param_type
            )

            # Request the parameter back for confirmation
            drone.conn.mav.param_request_read_send(
                drone.conn.target_system,
                drone.conn.target_component,
                param_name.encode('utf-8'),
                -1
            )

            # Log the parameter setting
            print(f"Set parameter {param_name}={value} for drone {drone.id}")
        except Exception as e:
            print(f"Failed to set parameter {param_name} for drone {drone.id}: {str(e)}")
            raise

    def run_script(self):
        """Runs one experiment with the current parameters"""
        if self.script_running:
            messagebox.showwarning("Warning", "Script is already running")
            return

        def script_thread():
            self.script_running = True
            self.script_button.config(state=tk.DISABLED)
            try:
                for drone in self.get_selected_drones():
                    if not drone.connected:
                        continue
                    mode_id = drone.conn.mode_mapping()["POSHOLD"]
                    drone.conn.mav.set_mode_send(
                        drone.conn.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        mode_id
                    )
                time.sleep(0.5)

                # 1. Fly forward for 10 seconds
                self.send_script_command(pitch=1200, roll=1500, yaw=1500, throttle=1500, duration=10)

                # 2. Fly forward and slightly turn for 15 seconds
                self.send_script_command(pitch=1125, roll=1500, yaw=1550, throttle=1500, duration=15)

                # 3. Fly forward for another 10 seconds
                self.send_script_command(pitch=1200, roll=1500, yaw=1500, throttle=1500, duration=10)

                # Return the joysticks to the center
                self.send_script_command(pitch=1500, roll=1500, yaw=1500, throttle=1500, duration=0)

                messagebox.showinfo("Script", "Script completed successfully")
            except Exception as e:
                messagebox.showerror("Script Error", f"Error during script execution: {str(e)}")
            finally:
                self.script_running = False
                self.script_button.config(state=tk.NORMAL)

        # Start the script in a separate thread to avoid blocking the GUI
        threading.Thread(target=script_thread, daemon=True).start()

    def run_sync_script(self):
        """Runs one experiment with the current parameters"""
        if self.script_running:
            messagebox.showwarning("Warning", "Script is already running")
            return

        self.script_running = True
        self.script_button.config(state=tk.DISABLED)
        try:
            # Make a pause of 30 seconds before starting the experiment
            print("Waiting 30 seconds before starting experiment...")
            time.sleep(30)
            
            selected_drones = self.get_selected_drones()
            
            # 1. Set the drones to GUIDED mode
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
            
            # 2. Arm the drones
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
            time.sleep(10)  # Wait for the drones to take off
            
            # 4. Set the drones to POSHOLD mode for further control
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

            # 5. Fly forward for 10 seconds
            print("Flying forward for 10 seconds...")
            self.send_script_command(pitch=1200, roll=1500, yaw=1500, throttle=1500, duration=10)

            # 6. Fly forward and slightly turn for 15 seconds
            print("Flying forward with slight turn for 15 seconds...")
            self.send_script_command(pitch=1200, roll=1500, yaw=1550, throttle=1500, duration=15)

            # 7. Fly forward for another 10 seconds
            print("Flying forward for 10 more seconds...")
            self.send_script_command(pitch=1200, roll=1500, yaw=1500, throttle=1500, duration=10)

            # 8. Return the joysticks to the center
            print("Experiment completed, centering sticks...")
            self.send_script_command(pitch=1500, roll=1500, yaw=1500, throttle=1500, duration=0)

            messagebox.showinfo("Script", "Script completed successfully")
        except Exception as e:
            messagebox.showerror("Script Error", f"Error during script execution: {str(e)}")
        finally:
            self.script_running = False
            self.script_button.config(state=tk.NORMAL)

    def send_script_command(self, pitch, roll, yaw, throttle, duration):
        """Sends a command to all drones and waits for the specified time"""
        start_time = time.time()

        while self.script_running and time.time() - start_time < duration:
            for drone in self.network_simulator.drones:
                drone.send_rc_override(roll, pitch, throttle, yaw)
            time.sleep(0.01)  # Small delay between sending

    def on_joystick_press(self, event, joystick_id):
        """Processing a joystick press."""
        if joystick_id == 1:
            self.joystick1_active = True
            self.update_joystick_position(event.x, event.y, joystick_id=1)
        elif joystick_id == 2:
            self.joystick2_active = True
            self.update_joystick_position(event.x, event.y, joystick_id=2)

    def on_joystick_drag(self, event, joystick_id):
        """Processing a joystick movement."""
        self.update_joystick_position(event.x, event.y, joystick_id)

    def on_joystick_release(self, event=None, joystick_id=None):
        """Processing a joystick release."""
        if joystick_id == 1:
            self.joystick1_active = False
            self.update_joystick_position(self.joystick1_center[0], self.joystick1_center[1], joystick_id=1)
        elif joystick_id == 2:
            self.joystick2_active = False
            self.update_joystick_position(self.joystick2_center[0], self.joystick2_center[1], joystick_id=2)

    def update_joystick_position(self, x, y, joystick_id):
        """Updating the joystick position and sending values to the drone."""
        if joystick_id == 1:
            center = self.joystick1_center
            radius = self.joystick1_radius
            canvas = self.joystick1_canvas
            self.joystick1_position = (x, y)
        elif joystick_id == 2:
            center = self.joystick2_center
            radius = self.joystick2_radius
            canvas = self.joystick2_canvas
            self.joystick2_position = (x, y)
        else:
            return

        # Limit the joystick position within the circle
        dx = x - center[0]
        dy = y - center[1]
        distance = (dx ** 2 + dy ** 2) ** 0.5
        if distance > radius:
            dx = dx * radius / distance
            dy = dy * radius / distance
            x = center[0] + dx
            y = center[1] + dy

        # Update the joystick position
        canvas.delete("joystick")
        canvas.create_oval(
            x - 10, y - 10, x + 10, y + 10, fill="blue", tags="joystick"
        )

        # Convert the joystick position to values for RC channels
        if joystick_id == 1:
            roll = int(1500 + (dx / radius) * 500)  # Channel 1 (Roll)
            pitch = int(1500 + (dy / radius) * 500)  # Channel 2 (Pitch)
            self.current_roll = roll
            self.current_pitch = pitch
        elif joystick_id == 2:
            throttle = int(1500 - (dy / radius) * 500)  # Channel 3 (Throttle)
            yaw = int(1500 + (dx / radius) * 500)  # Channel 4 (Yaw)
            self.current_throttle = throttle
            self.current_yaw = yaw

    def update_rc_values(self):
        """Method for sending values to the drone."""
        # Default values
        roll = default_roll = 1500
        pitch = default_pitch = 1500
        throttle = default_throttle = 1500
        yaw = default_yaw = 1500

        # Get the selected drone
        selected_drone = self.drone_var.get()

        # If joystick 1 is active, use its values
        if self.joystick1_active:
            roll = self.current_roll
            pitch = self.current_pitch

        # If joystick 2 is active, use its values
        if self.joystick2_active:
            throttle = self.current_throttle
            yaw = self.current_yaw

        # Send values to the selected drone
        for drone in self.network_simulator.drones:
            if selected_drone == f"Drone {drone.id}" or selected_drone == "All":
                drone.send_rc_override(roll, pitch, throttle, yaw)
            else:
                drone.send_rc_override(default_roll, default_pitch, default_throttle, default_yaw)

        # Repeat the method call every 90 ms
        self.root.after(90, self.update_rc_values)

    def get_selected_drones(self) -> list[Drone]:
        """Returns a list of selected drones"""
        selected = self.drone_var.get()
        if selected == "All":
            return self.network_simulator.drones
        return [d for d in self.network_simulator.drones if f"Drone {d.id}" == selected]

    def arm_selected(self):
        for drone in self.get_selected_drones():
            drone.arm()
        if len(self.get_selected_drones()) > 1:
            messagebox.showinfo("ARM", "All drones armed")
        else:
            messagebox.showinfo("ARM", f"Drone {self.get_selected_drones()[0].id} armed")

    def disarm_selected(self):
        for drone in self.get_selected_drones():
            drone.disarm()
        if len(self.get_selected_drones()) > 1:
            messagebox.showinfo("DISARM", "All drones disarmed")
        else:
            messagebox.showinfo("DISARM", f"Drone {self.get_selected_drones()[0].id} disarmed")

    def takeoff_selected(self):
        altitude = simpledialog.askfloat("Takeoff Altitude", "Enter altitude (meters):", initialvalue=10.0)
        if altitude is not None:
            for drone in self.get_selected_drones():
                drone.conn.mav.command_long_send(
                    drone.conn.target_system, drone.conn.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude
                )
            messagebox.showinfo("TAKE OFF", f"Takeoff command sent to {len(self.get_selected_drones())} drones")

    def set_mode_selected(self):
        modes = ["GUIDED", "POSHOLD", "STABILIZE"]
        mode = simpledialog.askstring("Set Mode", "Enter flight mode:", initialvalue="GUIDED")
        if mode and mode.upper() in modes:
            for drone in self.get_selected_drones():
                if not drone.connected:
                    continue
                mode_id = drone.conn.mode_mapping()[mode.upper()]
                drone.conn.mav.set_mode_send(
                    drone.conn.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id
                )
            messagebox.showinfo(
                "SET MODE",
                f"Mode {mode.upper()} set to {len(self.get_selected_drones())} drones"
            )
        else:
            messagebox.showerror("Error", "Invalid flight mode selected")

    def on_closing(self):
        self.script_running = False

        for drone in self.network_simulator.drones:
            drone.active = False
            if drone.serial5_socket:
                drone.serial5_socket.close()
        self.root.destroy()
        print("Application closed.")

    def run(self):
        self.root.mainloop()

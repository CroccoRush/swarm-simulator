from pymavlink import mavutil
import tkinter as tk
from tkinter import messagebox, ttk, simpledialog

from network import NetworkSimulator
from drone import Drone


class DroneControlGUI:
    def __init__(self, network_simulator: NetworkSimulator):
        self.network_simulator = network_simulator
        self.root = tk.Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.title("Drone Control Panel")
        self.create_widgets()

        # Флаги активности джойстиков
        self.joystick1_active = False  # Для Roll и Pitch
        self.joystick2_active = False  # Для Throttle и Yaw

        # Выбранный дрон для управления
        self.selected_drone = None

        # Запускаем цикл для отправки значений по умолчанию, если джойстики не активны
        self.update_rc_values()

    def create_widgets(self):
        # Переключатель для выбора дрона
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

        # Единый блок управления для всех дронов
        control_frame = tk.Frame(self.root)
        control_frame.pack(padx=10, pady=10)

        # Кнопки управления
        self.arm_button = tk.Button(control_frame, text="ARM", command=self.arm_selected)
        self.arm_button.pack(side=tk.LEFT, padx=5)

        self.disarm_button = tk.Button(control_frame, text="DISARM", command=self.disarm_selected)
        self.disarm_button.pack(side=tk.LEFT, padx=5)

        self.takeoff_button = tk.Button(control_frame, text="TAKE OFF", command=self.takeoff_selected)
        self.takeoff_button.pack(side=tk.LEFT, padx=5)

        self.mode_button = tk.Button(control_frame, text="SET MODE", command=self.set_mode_selected)
        self.mode_button.pack(side=tk.LEFT, padx=5)

        # Горизонтальное расположение джойстиков
        self.joysticks_frame = tk.Frame(self.root)
        self.joysticks_frame.pack(padx=10, pady=10)

        # Первый джойстик (Roll и Pitch)
        self.joystick1_frame = tk.Frame(self.joysticks_frame)
        self.joystick1_frame.pack(side=tk.LEFT, padx=10)

        self.joystick1_label = tk.Label(self.joystick1_frame, text="Joystick 1 (Roll/Pitch)")
        self.joystick1_label.pack()

        self.joystick1_canvas = tk.Canvas(self.joystick1_frame, width=200, height=200, bg="white")
        self.joystick1_canvas.pack()

        # Отрисовка центра первого джойстика
        self.joystick1_center = (100, 100)
        self.joystick1_radius = 50
        self.joystick1_canvas.create_oval(
            self.joystick1_center[0] - self.joystick1_radius,
            self.joystick1_center[1] - self.joystick1_radius,
            self.joystick1_center[0] + self.joystick1_radius,
            self.joystick1_center[1] + self.joystick1_radius,
            fill="gray"
        )

        # Текущее положение первого джойстика
        self.joystick1_position = self.joystick1_center

        # Привязка событий мыши для первого джойстика
        self.joystick1_canvas.bind("<Button-1>", lambda e: self.on_joystick_press(e, joystick_id=1))
        self.joystick1_canvas.bind("<B1-Motion>", lambda e: self.on_joystick_drag(e, joystick_id=1))
        self.joystick1_canvas.bind("<ButtonRelease-1>", lambda e: self.on_joystick_release(joystick_id=1))

        # Второй джойстик (Throttle и Yaw)
        self.joystick2_frame = tk.Frame(self.joysticks_frame)
        self.joystick2_frame.pack(side=tk.LEFT, padx=10)

        self.joystick2_label = tk.Label(self.joystick2_frame, text="Joystick 2 (Throttle/Yaw)")
        self.joystick2_label.pack()

        self.joystick2_canvas = tk.Canvas(self.joystick2_frame, width=200, height=200, bg="white")
        self.joystick2_canvas.pack()

        # Отрисовка центра второго джойстика
        self.joystick2_center = (100, 100)
        self.joystick2_radius = 50
        self.joystick2_canvas.create_oval(
            self.joystick2_center[0] - self.joystick2_radius,
            self.joystick2_center[1] - self.joystick2_radius,
            self.joystick2_center[0] + self.joystick2_radius,
            self.joystick2_center[1] + self.joystick2_radius,
            fill="gray"
        )

        # Текущее положение второго джойстика
        self.joystick2_position = self.joystick2_center

        # Привязка событий мыши для второго джойстика
        self.joystick2_canvas.bind("<Button-1>", lambda e: self.on_joystick_press(e, joystick_id=2))
        self.joystick2_canvas.bind("<B1-Motion>", lambda e: self.on_joystick_drag(e, joystick_id=2))
        self.joystick2_canvas.bind("<ButtonRelease-1>", lambda e: self.on_joystick_release(joystick_id=2))

    def on_joystick_press(self, event, joystick_id):
        """Обработка нажатия на джойстик."""
        if joystick_id == 1:
            self.joystick1_active = True
            self.update_joystick_position(event.x, event.y, joystick_id=1)
        elif joystick_id == 2:
            self.joystick2_active = True
            self.update_joystick_position(event.x, event.y, joystick_id=2)

    def on_joystick_drag(self, event, joystick_id):
        """Обработка перемещения джойстика."""
        self.update_joystick_position(event.x, event.y, joystick_id)

    def on_joystick_release(self, event=None, joystick_id=None):
        """Обработка отпускания джойстика."""
        if joystick_id == 1:
            self.joystick1_active = False
            self.update_joystick_position(self.joystick1_center[0], self.joystick1_center[1], joystick_id=1)
        elif joystick_id == 2:
            self.joystick2_active = False
            self.update_joystick_position(self.joystick2_center[0], self.joystick2_center[1], joystick_id=2)

    def update_joystick_position(self, x, y, joystick_id):
        """Обновление положения джойстика и отправка значений в дрон."""
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

        # Ограничиваем положение джойстика в пределах круга
        dx = x - center[0]
        dy = y - center[1]
        distance = (dx ** 2 + dy ** 2) ** 0.5
        if distance > radius:
            dx = dx * radius / distance
            dy = dy * radius / distance
            x = center[0] + dx
            y = center[1] + dy

        # Обновляем положение джойстика
        canvas.delete("joystick")
        canvas.create_oval(
            x - 10, y - 10, x + 10, y + 10, fill="blue", tags="joystick"
        )

        # Преобразуем положение джойстика в значения для RC-каналов
        if joystick_id == 1:
            roll = int(1500 + (dx / radius) * 500)  # Канал 1 (Roll)
            pitch = int(1500 + (dy / radius) * 500)  # Канал 2 (Pitch)
            self.current_roll = roll
            self.current_pitch = pitch
        elif joystick_id == 2:
            throttle = int(1500 - (dy / radius) * 500)  # Канал 3 (Throttle)
            yaw = int(1500 + (dx / radius) * 500)  # Канал 4 (Yaw)
            self.current_throttle = throttle
            self.current_yaw = yaw

    def update_rc_values(self):
        """Метод для отправки значений в дрон."""
        # Значения по умолчанию
        roll = 1500
        pitch = 1500
        throttle = 1500
        yaw = 1500

        # Получаем выбранного дрона
        selected_drone = self.drone_var.get()

        # Если джойстик 1 активен, используем его значения
        if self.joystick1_active:
            roll = self.current_roll
            pitch = self.current_pitch

        # Если джойстик 2 активен, используем его значения
        if self.joystick2_active:
            throttle = self.current_throttle
            yaw = self.current_yaw

        # Отправляем значения выбранному дрону
        for drone in self.network_simulator.drones:
            if selected_drone == f"Drone {drone.id}" or selected_drone == "All":
                drone.send_rc_override(roll, pitch, throttle, yaw)

        # Повторяем вызов метода каждые 100 мс
        self.root.after(100, self.update_rc_values)

    def get_selected_drones(self) -> list[Drone]:
        """Возвращает список выбранных дронов"""
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
        for drone in self.network_simulator.drones:
            drone.active = False
            if drone.serial5_socket:
                drone.serial5_socket.close()
        self.root.destroy()
        print("Application closed.")

    def run(self):
        self.root.mainloop()

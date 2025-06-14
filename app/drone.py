import socket
import time
import random
from typing import Optional

from pymavlink import mavutil
from threading import Thread

from pymavlink.mavutil import mavudp


class Drone:
    sync_flag = False

    def __init__(
        self, drone_id, udp_port, serial5_port, initial_position, base_packet_loss
    ):
        self.id = drone_id
        self.udp_port = udp_port
        self.serial5_port = serial5_port
        self.initial_position = initial_position
        self.base_packet_loss = base_packet_loss
        self.conn: Optional[mavudp] = None
        self.serial5_socket: Optional[socket.socket] = None
        self.position = (0.0, 0.0, 0.0, 0.0)
        self.mav_connected = False
        self.serial5_connected = False
        self.connected = False
        self.position_thread = None
        self.serial5_thread = None
        self.other_drones = []

        self.connect_serial5()
        time.sleep(0.5)
        self.connect_mavlink()
        time.sleep(0.5)

        self.connected = self.serial5_connected and self.mav_connected
        if self.connected:
            self.position_thread = Thread(target=self.speak)
            self.position_thread.daemon = True
            self.position_thread.start()

    # Function for sending RC commands
    def send_rc_override(
        self, channel_1, channel_2, channel_3, channel_4, channel_5=65535, channel_6=65535, channel_7=65535, channel_8=65535
    ):
        self.conn.mav.rc_channels_override_send(
            self.conn.target_system,     # ID system
            self.conn.target_component,  # ID component
            channel_1,  # Channel 1 (Roll)
            channel_2,  # Channel 2 (Pitch)
            channel_3,  # Channel 3 (Throttle)
            channel_4,  # Channel 4 (Yaw)
            channel_5,  # Channel 5 (additional)
            channel_6,  # Channel 6 (additional)
            channel_7,  # Channel 7 (additional)
            channel_8   # Channel 8 (additional)
        )

    def arm(self):
        self.conn.arducopter_arm()

    def disarm(self):
        self.conn.arducopter_disarm()

    def connect_mavlink(self):
        while not self.mav_connected:
            try:
                print(f"Drone#{self.id}: Connecting to MAVLink (UDP port {self.udp_port})...")
                self.conn: mavudp = mavutil.mavlink_connection(f'udpin:0.0.0.0:{self.udp_port}')
                self.conn.target_system = self.id + 1
                # self.conn.wait_heartbeat()
                msg = self.conn.recv_match(type='HEARTBEAT', blocking=True)
                if msg:
                    print(f"Drone#{self.id}: SYSID#{msg.get_srcSystem()}, COMPID#{msg.get_srcComponent()}")
                    print(f"Drone#{self.id}: CSYS#{self.conn.target_system}, CCOM#{self.conn.target_component}")
                self.mav_connected = True
                print(f"Drone#{self.id}: Connected to MAVLink successfully!")
            except Exception as e:
                print(f"Drone#{self.id}: Failed to connect to MAVLink: {str(e)}")

    def connect_serial5(self):
        """Connecting to the SERIAL5 TCP port."""
        print(f"Drone#{self.id}: Connecting to SERIAL5 (TCP port {self.serial5_port})...")
        self.serial5_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        while not self.serial5_connected:
            try:
                self.serial5_socket.connect(('127.0.0.1', self.serial5_port))
                self.serial5_connected = True
                print(f"Drone#{self.id}: Connected to SERIAL5 successfully!")
            except Exception as e:
                print(f"Drone#{self.id}: Failed to connect to SERIAL5: {str(e)}")

    def speak_mavlink(self):
        """Starting a thread to update the position"""
        self.position_thread = Thread(target=self.read_mavlink)
        self.position_thread.daemon = True
        self.position_thread.start()

    def speak_serial5(self):
        """Starting a thread to read data"""
        self.serial5_thread = Thread(target=self.read_serial5)
        self.serial5_thread.daemon = True
        self.serial5_thread.start()

    def read_mavlink(self):
        with open(f"./logs/drone_{self.id}_position.csv", "w") as log_file:
            log_file.write(f"time,lat,lon,alt,hdg\n")
            log_file.flush()
            while not self.sync_flag:
                continue
            while self.connected and self.mav_connected:
                try:
                    msg = self.conn.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                    if msg and msg.get_srcSystem() == self.conn.target_system:
                        self.position = (msg.lat, msg.lon, msg.alt, msg.hdg)
                        print(f"Drone#{self.id}: POSITION: {self.position}")
                        log_file.write(f"{time.time_ns()},{msg.lat},{msg.lon},{msg.alt},{msg.hdg}\n")
                        log_file.flush()
                except Exception as e:
                    self.connected = False
                    print(f"Drone#{self.id}: Failed to receive message from MAVlink: {str(e)}")
                    break

    def read_serial5(self):
        """Reading data from SERIAL5."""
        while True:
            try:
                data = self.serial5_socket.recv(32)
                if data:
                    print(f"Drone#{self.id}: SERIAL5 data: {data.hex()}")
                    # Forwarding data to other drones
                    self.forward_data(data)
            except Exception as e:
                self.connected = False
                print(f"Error reading from SERIAL5 for drone {self.id}: {str(e)}")
                break

    def send_data(self, data):
        try:
            self.conn.mav.data_transmission_send(
                type=0,
                size=len(data),
                data=data
            )
        except Exception as e:
            print(f"Error sending to drone {self.id}: {str(e)}")

    def forward_data(self, data):
        """Forwarding data to SERIAL5 of other drones."""
        for drone in self.other_drones:
            if drone.connected and drone.id != self.id:
                if random.random() < self.base_packet_loss:
                    print(f"Drone#{self.id}: Lost data to drone#{drone.id}")
                    continue
                try:
                    # Checking the socket state and reconnecting if necessary
                    if not drone.serial5_socket or drone.serial5_socket.fileno() == -1:
                        print(f"Drone#{self.id}: Reconnecting to drone#{drone.id}...")
                        drone.connect_serial5()

                    drone.serial5_socket.send(data)
                    print(f"Drone#{self.id}: Forwarded data to drone#{drone.id}")
                except Exception as e:
                    print(f"Drone#{self.id}: Failed to forward data to drone#{drone.id}: {str(e)}")
                    # Trying to reconnect when sending the next time
                    drone.connected = False
                    drone.serial5_socket = None

    def speak(self):
        self.speak_mavlink()

        while not self.sync_flag:
            try:
                self.serial5_socket.recv(32)
            except Exception as e:
                print(f"Drone#{self.id}: Error reading from SERIAL5: {str(e)}")
                break

        self.speak_serial5()

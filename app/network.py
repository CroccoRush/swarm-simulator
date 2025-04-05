import json
import math
import random
import time


from drone import Drone


class NetworkSimulator:
    def __init__(self, config_file):
        with open(config_file) as f:
            config = json.load(f)

        self.drones = []
        for drone_cfg in config["drones"]:
            drone = Drone(
                drone_cfg["id"],
                drone_cfg["udp_port"],
                drone_cfg["serial5_port"],
                drone_cfg["initial_position"],
            )
            self.drones.append(drone)

        # Establishing connections between drones
        for drone in self.drones:
            drone.other_drones = [d for d in self.drones if d.id != drone.id]

        self.network_cfg = config["network"]
        self.running = True
        Drone.sync_flag = True

    @staticmethod
    def calculate_distance(pos1, pos2):
        """Calculates the Cartesian distance between two points."""
        return math.sqrt(
            (pos1[0] - pos2[0]) ** 2 +
            (pos1[1] - pos2[1]) ** 2 +
            (pos1[2] - pos2[2]) ** 2
        )

    def delivery_probability(self, distance):
        """Calculates the probability of successful message delivery"""
        if distance > self.network_cfg["max_range"]:
            return 0.0
        return (1 - distance / self.network_cfg["max_range"]) * \
            (1 - self.network_cfg["base_packet_loss"])

    def process_message(self, sender_id, data):
        sender_drone = next(d for d in self.drones if d.id == sender_id)

        # Apply random disconnect
        if random.random() < self.network_cfg["disconnect_probability"]:
            print(f"Drone {sender_id} temporarily disconnected!")
            return

        # Convert binary to hex (simulate ESP-NOW)
        hex_data = data.hex()

        # Simulate broadcast
        for receiver in self.drones:
            if receiver.id == sender_id:
                continue

            distance = self.calculate_distance(
                sender_drone.position,
                receiver.position
            )

            if distance <= self.network_cfg["max_range"]:
                prob = self.delivery_probability(distance)
                if random.random() < prob:
                    binary_data = bytes.fromhex(hex_data)
                    receiver.send_data(binary_data)
                    print(f"Drone {sender_id} -> {receiver.id} [OK]")
                else:
                    print(f"Drone {sender_id} -> {receiver.id} [LOST]")
            else:
                print(f"Drone {sender_id} -> {receiver.id} [OUT OF RANGE]")

    def run(self):
        print("Network simulator started")
        try:
            while self.running:
                for drone in self.drones:
                    data = drone.serial5_socket.recv(1024)
                    if data:
                        self.process_message(drone.id, data)
                time.sleep(1 / self.network_cfg["update_rate_hz"])
        except KeyboardInterrupt:
            self.running = False
            print("Simulator stopped")

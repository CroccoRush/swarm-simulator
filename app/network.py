import json
import math
import queue
import random
import threading
import time

from drone import Drone


class NetworkSimulator:
    def __init__(self, config_file):
        with open(config_file) as f:
            config = json.load(f)

        self.drones = []
        self.message_queue = queue.Queue()
        self.network_thread = None

        network_cgf = config["network"]
        for drone_cfg in config["drones"]:
            drone = Drone(
                drone_cfg["id"],
                drone_cfg["udp_port"],
                drone_cfg["serial5_port"],
                drone_cfg["initial_position"],
                network_cgf["base_packet_loss"],
            )
            self.drones.append(drone)

        # Establishing connections between drones
        for drone in self.drones:
            drone.other_drones = [d for d in self.drones if d.id != drone.id]
            # Link the NetworkSimulator to each drone
            drone.network_simulator = self

        self.network_cfg = config["network"]
        self.running = True
        Drone.sync_flag = True

        self.start_network_simulation()

    @staticmethod
    def calculate_distance(pos1, pos2):
        """Calculates the Cartesian distance between two points."""
        return math.sqrt(
            (pos1[0] - pos2[0]) ** 2
            + (pos1[1] - pos2[1]) ** 2
            + (pos1[2] - pos2[2]) ** 2
        )

    def delivery_probability(self, distance):
        """Calculates the probability of successful message delivery"""
        if distance > self.network_cfg["max_range"]:
            return 0.0
        return (1 - distance / self.network_cfg["max_range"]) * (
            1 - self.network_cfg["base_packet_loss"]
        )

    def queue_message(
        self, sender_id: int, data: bytes, timestamp: float = None
    ):
        """
        Adds a message to the queue for processing by the network model.
        """
        if timestamp is None:
            timestamp = time.time()

        self.message_queue.put(
            {"sender_id": sender_id, "data": data, "timestamp": timestamp}
        )

    def start_network_simulation(self):
        """Starts a thread to process network messages."""
        if self.network_thread is None or not self.network_thread.is_alive():
            self.network_thread = threading.Thread(
                target=self._network_loop, daemon=True
            )
            self.network_thread.start()
            print("Realistic network model activated")

    def stop_network_simulation(self):
        """Stops network simulation."""
        self.running = False
        if self.network_thread and self.network_thread.is_alive():
            self.network_thread.join()

    def _network_loop(self):
        """The main processing cycle of network messages."""
        print("The network simulation is running")
        while self.running:
            try:
                message = self.message_queue.get(timeout=0.1)
                self.process_message(message["sender_id"], message["data"])
                self.message_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Error in network simulation: {e}")
        print("Network simulation stopped")

    def process_message(self, sender_id, data):
        """
        Processes the message based on a realistic network model
        """
        try:
            sender_drone = next(d for d in self.drones if d.id == sender_id)
        except StopIteration:
            print(f"Error: drone#{sender_id} not found")
            return

        # Checking for accidental disconnection of the sender
        if random.random() < self.network_cfg["disconnect_probability"]:
            print(f"Drone#{sender_id} temporarily disabled!")
            return

        # Statistics for logging
        successful_deliveries = 0
        lost_messages = 0
        out_of_range = 0

        # Simulating a broadcast
        for receiver in self.drones:
            if receiver.id == sender_id:
                continue

            # Calculating the distance between the drones
            distance = self.calculate_distance(
                sender_drone.position, receiver.position
            )

            if distance <= self.network_cfg["max_range"]:
                # We check the success of the delivery based on the distance
                prob = self.delivery_probability(distance)
                if random.random() < prob:
                    try:
                        if receiver.serial5_socket and receiver.connected:
                            receiver.serial5_socket.send(data)
                            successful_deliveries += 1
                            print(
                                f"Network: Drone#{sender_id} -> {receiver.id} "
                                f"[DELIVERY] (distance: {distance:.1f}м, "
                                f"probability: {prob:.2f})"
                            )
                        else:
                            print(
                                f"Network: Drone#{receiver.id} unavailable for "
                                "admission"
                            )
                    except Exception as e:
                        print(
                            "Network:Delivery error to the "
                            f"drone#{receiver.id}: {e}"
                        )
                else:
                    lost_messages += 1
                    print(
                        f"Network:Drone#{sender_id} -> {receiver.id} [LOSS] "
                        f"(distance: {distance:.1f}м, probability: {prob:.2f})"
                    )
            else:
                out_of_range += 1
                print(
                    f"Network:Drone#{sender_id} -> {receiver.id} [OUT OF AREA] "
                    f"(distance: {distance:.1f}м > "
                    f"{self.network_cfg['max_range']}м)"
                )

        # Logging general statistics
        if successful_deliveries > 0 or lost_messages > 0 or out_of_range > 0:
            print(
                f"Network: Drone#{sender_id} sent a message: "
                f"{successful_deliveries} delivered, {lost_messages} lost, "
                f"{out_of_range} out of range"
            )

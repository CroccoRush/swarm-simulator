#!/usr/bin/env python3
"""
Automatic configuration Generator for N drones
Enables optimal allocation of ports and positions
"""

import argparse
import json
import math
import os
from typing import Literal


class ConfigGenerator:
    def __init__(self):
        # Basic ports for different services
        self.base_udp_port = 14500
        self.base_serial5_port = 5765
        self.base_master_port = 5760
        self.base_sitl_port = 5501

        # Default Network Settings
        self.default_network_config = {
            "max_range": 1000.0,
            "base_packet_loss": 0.1,
            "disconnect_probability": 0.05,
            "update_rate_hz": 10,
        }

        # Default Formation settings
        self.default_formation = {
            "center_lat": 59.756450,
            "center_lon": 30.200250,
            "center_alt": 10,
            "spacing": 50,  # meters between drones
        }

    @staticmethod
    def check_port_availability(port: int, port_range: int = 100) -> bool:
        """Checks the availability of a range of ports."""
        import socket

        for p in range(port, port + port_range):
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.bind(("localhost", p))
                sock.close()
            except OSError:
                return False
        return True

    @classmethod
    def find_available_port_range(cls, start_port: int, num_ports: int) -> int:
        """Finds the first available range of ports."""
        port = start_port
        while not cls.check_port_availability(
            port, num_ports * 20  # 20 ports per drone for stock
        ):
            port += num_ports * 20
            if port > 65000:
                raise RuntimeError("Couldn't find available ports")
        return port

    @staticmethod
    def generate_grid_formation(
        num_drones: int,
        spacing: float,
        center: tuple[float, float, float],
    ) -> list[dict]:
        """Generates a grid formation of drones."""
        positions = []
        center_lat, center_lon, center_alt = center

        # Calculating the grid size
        grid_size = math.ceil(math.sqrt(num_drones))

        # Converting meters to degrees
        lat_per_meter = 1 / 111320
        lon_per_meter = 1 / (111320 * math.cos(math.radians(center_lat)))

        for i in range(num_drones):
            row = i // grid_size
            col = i % grid_size

            # Centering the grid
            offset_row = (row - (grid_size - 1) / 2) * spacing
            offset_col = (col - (grid_size - 1) / 2) * spacing

            lat = center_lat + offset_row * lat_per_meter
            lon = center_lon + offset_col * lon_per_meter

            positions.append(
                {"lat": round(lat, 6), "lon": round(lon, 6), "alt": center_alt}
            )

        return positions

    @staticmethod
    def generate_circle_formation(
        num_drones: int,
        radius: float,
        center: tuple[float, float, float],
    ) -> list[dict]:
        """Generates a circular formation of drones."""
        positions = []
        center_lat, center_lon, center_alt = center

        # Converting meters to degrees
        lat_per_meter = 1 / 111320
        lon_per_meter = 1 / (111320 * math.cos(math.radians(center_lat)))

        for i in range(num_drones):
            angle = 2 * math.pi * i / num_drones

            offset_lat = radius * math.cos(angle) * lat_per_meter
            offset_lon = radius * math.sin(angle) * lon_per_meter

            lat = center_lat + offset_lat
            lon = center_lon + offset_lon

            positions.append(
                {"lat": round(lat, 6), "lon": round(lon, 6), "alt": center_alt}
            )

        return positions

    def generate_config(
        self,
        num_drones: int,
        formation_type: Literal["grid", "circle"] = "grid",
        spacing: float = 50.0,
        network_config: dict | None = None,
        output_file: str | None = None,
        check_ports: bool = True,
    ) -> dict:
        """
        Generates a configuration for a given number of drones.
        """
        if network_config is None:
            network_config = self.default_network_config.copy()

        # We check the availability of ports if necessary
        if check_ports:
            try:
                base_port = self.find_available_port_range(
                    self.base_udp_port, num_drones
                )
                if base_port != self.base_udp_port:
                    print(
                        "We use an alternative range of ports"
                        f"beginning with {base_port}"
                    )
                    self.base_udp_port = base_port
                    self.base_serial5_port = base_port + 265
            except RuntimeError as e:
                print(f"Warning: {e}. We continue with the base ports.")

        # Generating positions
        center = (
            self.default_formation["center_lat"],
            self.default_formation["center_lon"],
            self.default_formation["center_alt"],
        )

        if formation_type == "grid":
            positions = self.generate_grid_formation(
                num_drones, spacing, center
            )
        elif formation_type == "circle":
            positions = self.generate_circle_formation(
                num_drones, spacing, center
            )
        else:
            raise ValueError(f"Unknown formation type: {formation_type}")

        # Generating the drone configuration
        drones = []
        for i in range(num_drones):
            drone_config = {
                "id": i,
                "udp_port": self.base_udp_port + i * 10,
                "serial5_port": self.base_serial5_port + i * 10,
                "initial_position": positions[i],
            }
            drones.append(drone_config)

        config = {"drones": drones, "network": network_config}

        # Save it to a file if specified
        if output_file:
            os.makedirs(
                (
                    os.path.dirname(output_file)
                    if os.path.dirname(output_file)
                    else "."
                ),
                exist_ok=True,
            )
            with open(output_file, "w") as f:
                json.dump(config, f, indent=4)
            print(f"The configuration is saved in {output_file}")

        return config

    @staticmethod
    def generate_parameter_files(
        num_drones: int,
        base_param_file: str = "mav.parm",
        output_dir: str = "params",
    ):
        """Generates parameter files for each drone."""
        if not os.path.exists(base_param_file):
            print(
                "Warning: the basic parameter file "
                f"{base_param_file} not found"
            )
            return

        os.makedirs(output_dir, exist_ok=True)

        # Reading the basic parameter file
        with open(base_param_file) as f:
            base_params = f.read()

        for i in range(num_drones):
            output_file = os.path.join(output_dir, f"copter_{i}.parm")

            # Modifying the parameters for each drone
            modified_params = base_params

            # Setting the unique SYSID_THISMAV
            if "SYSID_THISMAV" in modified_params:
                modified_params = modified_params.replace(
                    f"SYSID_THISMAV,1", f"SYSID_THISMAV,{i + 1}"
                )
            else:
                modified_params += f"\nSYSID_THISMAV,{i + 1}\n"

            with open(output_file, "w") as f:
                f.write(modified_params)

        print(
            f"Generated parameter files for {num_drones} drones in "
            f"{output_dir}/"
        )


def main():
    parser = argparse.ArgumentParser(
        description="Configuration Generator for Drone Swarm Simulator"
    )

    parser.add_argument("num_drones", type=int, help="Number of drones")
    parser.add_argument(
        "--formation",
        choices=["grid", "circle"],
        default="grid",
        help="Formation type",
    )
    parser.add_argument(
        "--spacing",
        type=float,
        default=50.0,
        help="The distance between the drones in meters",
    )
    parser.add_argument(
        "--output",
        "-o",
        default=None,
        help="The output file "
        "(by default: config/config_{num_drones}_drones.json)",
    )
    parser.add_argument(
        "--no-port-check",
        action="store_true",
        help="Do not check the availability of ports",
    )
    parser.add_argument(
        "--generate-params",
        action="store_true",
        help="Generate parameter files",
    )
    parser.add_argument(
        "--packet-loss",
        type=float,
        default=0.1,
        help="The basic probability of packet loss",
    )
    parser.add_argument(
        "--max-range",
        type=float,
        default=1000.0,
        help="Maximum communication range in meters",
    )

    args = parser.parse_args()

    if args.output is None:
        args.output = f"config/config_{args.num_drones}_drones.json"

    generator = ConfigGenerator()

    # Configuring network settings
    network_config = generator.default_network_config.copy()
    network_config["base_packet_loss"] = args.packet_loss
    network_config["max_range"] = args.max_range

    # Configuration generation
    generator.generate_config(
        num_drones=args.num_drones,
        formation_type=args.formation,
        spacing=args.spacing,
        network_config=network_config,
        output_file=args.output,
        check_ports=not args.no_port_check,
    )

    print(f"A configuration has been created for {args.num_drones} drones")
    print(f"Formation: {args.formation}, spacing: {args.spacing}м")
    print(
        f"Network Settings: loss {args.packet_loss}, distance "
        f"{args.max_range}м"
    )

    # Generating parameter files
    if args.generate_params:
        generator.generate_parameter_files(args.num_drones)


if __name__ == "__main__":
    main()

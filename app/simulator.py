from network import NetworkSimulator
from gamepad import DroneControlGUI


if __name__ == "__main__":
    sim = NetworkSimulator("./config.json")
    gui = DroneControlGUI(sim)
    gui.run()

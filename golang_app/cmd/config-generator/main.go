package main

import (
	"flag"
	"fmt"
	"log"
	"math"
	"swarm-simulator/internal/config"
)

func main() {
	var (
		numDrones       = flag.Int("drones", 4, "Number of drones")
		formation       = flag.String("formation", "grid", "Formation type: grid or circle")
		spacing         = flag.Float64("spacing", 50.0, "Spacing between drones in meters")
		maxRange        = flag.Float64("range", 1000.0, "Maximum communication range in meters")
		lossRate        = flag.Float64("loss", 0.1, "Base packet loss rate (0.0-1.0)")
		output          = flag.String("output", "", "Output file (default: config_N_drones.json)")
		verbose         = flag.Bool("verbose", false, "Verbose output")
		serial5Type     = flag.String("serial5-type", "tcp", "Serial5 connection type: tcp or unix")
		serial5BasePath = flag.String("serial5-base-path", "/tmp/swarm_sitl", "Base path for Unix sockets (only for unix type)")
		messageSize     = flag.Int("message-size", 32, "Size of messages for data connection in bytes")
	)

	flag.Parse()

	if *output == "" {
		*output = fmt.Sprintf("config_%d_drones.json", *numDrones)
	}

	// Validate serial5-type
	if *serial5Type != "tcp" && *serial5Type != "unix" {
		log.Fatalf("Invalid serial5-type: %s. Must be 'tcp' or 'unix'", *serial5Type)
	}

	cfg := generateConfig(
		*numDrones, *formation, *spacing, *maxRange, *lossRate, *serial5Type, *serial5BasePath, *messageSize,
	)

	if err := config.SaveConfig(cfg, *output); err != nil {
		log.Fatalf("Failed to save config: %v", err)
	}

	fmt.Printf("Generated configuration for %d drones\n", *numDrones)
	fmt.Printf("Saved to: %s\n", *output)
	fmt.Printf("Formation: %s (spacing: %.1fm)\n", *formation, *spacing)
	fmt.Printf("Network: range=%.0fm, loss=%.1f%%\n", *maxRange, *lossRate*100)
	fmt.Printf("Message Size: %d bytes\n", *messageSize)
	fmt.Printf("Serial5: %s", *serial5Type)
	if *serial5Type == "unix" {
		fmt.Printf(" (base path: %s)", *serial5BasePath)
	}
	fmt.Println()

	if *verbose {
		printConfigDetails(cfg)
	}
}

func generateConfig(
	numDrones int,
	formation string,
	spacing, maxRange, lossRate float64,
	serial5Type, serial5BasePath string,
	messageSize int,
) *config.Config {
	cfg := &config.Config{
		Drones: make([]config.DroneConfig, numDrones),
		Network: config.NetworkConfig{
			MaxRange:              maxRange,
			BasePacketLoss:        lossRate,
			DisconnectProbability: 0.05,
			UpdateRateHz:          10,
			ChannelBufferSize:     1000,
			MaxConcurrentMsgs:     100,
		},
		MessageSize: messageSize,
	}

	// Generate positions based on formation
	positions := generatePositions(numDrones, formation, spacing)

	// Generate drone configurations
	for i := 0; i < numDrones; i++ {
		var serial5Config config.Serial5Config

		switch serial5Type {
		case "tcp":
			port := 5765 + i*10
			serial5Config = config.Serial5Config{
				Type: "tcp",
				Port: &port,
			}
		case "unix":
			socketPath := fmt.Sprintf("%s_%d.sock", serial5BasePath, i)
			serial5Config = config.Serial5Config{
				Type: "unix",
				Path: &socketPath,
			}
		}

		cfg.Drones[i] = config.DroneConfig{
			ID:              i,
			UDPPort:         14500 + i*10,
			Serial5:         serial5Config,
			InitialPosition: positions[i],
		}
	}

	return cfg
}

func generatePositions(
	numDrones int, formation string, spacing float64,
) []config.Position {
	basePos := config.Position{
		Lat: 0.0001,
		Lon: 0.0001,
		Alt: 0,
	}

	positions := make([]config.Position, numDrones)

	switch formation {
	case "grid":
		positions = generateGridFormation(numDrones, spacing, basePos)
	case "circle":
		positions = generateCircleFormation(numDrones, spacing, basePos)
	default:
		log.Fatalf("Unknown formation: %s", formation)
	}

	return positions
}

func generateGridFormation(
	numDrones int, spacing float64, center config.Position,
) []config.Position {
	positions := make([]config.Position, numDrones)
	gridSize := int(math.Ceil(math.Sqrt(float64(numDrones))))

	// Convert meters to degrees
	latPerMeter := 1.0 / 111320.0
	lonPerMeter := 1.0 / (111320.0 * math.Cos(center.Lat*math.Pi/180))

	for i := 0; i < numDrones; i++ {
		row := i / gridSize
		col := i % gridSize

		// Center the grid
		offsetRow := (float64(row) - (float64(gridSize)-1)/2) * spacing
		offsetCol := (float64(col) - (float64(gridSize)-1)/2) * spacing

		positions[i] = config.Position{
			Lat: center.Lat + offsetRow*latPerMeter,
			Lon: center.Lon + offsetCol*lonPerMeter,
			Alt: center.Alt,
		}
	}

	return positions
}

func generateCircleFormation(
	numDrones int, radius float64, center config.Position,
) []config.Position {
	positions := make([]config.Position, numDrones)

	// Convert meters to degrees
	latPerMeter := 1.0 / 111320.0
	lonPerMeter := 1.0 / (111320.0 * math.Cos(center.Lat*math.Pi/180))

	for i := 0; i < numDrones; i++ {
		angle := 2 * math.Pi * float64(i) / float64(numDrones)

		offsetLat := radius * math.Cos(angle) * latPerMeter
		offsetLon := radius * math.Sin(angle) * lonPerMeter

		positions[i] = config.Position{
			Lat: center.Lat + offsetLat,
			Lon: center.Lon + offsetLon,
			Alt: center.Alt,
		}
	}

	return positions
}

func printConfigDetails(cfg *config.Config) {
	fmt.Println("\nConfiguration Details:")
	fmt.Println("========================")

	fmt.Printf("Drones (%d):\n", len(cfg.Drones))

	for _, drone := range cfg.Drones {
		var serial5Info string
		switch drone.Serial5.Type {
		case "tcp":
			if drone.Serial5.Port != nil {
				serial5Info = fmt.Sprintf("tcp:%d", *drone.Serial5.Port)
			} else {
				serial5Info = "tcp:N/A"
			}
		case "unix":
			if drone.Serial5.Path != nil {
				serial5Info = fmt.Sprintf("unix:%s", *drone.Serial5.Path)
			} else {
				serial5Info = "unix:N/A"
			}
		default:
			serial5Info = "unknown"
		}

		fmt.Printf("  ID %d: UDP=%d, Serial5=%s, Pos=(%.6f,%.6f,%.1f)\n",
			drone.ID, drone.UDPPort, serial5Info,
			drone.InitialPosition.Lat, drone.InitialPosition.Lon, drone.InitialPosition.Alt)
	}

	fmt.Printf("\nNetwork:\n")
	fmt.Printf("  Max Range:       %.1fm\n", cfg.Network.MaxRange)
	fmt.Printf("  Base Loss:       %.1f%%\n", cfg.Network.BasePacketLoss*100)
	fmt.Printf("  Disconnect Prob: %.1f%%\n", cfg.Network.DisconnectProbability*100)
	fmt.Printf("  Update Rate:     %d Hz\n", cfg.Network.UpdateRateHz)
	fmt.Printf("  Channel Buffer:  %d\n", cfg.Network.ChannelBufferSize)
	fmt.Printf("  Max Concurrent:  %d\n", cfg.Network.MaxConcurrentMsgs)
	fmt.Printf("  Message Size:    %d bytes\n", cfg.MessageSize)

	// Calculate port/socket ranges
	if len(cfg.Drones) > 0 {
		minUDP := cfg.Drones[0].UDPPort
		maxUDP := cfg.Drones[len(cfg.Drones)-1].UDPPort

		fmt.Printf("\nConnection Info:\n")
		fmt.Printf("  UDP Ports: %d - %d\n", minUDP, maxUDP)

		// Show Serial5 connection info
		firstDrone := cfg.Drones[0]
		lastDrone := cfg.Drones[len(cfg.Drones)-1]

		switch firstDrone.Serial5.Type {
		case "tcp":
			if firstDrone.Serial5.Port != nil && lastDrone.Serial5.Port != nil {
				minSerial5 := *firstDrone.Serial5.Port
				maxSerial5 := *lastDrone.Serial5.Port
				fmt.Printf("  Serial5 TCP Ports: %d - %d\n", minSerial5, maxSerial5)
			}
		case "unix":
			if firstDrone.Serial5.Path != nil && lastDrone.Serial5.Path != nil {
				fmt.Printf(
					"  Serial5 Unix Sockets: %s ... %s\n",
					*firstDrone.Serial5.Path, *lastDrone.Serial5.Path,
				)
			}
		}
	}
}

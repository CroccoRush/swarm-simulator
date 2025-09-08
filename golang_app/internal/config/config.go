package config

import (
	"encoding/json"
	"fmt"
	"os"
)

// Serial5Config represents Serial5 connection configuration
type Serial5Config struct {
	Type string  `json:"type"`           // "tcp" or "unix"
	Port *int    `json:"port,omitempty"` // TCP port (used when type is "tcp")
	Path *string `json:"path,omitempty"` // Unix socket path (used when type is "unix")
}

// Info returns a string description of Serial5 connection
func (c *Serial5Config) Info() string {
	switch c.Type {
	case "tcp":
		if c.Port != nil {
			return fmt.Sprintf("tcp:%d", *c.Port)
		}

		return "tcp:5765" // Default port
	case "unix":
		if c.Path != nil {
			return fmt.Sprintf("unix:%s", *c.Path)
		}

		return "unix:/tmp/default.sock" // Default path
	default:
		return "tcp:5765" // Default
	}
}

// Address returns the network and address for Serial5 connection
func (c *Serial5Config) Address() (network, address string) {
	switch c.Type {
	case "tcp":
		port := 5765 // Default port
		if c.Port != nil {
			port = *c.Port
		}

		return "tcp", fmt.Sprintf("127.0.0.1:%d", port)
	case "unix":
		path := "/tmp/default.sock" // Default path
		if c.Path != nil {
			path = *c.Path
		}

		return "unix", path
	default:
		return "tcp", "127.0.0.1:5765" // Default
	}
}

// DroneConfig represents configuration for a single drone
type DroneConfig struct {
	ID              int           `json:"id"`
	UDPPort         int           `json:"udp_port"`
	Serial5         Serial5Config `json:"serial5"`
	InitialPosition Position      `json:"initial_position"`
}

// Position represents a GPS position
type Position struct {
	Lat float64 `json:"lat"`
	Lon float64 `json:"lon"`
	Alt float64 `json:"alt"`
}

// NetworkConfig represents network simulation parameters
type NetworkConfig struct {
	MaxRange              float64 `json:"max_range"`              // Maximum communication range in meters
	BasePacketLoss        float64 `json:"base_packet_loss"`       // Base packet loss probability (0.0-1.0)
	DisconnectProbability float64 `json:"disconnect_probability"` // Probability of temporary disconnection
	UpdateRateHz          int     `json:"update_rate_hz"`         // Network update frequency
	ChannelBufferSize     int     `json:"channel_buffer_size"`    // Go channel buffer size
	MaxConcurrentMsgs     int     `json:"max_concurrent_msgs"`    // Maximum concurrent message processing
}

// Config represents the complete simulator configuration
type Config struct {
	Drones  []DroneConfig `json:"drones"`
	Network NetworkConfig `json:"network"`
}

// LoadConfig loads configuration from a JSON file
func LoadConfig(filename string) (*Config, error) {
	data, err := os.ReadFile(filename)
	if err != nil {
		return nil, err
	}

	var config Config
	if err := json.Unmarshal(data, &config); err != nil {
		return nil, err
	}

	// Set defaults for new Go-specific parameters
	if config.Network.ChannelBufferSize == 0 {
		config.Network.ChannelBufferSize = 1000
	}

	if config.Network.MaxConcurrentMsgs == 0 {
		config.Network.MaxConcurrentMsgs = 100
	}

	// Set defaults for Serial5 configuration if not specified
	for i := range config.Drones {
		drone := &config.Drones[i]

		// If Serial5 config is empty, set TCP defaults
		if drone.Serial5.Type == "" {
			drone.Serial5.Type = "tcp"
			defaultPort := 5765 + drone.ID
			drone.Serial5.Port = &defaultPort
		}
	}

	return &config, nil
}

// SaveConfig saves configuration to a JSON file
func SaveConfig(config *Config, filename string) error {
	data, err := json.MarshalIndent(config, "", "  ")
	if err != nil {
		return err
	}

	return os.WriteFile(filename, data, 0o644)
}

// DefaultConfig returns a default configuration
func DefaultConfig() *Config {
	return &Config{
		Drones: []DroneConfig{
			{
				ID:      0,
				UDPPort: 14500,
				Serial5: Serial5Config{
					Type: "tcp",
					Port: func() *int { p := 5765; return &p }(),
				},
				InitialPosition: Position{
					Lat: 59.756450,
					Lon: 30.200250,
					Alt: 10,
				},
			},
		},
		Network: NetworkConfig{
			MaxRange:              1000.0,
			BasePacketLoss:        0.1,
			DisconnectProbability: 0.05,
			UpdateRateHz:          10,
			ChannelBufferSize:     1000,
			MaxConcurrentMsgs:     100,
		},
	}
}

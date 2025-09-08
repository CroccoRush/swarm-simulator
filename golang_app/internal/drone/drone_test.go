package drone

import (
	"bytes"
	"swarm-simulator/internal/config"
	"testing"
	"time"

	"github.com/sirupsen/logrus"
)

var testLogLevel *string

func init() {
	tmp := logrus.WarnLevel.String()
	testLogLevel = &tmp
}

func TestNewDrone(t *testing.T) {
	cfg := config.DroneConfig{
		ID:      0,
		UDPPort: 14500,
		Serial5: config.Serial5Config{
			Type: "tcp",
			Port: func() *int { p := 5765; return &p }(),
		},
		InitialPosition: config.Position{
			Lat: 59.756450,
			Lon: 30.200250,
			Alt: 30,
		},
	}

	messageCh := make(chan Message, 10)
	drone := NewDrone(cfg, messageCh, testLogLevel)

	if drone.ID != cfg.ID {
		t.Errorf("Expected drone ID %d, got %d", cfg.ID, drone.ID)
	}

	if drone.UDPPort != cfg.UDPPort {
		t.Errorf("Expected UDP port %d, got %d", cfg.UDPPort, drone.UDPPort)
	}

	if drone.Serial5Config.Type != cfg.Serial5.Type {
		t.Errorf("Expected Serial5 type %s, got %s", cfg.Serial5.Type, drone.Serial5Config.Type)
	}

	// Test initial position
	pos := drone.GetPosition()
	expectedLat := cfg.InitialPosition.Lat * 1e7

	if pos.Lat != expectedLat {
		t.Errorf("Expected latitude %f, got %f", expectedLat, pos.Lat)
	}
}

func TestDronePositionUpdate(t *testing.T) {
	cfg := config.DroneConfig{
		ID:      1,
		UDPPort: 14510,
		Serial5: config.Serial5Config{
			Type: "tcp",
			Port: func() *int { p := 5775; return &p }(),
		},
		InitialPosition: config.Position{
			Lat: 59.756450,
			Lon: 30.200250,
			Alt: 30,
		},
	}

	messageCh := make(chan Message, 10)
	drone := NewDrone(cfg, messageCh, testLogLevel)

	// Test initial position
	_ = drone.GetPosition()

	// Simulate position update
	newPos := Position{
		Lat:       59.756500 * 1e7,
		Lon:       30.200300 * 1e7,
		Alt:       35000, // 35m in millimeters
		Heading:   18000, // 180 degrees * 100
		Timestamp: time.Now(),
	}

	drone.position.Store(newPos)

	// Verify position update
	updatedPos := drone.GetPosition()
	if updatedPos.Lat != newPos.Lat {
		t.Errorf("Expected latitude %f, got %f", newPos.Lat, updatedPos.Lat)
	}

	if updatedPos.Lon != newPos.Lon {
		t.Errorf("Expected longitude %f, got %f", newPos.Lon, updatedPos.Lon)
	}

	if updatedPos.Alt != newPos.Alt {
		t.Errorf("Expected altitude %f, got %f", newPos.Alt, updatedPos.Alt)
	}
}

func TestDroneMessageSending(t *testing.T) {
	cfg := config.DroneConfig{
		ID:      2,
		UDPPort: 14520,
		Serial5: config.Serial5Config{
			Type: "tcp",
			Port: func() *int { p := 5785; return &p }(),
		},
		InitialPosition: config.Position{
			Lat: 59.756450,
			Lon: 30.200250,
			Alt: 30,
		},
	}

	messageCh := make(chan Message, 10)
	drone := NewDrone(cfg, messageCh, testLogLevel)
	drone.connected.Store(true) // Simulate connected state

	// Test message sending
	testData := []byte("Hello from drone!")
	drone.SendMessage(testData)

	// Verify message was sent to channel
	select {
	case msg := <-messageCh:
		if msg.SenderID != drone.ID {
			t.Errorf("Expected sender ID %d, got %d", drone.ID, msg.SenderID)
		}

		if !bytes.Equal(msg.Data, testData) {
			t.Errorf("Expected data %s, got %s", string(testData), string(msg.Data))
		}
	case <-time.After(100 * time.Millisecond):
		t.Error("Message was not sent to channel")
	}
}

func TestDroneConnectionState(t *testing.T) {
	cfg := config.DroneConfig{
		ID:      3,
		UDPPort: 14530,
		Serial5: config.Serial5Config{
			Type: "tcp",
			Port: func() *int { p := 5795; return &p }(),
		},
		InitialPosition: config.Position{
			Lat: 59.756450,
			Lon: 30.200250,
			Alt: 30,
		},
	}

	messageCh := make(chan Message, 10)
	drone := NewDrone(cfg, messageCh, testLogLevel)

	// Initial state should be disconnected
	if drone.IsConnected() {
		t.Error("Drone should be initially disconnected")
	}

	// Simulate connection
	drone.connected.Store(true)

	if !drone.IsConnected() {
		t.Error("Drone should be connected after setting connected state")
	}

	// Simulate disconnection
	drone.connected.Store(false)

	if drone.IsConnected() {
		t.Error("Drone should be disconnected after setting disconnected state")
	}
}

// Benchmark tests for performance verification
func BenchmarkDroneMessageSending(b *testing.B) {
	cfg := config.DroneConfig{
		ID:      0,
		UDPPort: 14500,
		Serial5: config.Serial5Config{
			Type: "tcp",
			Port: func() *int { p := 5765; return &p }(),
		},
		InitialPosition: config.Position{
			Lat: 59.756450,
			Lon: 30.200250,
			Alt: 30,
		},
	}

	messageCh := make(chan Message, 1000)
	drone := NewDrone(cfg, messageCh, testLogLevel)
	drone.connected.Store(true)

	testData := []byte("Benchmark message data")

	b.ResetTimer()
	b.RunParallel(func(pb *testing.PB) {
		for pb.Next() {
			drone.SendMessage(testData)
		}
	})
}

func BenchmarkDronePositionUpdate(b *testing.B) {
	cfg := config.DroneConfig{
		ID:      0,
		UDPPort: 14500,
		Serial5: config.Serial5Config{
			Type: "tcp",
			Port: func() *int { p := 5765; return &p }(),
		},
		InitialPosition: config.Position{
			Lat: 59.756450,
			Lon: 30.200250,
			Alt: 30,
		},
	}

	messageCh := make(chan Message, 10)
	drone := NewDrone(cfg, messageCh, testLogLevel)

	pos := Position{
		Lat:       59.756500 * 1e7,
		Lon:       30.200300 * 1e7,
		Alt:       35000,
		Heading:   18000,
		Timestamp: time.Now(),
	}

	b.ResetTimer()
	b.RunParallel(func(pb *testing.PB) {
		for pb.Next() {
			drone.position.Store(pos)
			_ = drone.GetPosition()
		}
	})
}

// Test basic drone functionality
func TestDroneBasicFunctionality(t *testing.T) {
	cfg := config.DroneConfig{
		ID:      4,
		UDPPort: 14540,
		Serial5: config.Serial5Config{
			Type: "tcp",
			Port: func() *int { p := 5805; return &p }(),
		},
		InitialPosition: config.Position{
			Lat: 59.756450,
			Lon: 30.200250,
			Alt: 30,
		},
	}

	messageCh := make(chan Message, 10)

	// Test basic drone creation
	drone := NewDrone(cfg, messageCh, testLogLevel)
	if drone.ID != cfg.ID {
		t.Errorf("Expected ID %d, got %d", cfg.ID, drone.ID)
	}

	if drone.UDPPort != cfg.UDPPort {
		t.Errorf("Expected UDP port %d, got %d", cfg.UDPPort, drone.UDPPort)
	}
}

// Test new Serial5 configuration structure
func TestDroneSerial5Config(t *testing.T) {
	// Test TCP Serial5 configuration
	cfgTCP := config.DroneConfig{
		ID:      5,
		UDPPort: 14550,
		Serial5: config.Serial5Config{
			Type: "tcp",
			Port: func() *int { p := 5815; return &p }(),
		},
		InitialPosition: config.Position{
			Lat: 59.756450,
			Lon: 30.200250,
			Alt: 30,
		},
	}

	messageCh := make(chan Message, 10)

	droneTCP := NewDrone(cfgTCP, messageCh, testLogLevel)
	if droneTCP.Serial5Config.Type != "tcp" {
		t.Errorf("Expected Serial5 type 'tcp', got '%s'", droneTCP.Serial5Config.Type)
	}

	if droneTCP.Serial5Config.Port == nil || *droneTCP.Serial5Config.Port != 5815 {
		if droneTCP.Serial5Config.Port == nil {
			t.Error("Expected Serial5 port 5815, got nil")
		} else {
			t.Errorf("Expected Serial5 port 5815, got %d", *droneTCP.Serial5Config.Port)
		}
	}

	// Test Unix Socket Serial5 configuration
	cfgUnix := config.DroneConfig{
		ID:      6,
		UDPPort: 14560,
		Serial5: config.Serial5Config{
			Type: "unix",
			Path: func() *string { p := "/tmp/test_socket.sock"; return &p }(),
		},
		InitialPosition: config.Position{
			Lat: 59.756450,
			Lon: 30.200250,
			Alt: 30,
		},
	}

	droneUnix := NewDrone(cfgUnix, messageCh, testLogLevel)
	if droneUnix.Serial5Config.Type != "unix" {
		t.Errorf("Expected Serial5 type 'unix', got '%s'", droneUnix.Serial5Config.Type)
	}

	if droneUnix.Serial5Config.Path == nil || *droneUnix.Serial5Config.Path != "/tmp/test_socket.sock" {
		if droneUnix.Serial5Config.Path == nil {
			t.Error("Expected Serial5 path '/tmp/test_socket.sock', got nil")
		} else {
			t.Errorf("Expected Serial5 path '/tmp/test_socket.sock', got '%s'", *droneUnix.Serial5Config.Path)
		}
	}
}

package drone

import (
	"context"
	"errors"
	"fmt"
	"net"
	"swarm-simulator/internal/config"
	"swarm-simulator/internal/logger"
	"swarm-simulator/internal/mavlink"
	"sync"
	"sync/atomic"
	"time"

	"github.com/sirupsen/logrus"
)

var (
	messagePool = sync.Pool{
		New: func() interface{} {
			return new(Message)
		},
	}
)

// Drone represents a single drone in the swarm
type Drone struct {
	// Basic configuration
	ID              int
	UDPPort         int
	Serial5Config   config.Serial5Config
	InitialPosition config.Position

	// Current state
	position     atomic.Value // stores Position
	connected    atomic.Bool
	mavConnected atomic.Bool

	// Network connections
	mavlinkConn *mavlink.Connection
	dataConn    net.Conn // Connection for inter-drone data (Serial5, TCP, or via QEMU)

	// Communication channels
	messageTx  chan<- Message // Send messages to network simulator
	messageRx  chan Message   // Receive messages from network simulator
	controlCh  chan ControlCommand
	shutdownCh chan struct{}

	// Concurrency control
	wg     sync.WaitGroup
	mu     sync.RWMutex
	logger *logrus.Entry
}

// Message represents a message sent between drones
type Message struct {
	SenderID  int
	Data      []byte
	Timestamp time.Time
}

// ControlCommand represents various control commands
type ControlCommand struct {
	Type string
	Data any
}

// Position represents current drone position
type Position struct {
	Lat       float64 // Latitude in degrees * 1e7 (MAVLink format)
	Lon       float64 // Longitude in degrees * 1e7 (MAVLink format)
	Alt       float64 // Altitude in millimeters
	Heading   float64 // Heading in degrees * 100
	Timestamp time.Time
}

// NewDrone creates a new drone instance
func NewDrone(
	cfg config.DroneConfig,
	messageTx chan<- Message,
	logLevel *string,
) *Drone {
	log := logger.NewLogger(logLevel, "DRONE").WithField("id", cfg.ID)
	d := &Drone{
		ID:              cfg.ID,
		UDPPort:         cfg.UDPPort,
		Serial5Config:   cfg.Serial5,
		InitialPosition: cfg.InitialPosition,
		messageTx:       messageTx,
		messageRx:       make(chan Message, 100),
		controlCh:       make(chan ControlCommand, 10),
		shutdownCh:      make(chan struct{}),
		logger:          log,
	}

	// Set initial position
	d.position.Store(Position{
		Lat:       cfg.InitialPosition.Lat * 1e7,
		Lon:       cfg.InitialPosition.Lon * 1e7,
		Alt:       cfg.InitialPosition.Alt * 1000,
		Heading:   0,
		Timestamp: time.Now(),
	})

	return d
}

// Start starts all drone subsystems
func (d *Drone) Start(ctx context.Context) error {
	serial5Info := d.Serial5Config.Info()
	d.logger.Infof("Starting (MAVLink UDP:%d, Data:%s)", d.UDPPort, serial5Info)

	// Start data connection (Serial5, TCP, or via QEMU)
	if err := d.startConnection(ctx); err != nil {
		return fmt.Errorf("failed to start data connection: %w", err)
	}

	// Start MAVLink connection
	if err := d.startMAVLink(ctx); err != nil {
		return fmt.Errorf("failed to start MAVLink: %w", err)
	}

	// Start main processing loops
	d.wg.Add(4)
	go d.dataHandler(ctx)
	go d.mavlinkReader(ctx)
	go d.messageProcessor(ctx)
	go d.controlHandler(ctx)

	d.connected.Store(true)
	d.logger.Info("Started successfully")

	return nil
}

// Stop gracefully stops the drone
func (d *Drone) Stop(ctx context.Context) error {
	d.logger.Info("Stopping...")

	d.connected.Store(false)
	close(d.shutdownCh)

	// Wait for goroutines to finish or timeout
	done := make(chan struct{})
	go func() {
		d.wg.Wait()
		close(done)
	}()

	select {
	case <-done:
		d.logger.Info("Stopped gracefully")
	case <-ctx.Done():
		d.logger.Warn("Stop timeout")
	}

	// Close connections
	if d.mavlinkConn != nil {
		d.mavlinkConn.Close()
	}

	if d.dataConn != nil {
		d.dataConn.Close()
	}

	return nil
}

// GetPosition returns current drone position
func (d *Drone) GetPosition() Position {
	return d.position.Load().(Position)
}

// IsConnected returns connection status
func (d *Drone) IsConnected() bool {
	return d.connected.Load()
}

// SendMessage sends a message to the network
func (d *Drone) SendMessage(data []byte) {
	if !d.IsConnected() {
		return
	}

	msg := messagePool.Get().(*Message)
	msg.SenderID = d.ID
	msg.Data = data
	msg.Timestamp = time.Now()

	select {
	case d.messageTx <- *msg:
		d.logger.Debugf("Message sent: %x", data)
	default:
		d.logger.Warn("Message queue full, dropping message")
		messagePool.Put(msg) // Put the message back in the pool if dropped
	}
}

// ReceiveMessage processes incoming messages
func (d *Drone) ReceiveMessage(msg Message) {
	select {
	case d.messageRx <- msg:
	default:
		d.logger.Warnf("Receive queue full, dropping message from %d", msg.SenderID)
		// Since the message is dropped, put it back in the pool
		messagePool.Put(&msg)
	}
}

// startMAVLink initializes MAVLink connection
func (d *Drone) startMAVLink(ctx context.Context) error {
	d.logger.Infof(
		"Starting MAVLink UDP server on 0.0.0.0:%d (waiting for MAVProxy)...",
		d.UDPPort,
	)

	conn, err := mavlink.NewConnection(d.UDPPort, d.ID+1)
	if err != nil {
		d.logger.Errorf("MAVLink connection failed: %v", err)
		d.logger.Warnf(
			"Make sure MAVProxy is running and connecting to UDP port %d",
			d.UDPPort,
		)

		return err
	}

	d.mavlinkConn = conn
	d.mavConnected.Store(true)
	d.logger.Info("MAVLink connected (received heartbeat)")

	return nil
}

// startConnection connects to data source (Serial5, TCP, or QEMU)
func (d *Drone) startConnection(ctx context.Context) error {
	// Try to connect to configured data source
	// This could be:
	// 1. Direct Serial5 connection to SITL (tcp/unix)
	// 2. TCP connection to QEMU ESP32 emulator (which connects to SITL via Serial5)
	network, address := d.Serial5Config.Address()
	d.logger.Infof("Connecting to data source on %s://%s...", network, address)

	conn, err := net.Dial(network, address)
	if err != nil {
		d.logger.Errorf("Data connection failed: %v", err)

		switch network {
		case "tcp":
			port := 5765
			if d.Serial5Config.Port != nil {
				port = *d.Serial5Config.Port
			}

			d.logger.Infof(
				"Make sure the data source (SITL or QEMU) is listening on TCP port %d",
				port,
			)
		case "unix":
			path := "/tmp/default.sock"
			if d.Serial5Config.Path != nil {
				path = *d.Serial5Config.Path
			}

			d.logger.Infof(
				"Make sure the data source (SITL) is listening on Unix socket %s",
				path,
			)
		}

		return err
	}

	d.mu.Lock()
	d.dataConn = conn
	d.mu.Unlock()

	d.logger.Infof("Data connection established via %s", network)

	return nil
}

// mavlinkReader reads position updates from MAVLink
func (d *Drone) mavlinkReader(ctx context.Context) {
	defer d.wg.Done()

	ticker := time.NewTicker(100 * time.Millisecond)
	defer ticker.Stop()

	lastConnectionStatus := true

	for {
		select {
		case <-ctx.Done():
			return
		case <-d.shutdownCh:
			return
		case <-ticker.C:
			if d.mavlinkConn == nil {
				continue
			}

			// Check connection status
			currentStatus := d.mavlinkConn.IsConnected()

			if currentStatus != lastConnectionStatus {
				if currentStatus {
					d.logger.Info("MAVLink reconnected")
				} else {
					d.logger.Warn("MAVLink disconnected (no heartbeat)")
				}

				lastConnectionStatus = currentStatus
				d.mavConnected.Store(currentStatus)
			}

			if !currentStatus {
				continue
			}

			pos, err := d.mavlinkConn.ReadPosition()
			if err != nil {
				d.logger.Warnf("MAVLink read error: %v", err)
				continue
			}

			if pos != nil {
				// Convert mavlink.Position to drone.Position
				dronePos := Position{
					Lat:       pos.Lat,
					Lon:       pos.Lon,
					Alt:       pos.Alt,
					Heading:   pos.Heading,
					Timestamp: pos.Timestamp,
				}
				d.position.Store(dronePos)
				d.logger.Debugf(
					"Position: %.6f,%.6f,%.1f",
					pos.Lat/1e7, pos.Lon/1e7, pos.Alt/1000,
				)
			}
		}
	}
}

// dataHandler handles data communication (Serial5, TCP, or QEMU)
func (d *Drone) dataHandler(ctx context.Context) {
	defer d.wg.Done()

	buffer := make([]byte, 1024)

	for {
		select {
		case <-ctx.Done():
			return
		case <-d.shutdownCh:
			return
		default:
			d.mu.RLock()
			conn := d.dataConn
			d.mu.RUnlock()

			if conn == nil {
				time.Sleep(100 * time.Millisecond)
				continue
			}

			conn.SetReadDeadline(time.Now().Add(1 * time.Second))

			n, err := conn.Read(buffer)
			if err != nil {
				var netErr net.Error
				if errors.As(err, &netErr) && netErr.Timeout() {
					continue
				}

				d.logger.Warnf("Data connection read error: %v", err)

				d.mu.Lock()
				d.dataConn = nil
				d.mu.Unlock()

				continue
			}

			if n > 0 {
				data := make([]byte, n)
				copy(data, buffer[:n])
				d.logger.Debugf("Received data: %x", data)
				d.SendMessage(data)
			}
		}
	}
}

// messageProcessor handles incoming network messages
func (d *Drone) messageProcessor(ctx context.Context) {
	defer d.wg.Done()

	for {
		select {
		case <-ctx.Done():
			return
		case <-d.shutdownCh:
			return
		case msg := <-d.messageRx:
			d.logger.Debugf("Received message from %d: %x", msg.SenderID, msg.Data)

			// Forward to data connection
			d.mu.RLock()
			conn := d.dataConn
			d.mu.RUnlock()

			if conn != nil {
				conn.SetWriteDeadline(time.Now().Add(1 * time.Second))

				if _, err := conn.Write(msg.Data); err != nil {
					d.logger.Warnf("Data connection write error: %v", err)
				} else {
					d.logger.Debugf("Forwarded to data connection: %x", msg.Data)
				}
			}
			// Put the message back in the pool after processing
			messagePool.Put(&msg)
		}
	}
}

// controlHandler handles control commands
func (d *Drone) controlHandler(ctx context.Context) {
	defer d.wg.Done()

	for {
		select {
		case <-ctx.Done():
			return
		case <-d.shutdownCh:
			return
		case cmd := <-d.controlCh:
			switch cmd.Type {
			case "arm":
				if d.mavlinkConn != nil {
					d.mavlinkConn.Arm()
				}
			case "disarm":
				if d.mavlinkConn != nil {
					d.mavlinkConn.Disarm()
				}
			case "rc_override":
				if d.mavlinkConn != nil && cmd.Data != nil {
					if rcData, ok := cmd.Data.(mavlink.RCOverride); ok {
						d.mavlinkConn.SendRCOverride(rcData)
					}
				}
			case "set_mode":
				if d.mavlinkConn != nil && cmd.Data != nil {
					if mode, ok := cmd.Data.(string); ok {
						d.mavlinkConn.SetMode(mode)
					}
				}
			case "takeoff":
				if d.mavlinkConn != nil && cmd.Data != nil {
					if altitude, ok := cmd.Data.(float64); ok {
						d.mavlinkConn.SendTakeoff(altitude)
					}
				}
			case "land":
				if d.mavlinkConn != nil {
					d.mavlinkConn.SendLand()
				}
			}
		}
	}
}

// SendControlCommand sends a control command to the drone
func (d *Drone) SendControlCommand(cmd ControlCommand) {
	if !d.IsConnected() {
		d.logger.Warn("Not connected, skipping send control command")
		return
	}

	select {
	case d.controlCh <- cmd:
	default:
		d.logger.Warnf("Control queue full, dropping command: %s", cmd.Type)
	}
}

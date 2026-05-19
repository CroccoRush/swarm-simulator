package drone

import (
	"context"
	"errors"
	"fmt"
	"io"
	"net"
	"os"
	"strings"
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
	MessageSize     int

	// Current state
	position          atomic.Value // stores Position
	connected         atomic.Bool
	mavConnected      atomic.Bool
	hasPositionFix    atomic.Bool
	isLoggingPosition atomic.Bool // By default, logging is off until enabled by a script command

	// Network connections
	mavlinkConn *mavlink.Connection
	dataConn    net.Conn // Connection for inter-drone data (Serial5, TCP, or via QEMU)
	logFile     *os.File

	// Communication channels
	messageTx  chan<- Message // Send messages to network simulator
	messageRx  chan Message   // Receive messages from network simulator
	controlCh  chan ControlCommand
	shutdownCh chan struct{}

	// Concurrency control
	wg         sync.WaitGroup
	mu         sync.RWMutex
	logger     *logrus.Entry
	rcOverride mavlink.RCOverride
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
	messageSize int,
) *Drone {
	log := logger.NewLogger(logLevel, "DRONE").WithField("id", cfg.ID)
	d := &Drone{
		ID:              cfg.ID,
		UDPPort:         cfg.UDPPort,
		Serial5Config:   cfg.Serial5,
		InitialPosition: cfg.InitialPosition,
		MessageSize:     messageSize,
		messageTx:       messageTx,
		messageRx:       make(chan Message, 100),
		controlCh:       make(chan ControlCommand, 10),
		shutdownCh:      make(chan struct{}),
		logger:          log,
		rcOverride: mavlink.RCOverride{
			Channel1: 1500, // Roll center
			Channel2: 1500, // Pitch center
			Channel3: 1500, // Throttle center
			Channel4: 1500, // Yaw center
		},
		isLoggingPosition: atomic.Bool{}, // Initialize the new field
	}

	// Create log file
	if err := os.MkdirAll("logs", 0755); err != nil {
		log.Errorf("Failed to create logs directory: %v", err)
	} else {
		logPath := fmt.Sprintf("logs/drone_%d_position.csv", d.ID)
		file, err := os.Create(logPath)
		if err != nil {
			log.Errorf("Failed to create log file: %v", err)
		} else {
			d.logFile = file
			// Write CSV header
			if _, err := d.logFile.WriteString("time,lat,lon,alt,hdg,mark\n"); err != nil {
				log.Errorf("Failed to write header to log file: %v", err)
			}
		}
	}

	// Set initial position
	d.position.Store(Position{
		Lat:       cfg.InitialPosition.Lat * 1e7,
		Lon:       cfg.InitialPosition.Lon * 1e7,
		Alt:       cfg.InitialPosition.Alt * 1000,
		Heading:   0,
		Timestamp: time.Now(),
	})

	d.isLoggingPosition.Store(false) // By default, logging is off until enabled by a script command

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

	if d.logFile != nil {
		d.logFile.Close()
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

// HasPositionFix returns true if the drone has received at least one position update from SITL
func (d *Drone) HasPositionFix() bool {
	return d.hasPositionFix.Load()
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
				if !d.hasPositionFix.Load() {
					d.hasPositionFix.Store(true)
					d.logger.Info("Position fix acquired from SITL")
				}
				d.logger.Debugf(
					"Position: %.6f,%.6f,%.1f",
					pos.Lat/1e7, pos.Lon/1e7, pos.Alt/1000,
				)
				// Log to file
				if d.logFile != nil && d.isLoggingPosition.Load() {
					logLine := fmt.Sprintf("%d,%d,%d,%d,%d,\n",
						time.Now().UnixNano(),
						int64(pos.Lat),
						int64(pos.Lon),
						int64(pos.Alt),
						int64(pos.Heading),
					)
					if _, err := d.logFile.WriteString(logLine); err != nil {
						d.logger.Warnf("Failed to write to log file: %v", err)
					}
				}
			}
		}
	}
}

// dataHandler handles data communication (Serial5, TCP, or QEMU)
func (d *Drone) dataHandler(ctx context.Context) {
	defer d.wg.Done()

	if d.MessageSize <= 0 {
		d.logger.Warn(
			"MessageSize not configured or invalid, data handler will not run",
		)
		return
	}
	buffer := make([]byte, d.MessageSize)

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
				d.logger.Warn("Data connection closed")
				time.Sleep(100 * time.Millisecond)
				continue
			}

			conn.SetReadDeadline(time.Now().Add(1 * time.Second))

			// Use io.ReadFull to ensure a full message is read
			n, err := io.ReadFull(conn, buffer)
			if err != nil {
				var netErr net.Error
				if errors.As(err, &netErr) && netErr.Timeout() {
					d.logger.Warnf("Data connection read timeout: %v", err)
					continue
				}

				d.logger.Warnf("Data connection read error: %v", err)

				d.mu.Lock()
				d.dataConn = nil
				d.mu.Unlock()

				continue
			}

			if n > 0 { // This will be equal to d.MessageSize
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

// controlHandler handles control commands, including periodic RC overrides
func (d *Drone) controlHandler(ctx context.Context) {
	defer d.wg.Done()

	// Ticker to periodically send RC override commands
	rcTicker := time.NewTicker(75 * time.Millisecond)
	defer rcTicker.Stop()

	// Timer to reset to neutral if no commands are received
	neutralTimeout := 200 * time.Millisecond
	neutralTimer := time.NewTimer(neutralTimeout)

	for {
		select {
		case <-ctx.Done():
			return
		case <-d.shutdownCh:
			return
		case cmd := <-d.controlCh:
			// When a command is received, handle it.
			// If it's an RC override, update our state and reset the neutral timer.
			if cmd.Type == "rc_override" {
				if d.mavlinkConn != nil && cmd.Data != nil {
					if rcData, ok := cmd.Data.(mavlink.RCOverride); ok {
						d.mavlinkConn.SendRCOverride(rcData)
						d.mu.Lock()
						d.rcOverride = rcData
						d.mu.Unlock()
						// Reset the timer since we got a fresh command
						if !neutralTimer.Stop() {
							// Drain the channel if Stop returns false
							select {
							case <-neutralTimer.C:
							default:
							}
						}
						neutralTimer.Reset(neutralTimeout)
					}
				}
			} else {
				// Handle other commands
				d.executeCommand(cmd)
			}
		case <-rcTicker.C:
			// Periodically send the current RC override values
			if d.mavlinkConn != nil && d.mavlinkConn.IsConnected() {
				d.mu.RLock()
				rcToSend := d.rcOverride
				d.mu.RUnlock()
				d.mavlinkConn.SendRCOverride(rcToSend)
			}
		case <-neutralTimer.C:
			// If the timer fires, it means we haven't received an RC command
			// for a while. Reset to neutral.
			d.mu.Lock()
			d.rcOverride = mavlink.RCOverride{
				Channel1: 1500, // Roll center
				Channel2: 1500, // Pitch center
				Channel3: 1500, // Throttle center
				Channel4: 1500, // Yaw center
			}
			d.mu.Unlock()
		}
	}
}

// executeCommand executes non-rc_override commands
func (d *Drone) executeCommand(cmd ControlCommand) {
	d.logger.Infof("Executing command: %s, with data: %v", cmd.Type, cmd.Data)
	switch cmd.Type {
	case "arm":
		if d.mavlinkConn != nil {
			d.mavlinkConn.Arm()
		}
	case "disarm":
		if d.mavlinkConn != nil {
			d.mavlinkConn.Disarm()
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

// StartLogging enables logging for a specific topic
func (d *Drone) StartLogging(topic string) error {
	if topic == "position" {
		d.isLoggingPosition.Store(true)
		d.logger.Infof("Drone %d: Started position logging", d.ID)
		return nil
	}
	return fmt.Errorf("unknown logging topic: %s", topic)
}

// StopLogging disables logging for a specific topic
func (d *Drone) StopLogging(topic string) error {
	if topic == "position" {
		d.isLoggingPosition.Store(false)
		d.logger.Infof("Drone %d: Stopped position logging", d.ID)
		return nil
	}
	return fmt.Errorf("unknown logging topic: %s", topic)
}

// WriteLog writes a custom marker to the position log file
func (d *Drone) WriteLog(marker string) error {
	if d.logFile == nil {
		return fmt.Errorf("log file for drone %d is not open", d.ID)
	}
	// Sanitize marker to keep the CSV row structure stable.
	sanitizedMarker := strings.ReplaceAll(marker, ",", ";")
	sanitizedMarker = strings.ReplaceAll(sanitizedMarker, "\n", " ")
	sanitizedMarker = strings.ReplaceAll(sanitizedMarker, "\r", " ")

	pos := d.GetPosition()
	logLine := fmt.Sprintf("%d,%d,%d,%d,%d,%s\n",
		time.Now().UnixNano(),
		int64(pos.Lat),
		int64(pos.Lon),
		int64(pos.Alt),
		int64(pos.Heading),
		sanitizedMarker,
	)

	if _, err := d.logFile.WriteString(logLine); err != nil {
		return fmt.Errorf("failed to write marker to log for drone %d: %w", d.ID, err)
	}
	return nil
}

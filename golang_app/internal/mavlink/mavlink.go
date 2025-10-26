package mavlink

import (
	"context"
	"fmt"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
)

// Connection represents a MAVLink connection using gomavlib
type Connection struct {
	node          *gomavlib.Node
	systemID      uint8
	componentID   uint8
	targetSystem  uint8
	lastPosition  *Position
	ctx           context.Context
	cancel        context.CancelFunc
	connected     bool
	lastHeartbeat time.Time
	heartbeatCh   chan bool
}

// Position represents a position from MAVLink
type Position struct {
	Lat       float64 // Latitude in degrees * 1e7
	Lon       float64 // Longitude in degrees * 1e7
	Alt       float64 // Altitude in millimeters
	Heading   float64 // Heading in degrees * 100
	Timestamp time.Time
}

// RCOverride represents RC channel override values
type RCOverride struct {
	Channel1 uint16 // Roll
	Channel2 uint16 // Pitch
	Channel3 uint16 // Throttle
	Channel4 uint16 // Yaw
	Channel5 uint16
	Channel6 uint16
	Channel7 uint16
	Channel8 uint16
}

// NewConnection creates a new MAVLink UDP connection using gomavlib
func NewConnection(udpPort, systemID int) (*Connection, error) {
	ctx, cancel := context.WithCancel(context.Background())

	// Create gomavlib node with UDP server (like Python's udpin:0.0.0.0:port)
	node := &gomavlib.Node{
		Endpoints: []gomavlib.EndpointConf{
			gomavlib.EndpointUDPServer{
				Address: fmt.Sprintf("0.0.0.0:%d", udpPort),
			},
		},
		Dialect:     ardupilotmega.Dialect,
		OutVersion:  gomavlib.V2, // Use MAVLink v2
		OutSystemID: uint8(systemID),
	}

	err := node.Initialize()
	if err != nil {
		cancel()
		return nil, fmt.Errorf("failed to initialize MAVLink node: %w", err)
	}

	conn := &Connection{
		node:         node,
		systemID:     uint8(systemID),
		componentID:  1,
		targetSystem: uint8(systemID),
		ctx:          ctx,
		cancel:       cancel,
		connected:    false,
		heartbeatCh:  make(chan bool, 1),
	}

	// Start processing messages in background
	go conn.processMessages()

	// Wait for heartbeat to confirm real connection
	if err = conn.waitForHeartbeat(5 * time.Second); err != nil {
		conn.Close()
		return nil, fmt.Errorf(
			"MAVLink connection failed - no heartbeat from ArduPilot: %w",
			err,
		)
	}

	conn.connected = true

	return conn, nil
}

// processMessages processes incoming MAVLink messages
func (c *Connection) processMessages() {
	defer func() {
		if r := recover(); r != nil {
			fmt.Printf("MAVLink processMessages recovered from panic: %v\n", r)
		}
	}()

	for {
		select {
		case <-c.ctx.Done():
			return
		case evt, ok := <-c.node.Events():
			if !ok {
				return // Channel closed
			}

			if frm, ok := evt.(*gomavlib.EventFrame); ok {
				c.handleFrame(frm)
			}
		}
	}
}

// handleFrame handles incoming MAVLink frames
func (c *Connection) handleFrame(evt *gomavlib.EventFrame) {
	switch msg := evt.Message().(type) {
	case *ardupilotmega.MessageGlobalPositionInt:
		// Update position from GLOBAL_POSITION_INT message
		c.lastPosition = &Position{
			Lat:       float64(msg.Lat),
			Lon:       float64(msg.Lon),
			Alt:       float64(msg.Alt),
			Heading:   float64(msg.Hdg),
			Timestamp: time.Now(),
		}
	case *ardupilotmega.MessageHeartbeat:
		// Handle heartbeat messages - confirms connection is alive
		c.lastHeartbeat = time.Now()

		// Notify waitForHeartbeat if waiting
		select {
		case c.heartbeatCh <- true:
		default:
			// Channel full, don't block
		}
	}
}

// waitForHeartbeat waits for a heartbeat message with timeout
func (c *Connection) waitForHeartbeat(timeout time.Duration) error {
	select {
	case <-c.heartbeatCh:
		return nil
	case <-time.After(timeout):
		return fmt.Errorf("timeout waiting for heartbeat")
	case <-c.ctx.Done():
		return c.ctx.Err()
	}
}

// IsConnected returns true if MAVLink connection is active and receiving heartbeats
func (c *Connection) IsConnected() bool {
	if !c.connected {
		return false
	}

	// Check if heartbeat is recent (within last 3 seconds)
	return time.Since(c.lastHeartbeat) < 3*time.Second
}

// Close closes the MAVLink connection
func (c *Connection) Close() error {
	c.connected = false
	if c.cancel != nil {
		c.cancel()
	}

	if c.node != nil {
		c.node.Close()
	}

	return nil
}

// ReadPosition reads the latest position data from MAVLink
func (c *Connection) ReadPosition() (*Position, error) {
	if c.lastPosition != nil {
		// Return a copy of the last position
		pos := *c.lastPosition
		return &pos, nil
	}

	return nil, nil // No position data available yet
}

func (c *Connection) Arming(arm bool) error {
	var param1 float32
	if arm {
		param1 = 1
	} else {
		param1 = 0
	}
	return c.node.WriteMessageAll(&ardupilotmega.MessageCommandLong{
		TargetSystem:    c.targetSystem,
		TargetComponent: 1,
		Command:         common.MAV_CMD_COMPONENT_ARM_DISARM,
		Confirmation:    0,
		Param1:          param1,
		Param2:          0,
		Param3:          0,
		Param4:          0,
		Param5:          0,
		Param6:          0,
		Param7:          0,
	})
}

// Arm sends an arm command using proper MAVLink
func (c *Connection) Arm() error {
	return c.Arming(true)
}

// Disarm sends a disarm command using proper MAVLink
func (c *Connection) Disarm() error {
	return c.Arming(false)
}

// SendRCOverride sends RC channel override values using proper MAVLink
func (c *Connection) SendRCOverride(rc RCOverride) error {
	return c.node.WriteMessageAll(&common.MessageRcChannelsOverride{
		TargetSystem:    c.targetSystem,
		TargetComponent: 0,
		Chan1Raw:        rc.Channel1,
		Chan2Raw:        rc.Channel2,
		Chan3Raw:        rc.Channel3,
		Chan4Raw:        rc.Channel4,
		Chan5Raw:        65535, // Invalid/ignored
		Chan6Raw:        65535, // Invalid/ignored
		Chan7Raw:        65535, // Invalid/ignored
		Chan8Raw:        65535, // Invalid/ignored
		Chan9Raw:        0,     // Invalid/ignored
		Chan10Raw:       0,     // Invalid/ignored
		Chan11Raw:       0,     // Invalid/ignored
		Chan12Raw:       0,     // Invalid/ignored
		Chan13Raw:       0,     // Invalid/ignored
		Chan14Raw:       0,     // Invalid/ignored
		Chan15Raw:       0,     // Invalid/ignored
		Chan16Raw:       0,     // Invalid/ignored
		Chan17Raw:       0,     // Invalid/ignored
		Chan18Raw:       0,     // Invalid/ignored
	})
}

// SetMode sets flight mode using proper MAVLink
func (c *Connection) SetMode(mode string) error {
	// Convert mode string to ArduPilot mode number
	var modeNumber uint32

	switch mode {
	case "STABILIZE":
		modeNumber = 0
	case "ACRO":
		modeNumber = 1
	case "ALT_HOLD":
		modeNumber = 2
	case "AUTO":
		modeNumber = 3
	case "GUIDED":
		modeNumber = 4
	case "LOITER":
		modeNumber = 5
	case "RTL":
		modeNumber = 6
	case "CIRCLE":
		modeNumber = 7
	case "LAND":
		modeNumber = 9
	case "DRIFT":
		modeNumber = 11
	case "SPORT":
		modeNumber = 13
	case "FLIP":
		modeNumber = 14
	case "AUTOTUNE":
		modeNumber = 15
	case "POSHOLD":
		modeNumber = 16
	case "BRAKE":
		modeNumber = 17
	case "THROW":
		modeNumber = 18
	case "AVOID_ADSB":
		modeNumber = 19
	case "GUIDED_NOGPS":
		modeNumber = 20
	case "SMART_RTL":
		modeNumber = 21
	default:
		return fmt.Errorf("unknown flight mode: %s", mode)
	}

	return c.node.WriteMessageAll(&ardupilotmega.MessageSetMode{
		TargetSystem: c.targetSystem,
		BaseMode:     common.MAV_MODE(common.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
		CustomMode:   modeNumber,
	})
}

// SendTakeoff sends takeoff command using proper MAVLink
func (c *Connection) SendTakeoff(altitude float64) error {
	return c.node.WriteMessageAll(&ardupilotmega.MessageCommandLong{
		TargetSystem:    c.targetSystem,
		TargetComponent: 1,
		Command:         common.MAV_CMD_NAV_TAKEOFF,
		Confirmation:    0,
		Param1:          0,                 // Minimum pitch (if airspeed sensor present), desired pitch without sensor
		Param2:          0,                 // Empty
		Param3:          0,                 // Empty
		Param4:          0,                 // Yaw angle (if magnetometer present), ignored without magnetometer
		Param5:          0,                 // Latitude
		Param6:          0,                 // Longitude
		Param7:          float32(altitude), // Altitude
	})
}

// SendLand sends land command using proper MAVLink
func (c *Connection) SendLand() error {
	return c.node.WriteMessageAll(&ardupilotmega.MessageCommandLong{
		TargetSystem:    c.targetSystem,
		TargetComponent: 1,
		Command:         common.MAV_CMD_NAV_LAND,
		Confirmation:    0,
		Param1:          0, // Minimum target altitude if landing is aborted
		Param2:          0, // Precision land mode
		Param3:          0, // Empty
		Param4:          0, // Desired yaw angle
		Param5:          0, // Latitude
		Param6:          0, // Longitude
		Param7:          0, // Altitude (ground level)
	})
}

// SendHeartbeat sends a heartbeat message
func (c *Connection) SendHeartbeat() error {
	return c.node.WriteMessageAll(&ardupilotmega.MessageHeartbeat{
		Type:           ardupilotmega.MAV_TYPE_GCS, // Ground Control Station
		Autopilot:      ardupilotmega.MAV_AUTOPILOT_INVALID,
		BaseMode:       0,
		CustomMode:     0,
		SystemStatus:   ardupilotmega.MAV_STATE_ACTIVE,
		MavlinkVersion: 3,
	})
}

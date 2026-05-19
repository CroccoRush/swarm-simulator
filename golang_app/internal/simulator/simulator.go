package simulator

import (
	"context"
	"fmt"
	"swarm-simulator/internal/config"
	"swarm-simulator/internal/drone"
	"swarm-simulator/internal/logger"
	"swarm-simulator/internal/mavlink"
	"swarm-simulator/internal/network"
	"swarm-simulator/internal/scenario"
	"sync"
	"time"

	"github.com/sirupsen/logrus"
)

// Simulator represents the main swarm simulator
type Simulator struct {
	config     *config.Config
	drones     []*drone.Drone
	networkSim *network.Simulator
	logger     *logrus.Logger

	// Experiment control
	experimentMode bool
	controlCh      chan ExperimentCommand

	// Synchronization
	wg sync.WaitGroup
}

// ExperimentCommand represents commands for automated experiments
type ExperimentCommand struct {
	Type string
	Data any
}

// ExperimentData contains experiment parameters
type ExperimentData struct {
	Duration      time.Duration
	FlightPattern string
	Parameters    map[string]any
}

// NewSimulator creates a new swarm simulator
func NewSimulator(cfg *config.Config, logLevel *string) (*Simulator, error) {
	log := logger.NewLogger(logLevel, "SIMULATOR")
	log.Info("Will connect to configured data sources")

	// Create network simulator
	networkSim := network.NewSimulator(&cfg.Network, logLevel)

	sim := &Simulator{
		config:     cfg,
		networkSim: networkSim,
		logger:     log,
		controlCh:  make(chan ExperimentCommand, 10),
	}

	// Create drones
	messageCh := networkSim.GetMessageChannel()

	for _, droneConfig := range cfg.Drones {
		d := drone.NewDrone(droneConfig, messageCh, logLevel, cfg.MessageSize)
		sim.drones = append(sim.drones, d)
		networkSim.AddDrone(d)
	}

	sim.logger.Infof("Created swarm simulator with %d drones", len(sim.drones))

	return sim, nil
}

// Run starts the simulator in the specified mode
func (s *Simulator) Run(ctx context.Context, mode string, scenarioPath string) error {
	s.logger.Infof("Starting simulator in %s mode", mode)

	// Start network simulation
	if err := s.networkSim.Start(ctx); err != nil {
		return fmt.Errorf("failed to start network simulator: %w", err)
	}

	// Start all drones
	anyStarted := false

	for _, d := range s.drones {
		if err := d.Start(ctx); err != nil {
			s.logger.Errorf("Failed to start drone %d: %v", d.ID, err)
			continue
		}

		anyStarted = true
	}

	if !anyStarted {
		return fmt.Errorf("failed to start any drones")
	}

	switch mode {
	case "experiment":
		s.experimentMode = true
		s.wg.Add(1)

		go s.runExperimentMode(ctx, scenarioPath)
	case "gui":
		s.experimentMode = false
		s.wg.Add(1)

		go s.runGUIMode(ctx)
	default:
		return fmt.Errorf("unknown mode: %s", mode)
	}

	// Wait for completion
	s.wg.Wait()

	return nil
}

// Shutdown gracefully shuts down the simulator
func (s *Simulator) Shutdown(ctx context.Context) error {
	s.logger.Info("Shutting down simulator...")

	// Stop all drones
	var wg sync.WaitGroup
	for _, d := range s.drones {
		wg.Add(1)

		go func(drone *drone.Drone) {
			defer wg.Done()

			if err := drone.Stop(ctx); err != nil {
				s.logger.Errorf("Error stopping drone %d: %v", drone.ID, err)
			}
		}(d)
	}

	// Wait for drones to stop
	done := make(chan struct{})
	go func() {
		wg.Wait()
		close(done)
	}()

	select {
	case <-done:
		s.logger.Infof("All drones stopped")
	case <-ctx.Done():
		s.logger.Warnf("Timeout waiting for drones to stop")
	}

	// Stop network simulation
	if err := s.networkSim.Stop(ctx); err != nil {
		s.logger.Errorf("Error stopping network simulator: %v", err)
	}

	return nil
}

// runExperimentMode runs the simulator in experiment mode
func (s *Simulator) runExperimentMode(ctx context.Context, scenarioPath string) {
	defer s.wg.Done()

	// Wait for initialization
	s.logger.Info("Waiting for MAVLink connections and position fixes...")
	if err := s.waitForDronesReady(ctx); err != nil {
		s.logger.Errorf("Drones not ready for experiment: %v", err)
		return
	}
	s.logger.Info("All drones are connected and have a position fix.")

	// Load the scenario
	s.logger.Infof("Loading scenario from: %s", scenarioPath)
	scen, err := scenario.LoadScenario(scenarioPath)
	if err != nil {
		s.logger.Errorf("Failed to load scenario: %v", err)
		return
	}

	s.logger.Infof("Running scenario: '%s'", scen.Name)

	// Run experiment sequence from scenario
	if err := s.executeScenario(ctx, scen); err != nil {
		s.logger.Errorf("Experiment scenario failed: %v", err)
		return
	}

	s.logger.Infof("Experiment scenario '%s' completed successfully", scen.Name)
}

// runGUIMode runs the simulator in GUI mode (placeholder)
func (s *Simulator) runGUIMode(ctx context.Context) {
	defer s.wg.Done()

	s.logger.Info(
		"Running in GUI mode (placeholder)." +
			" GUI implementation would go here." +
			" For now, running in monitoring mode...",
	)

	// In GUI mode, just monitor the system
	ticker := time.NewTicker(10 * time.Second)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			s.printSwarmStatus()
		}
	}
}

// getExperimentParameters gets experiment parameters from environment
func (s *Simulator) getExperimentParameters() ExperimentData {
	// In a real implementation, you'd read from environment variables
	// For now, return default parameters
	return ExperimentData{
		Duration:      60 * time.Second,
		FlightPattern: "formation_flight",
		Parameters: map[string]any{
			"altitude":  10.0,
			"speed":     5.0,
			"formation": "grid",
		},
	}
}

// executeScenario runs the main experiment sequence based on a loaded scenario
func (s *Simulator) executeScenario(ctx context.Context, scen *scenario.Scenario) error {
	s.logger.Infof("Executing %d actions from scenario '%s'", len(scen.Actions), scen.Name)

	for i, action := range scen.Actions {
		s.logger.Infof("--- Action %d/%d: Executing command '%v' ---", i, len(scen.Actions), action["command"])

		// Extract command
		command, ok := action["command"].(string)
		if !ok {
			return fmt.Errorf("action %d has a missing or invalid 'command' field", i)
		}

		// Handle target drones
		targetDrones, err := s.getTargetDrones(action)
		if err != nil {
			return fmt.Errorf("failed to get target drones for action %d: %w", i, err)
		}

		// Execute command
		switch command {
		case "set_mode":
			mode, ok := action["mode"].(string)
			if !ok {
				return fmt.Errorf("action %d ('set_mode'): missing or invalid 'mode' parameter", i)
			}
			for _, d := range targetDrones {
				d.SendControlCommand(drone.ControlCommand{Type: "set_mode", Data: mode})
			}

		case "arm":
			for _, d := range targetDrones {
				d.SendControlCommand(drone.ControlCommand{Type: "arm"})
			}

		case "disarm":
			for _, d := range targetDrones {
				d.SendControlCommand(drone.ControlCommand{Type: "disarm"})
			}

		case "takeoff":
			alt, ok := action["altitude"].(float64)
			if !ok {
				return fmt.Errorf("action %d ('takeoff'): missing or invalid 'altitude' parameter", i)
			}
			for _, d := range targetDrones {
				d.SendControlCommand(drone.ControlCommand{Type: "takeoff", Data: alt})
			}

		case "land":
			for _, d := range targetDrones {
				d.SendControlCommand(drone.ControlCommand{Type: "land"})
			}

		case "rc_override":
			rc, err := s.parseRCOverride(action)
			if err != nil {
				return fmt.Errorf("action %d ('rc_override'): %w", i, err)
			}
			// Check for duration. If present, execute as a maneuver.
			if durationVal, ok := action["duration"]; ok {
				duration, ok := durationVal.(float64)
				if !ok {
					return fmt.Errorf("action %d ('rc_override'): invalid 'duration' type", i)
				}
				s.executeRCOverrideForDuration(targetDrones, rc, time.Duration(duration*float64(time.Second)))
			} else {
				// If no duration, send the command just once.
				for _, d := range targetDrones {
					d.SendControlCommand(drone.ControlCommand{Type: "rc_override", Data: rc})
				}
			}

		case "wait":
			duration, ok := action["duration"].(float64) // YAML/JSON parsers often use float64 for numbers
			if !ok {
				return fmt.Errorf("action %d ('wait'): missing or invalid 'duration' parameter", i)
			}
			s.logger.Infof("Waiting for %.2f seconds...", duration)
			time.Sleep(time.Duration(duration * float64(time.Second)))

		case "start_log":
			topic, ok := action["topic"].(string)
			if !ok {
				return fmt.Errorf("action %d ('start_log'): missing or invalid 'topic' parameter", i)
			}
			for _, d := range targetDrones {
				if err := d.StartLogging(topic); err != nil {
					s.logger.Warnf("Failed to start logging for drone %d: %v", d.ID, err)
				}
			}

		case "stop_log":
			topic, ok := action["topic"].(string)
			if !ok {
				return fmt.Errorf("action %d ('stop_log'): missing or invalid 'topic' parameter", i)
			}
			for _, d := range targetDrones {
				if err := d.StopLogging(topic); err != nil {
					s.logger.Warnf("Failed to stop logging for drone %d: %v", d.ID, err)
				}
			}

		case "write_log":
			marker, ok := action["marker"].(string)
			if !ok {
				return fmt.Errorf("action %d ('write_log'): missing or invalid 'marker' parameter", i)
			}
			for _, d := range targetDrones {
				if err := d.WriteLog(marker); err != nil {
					s.logger.Warnf("Failed to write log marker for drone %d: %v", d.ID, err)
				}
			}

		default:
			return fmt.Errorf("action %d: unknown command '%s'", i, command)
		}

	}
	return nil
}

// executeRCOverrideForDuration sends RC override commands to specified drones for a duration.
func (s *Simulator) executeRCOverrideForDuration(targets []*drone.Drone, rc mavlink.RCOverride, duration time.Duration) {
	s.logger.Infof(
		"Executing RC command for %.1fs: Roll=%d, Pitch=%d, Throttle=%d, Yaw=%d",
		duration.Seconds(), rc.Channel1, rc.Channel2, rc.Channel3, rc.Channel4,
	)

	// The loop frequency should be high enough to avoid failsafe
	ticker := time.NewTicker(75 * time.Millisecond)
	defer ticker.Stop()

	ctx, cancel := context.WithTimeout(context.Background(), duration)
	defer cancel()

loop:
	for {
		select {
		case <-ctx.Done():
			break loop
		case <-ticker.C:
			for _, d := range targets {
				d.SendControlCommand(drone.ControlCommand{
					Type: "rc_override",
					Data: rc,
				})
			}
		}
	}

	// Return to center after the maneuver is complete
	s.logger.Info("Maneuver complete. Sending neutral RC command.")
	centerRC := mavlink.RCOverride{
		Channel1: 1500, Channel2: 1500, Channel3: 1500, Channel4: 1500,
	}
	for _, d := range targets {
		d.SendControlCommand(drone.ControlCommand{
			Type: "rc_override",
			Data: centerRC,
		})
	}
}

// getTargetDrones parses the 'drones' field from an action to determine which drones to command.
func (s *Simulator) getTargetDrones(action scenario.Action) ([]*drone.Drone, error) {
	dronesField, ok := action["drones"]
	if !ok {
		// Default to all drones if not specified
		return s.drones, nil
	}

	// Handle "all" keyword
	if droneStr, ok := dronesField.(string); ok && droneStr == "all" {
		return s.drones, nil
	}

	// Handle list of IDs
	if droneIDs, ok := dronesField.([]interface{}); ok {
		var targets []*drone.Drone
		for _, idInterface := range droneIDs {
			idFloat, ok := idInterface.(float64) // Numbers from JSON/YAML are often float64
			if !ok {
				return nil, fmt.Errorf("invalid drone ID type in list: %T", idInterface)
			}
			id := int(idFloat)
			found := false
			for _, d := range s.drones {
				if d.ID == id {
					targets = append(targets, d)
					found = true
					break
				}
			}
			if !found {
				return nil, fmt.Errorf("drone with ID %d not found", id)
			}
		}
		return targets, nil
	}

	return nil, fmt.Errorf("invalid 'drones' field format: must be 'all' or a list of IDs")
}

// parseUint16 converts an interface{} to uint16, accepting int, uint16 or float64.
func parseUint16(v interface{}) (uint16, bool) {
	if i, ok := v.(int); ok {
		return uint16(i), true
	}
	if f, ok := v.(uint16); ok {
		return f, true
	}
	if f, ok := v.(float64); ok {
		return uint16(f), true
	}
	return 0, false
}

// parseRCOverride parses RC override values from a scenario action.
func (s *Simulator) parseRCOverride(action scenario.Action) (mavlink.RCOverride, error) {
	rc := mavlink.RCOverride{
		Channel1: 1500, // Roll center
		Channel2: 1500, // Pitch center
		Channel3: 1500, // Throttle center
		Channel4: 1500, // Yaw center
	}

	if val, ok := action["roll"]; ok {
		if f, ok := parseUint16(val); ok {
			rc.Channel1 = uint16(f)
		} else {
			return rc, fmt.Errorf("invalid 'roll' value type: %T", val)
		}
	}
	if val, ok := action["pitch"]; ok {
		if f, ok := parseUint16(val); ok {
			rc.Channel2 = uint16(f)
		} else {
			return rc, fmt.Errorf("invalid 'pitch' value type: %T", val)
		}
	}
	if val, ok := action["throttle"]; ok {
		if f, ok := parseUint16(val); ok {
			rc.Channel3 = uint16(f)
		} else {
			return rc, fmt.Errorf("invalid 'throttle' value type: %T", val)
		}
	}
	if val, ok := action["yaw"]; ok {
		if f, ok := parseUint16(val); ok {
			rc.Channel4 = uint16(f)
		} else {
			return rc, fmt.Errorf("invalid 'yaw' value type: %T", val)
		}
	}

	return rc, nil
}

// waitForDronesReady waits for all drones to be connected and have a position fix.
func (s *Simulator) waitForDronesReady(ctx context.Context) error {
	ctx, cancel := context.WithTimeout(ctx, 60*time.Second) // Generous timeout
	defer cancel()

	var wg sync.WaitGroup
	for _, d := range s.drones {
		wg.Add(1)
		go func(dr *drone.Drone) {
			defer wg.Done()
			ticker := time.NewTicker(200 * time.Millisecond)
			defer ticker.Stop()
			for {
				select {
				case <-ctx.Done():
					s.logger.Errorf("Drone %d failed to get ready: %v", dr.ID, ctx.Err())
					return
				case <-ticker.C:
					if dr.IsConnected() && dr.HasPositionFix() {
						return
					}
				}
			}
		}(d)
	}
	wg.Wait()

	if ctx.Err() != nil {
		return fmt.Errorf("one or more drones did not become ready in time")
	}

	return nil
}

// runExperimentSequence runs the main experiment sequence
func (s *Simulator) runExperimentSequence(ctx context.Context, params ExperimentData) error {
	s.logger.Infof("Starting experiment sequence: %s", params.FlightPattern)

	// Phase 1: Prepare drones
	s.logger.Infof("Phase 1: Preparing drones...")

	if err := s.prepareSwarm(ctx, params); err != nil {
		return fmt.Errorf("failed to prepare swarm: %w", err)
	}

	// Phase 2: Execute flight pattern
	s.logger.Infof("Phase 2: Executing flight pattern...")

	if err := s.executeFlightPattern(ctx, params); err != nil {
		return fmt.Errorf("failed to execute flight pattern: %w", err)
	}

	// Phase 3: Monitor and collect data
	s.logger.Infof("Phase 3: Monitoring swarm...")

	if err := s.monitorSwarm(ctx, params.Duration); err != nil {
		return fmt.Errorf("failed to monitor swarm: %w", err)
	}

	// Phase 4: Land and collect results
	s.logger.Infof("Phase 4: Landing and collecting results...")

	if err := s.landSwarm(ctx); err != nil {
		return fmt.Errorf("failed to land swarm: %w", err)
	}

	return nil
}

// prepareSwarm prepares all drones for the experiment
func (s *Simulator) prepareSwarm(ctx context.Context, params ExperimentData) error {
	s.logger.Info("Preparing swarm...")

	for _, d := range s.drones {
		d.SendControlCommand(drone.ControlCommand{
			Type: "set_mode",
			Data: "GUIDED",
		})
	}

	// Wait for all drones to have a position fix before arming
	s.logger.Info("Waiting for all drones to acquire position fix from SITL...")
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()

	wg := sync.WaitGroup{}
	for i := range s.drones {
		wg.Add(1)
		go func(d *drone.Drone) {
			defer wg.Done()
			for {
				select {
				case <-ctx.Done():
					s.logger.Errorf("Drone %d failed to acquire position fix within timeout", d.ID)
					return
				default:
					if d.HasPositionFix() {
						s.logger.Infof("Drone %d has position fix", d.ID)
						return
					}
					time.Sleep(100 * time.Millisecond)
				}
			}
		}(s.drones[i])
	}
	wg.Wait()

	if ctx.Err() != nil {
		return fmt.Errorf("failed to prepare swarm: one or more drones did not acquire a position fix")
	}

	// Arming
	time.Sleep(60 * time.Second)

	s.logger.Info("All drones have position fix. Arming...")
	for _, d := range s.drones {
		d.SendControlCommand(drone.ControlCommand{Type: "arm"})
	}

	time.Sleep(1 * time.Second)

	// Take off
	altitude := 10.0
	if alt, ok := params.Parameters["altitude"].(float64); ok {
		altitude = alt
	}

	s.logger.Infof("Taking off to %.1fm...", altitude)

	for _, d := range s.drones {
		d.SendControlCommand(drone.ControlCommand{
			Type: "takeoff",
			Data: altitude,
		})
	}

	time.Sleep(10 * time.Second)

	return nil
}

// executeFlightPattern executes the specified flight pattern
func (s *Simulator) executeFlightPattern(ctx context.Context, params ExperimentData) error {
	switch params.FlightPattern {
	case "formation_flight":
		return s.executeFormationFlight(ctx)
	case "dispersion":
		return s.executeDispersion(ctx)
	default:
		return fmt.Errorf("unknown flight pattern: %s", params.FlightPattern)
	}
}

// executeFormationFlight executes formation flight pattern
func (s *Simulator) executeFormationFlight(ctx context.Context) error {
	s.logger.Info("Executing formation flight pattern")

	for _, d := range s.drones {
		d.SendControlCommand(drone.ControlCommand{
			Type: "set_mode",
			Data: "POSHOLD",
		})
	}

	// Execute formation maneuvers
	s.logger.Info("Executing formation maneuvers...")

	// Forward flight
	s.executeRCCommand(mavlink.RCOverride{
		Channel1: 1500, // Roll center
		Channel2: 1200, // Pitch forward
		Channel3: 1500, // Throttle center
		Channel4: 1500, // Yaw center
	}, 20*time.Second)

	// Turn right
	s.executeRCCommand(mavlink.RCOverride{
		Channel1: 1500, // Roll center
		Channel2: 1400, // Pitch slight forward
		Channel3: 1500, // Throttle center
		Channel4: 1550, // Yaw right
	}, 15*time.Second)

	// Forward flight again
	s.executeRCCommand(mavlink.RCOverride{
		Channel1: 1500, // Roll center
		Channel2: 1200, // Pitch forward
		Channel3: 1500, // Throttle center
		Channel4: 1500, // Yaw center
	}, 20*time.Second)

	return nil
}

// executeDispersion executes dispersion pattern
func (s *Simulator) executeDispersion(ctx context.Context) error {
	s.logger.Info("Executing dispersion pattern")
	// TODO: Implement dispersion pattern
	return nil
}

// executeRCCommand sends RC override commands to all drones for specified duration
func (s *Simulator) executeRCCommand(rc mavlink.RCOverride, duration time.Duration) {
	s.logger.Infof(
		"Executing RC command for %.1fs: Roll=%d, Pitch=%d, Throttle=%d, Yaw=%d",
		duration.Seconds(), rc.Channel1, rc.Channel2, rc.Channel3, rc.Channel4,
	)

	startTime := time.Now()
	for time.Since(startTime) < duration {
		for _, d := range s.drones {
			d.SendControlCommand(drone.ControlCommand{
				Type: "rc_override",
				Data: rc,
			})
		}

		time.Sleep(75 * time.Millisecond)
	}

	// Return to center
	centerRC := mavlink.RCOverride{
		Channel1: 1500, Channel2: 1500, Channel3: 1500, Channel4: 1500,
	}

	for _, d := range s.drones {
		d.SendControlCommand(drone.ControlCommand{
			Type: "rc_override",
			Data: centerRC,
		})
	}
}

// monitorSwarm monitors the swarm during experiment
func (s *Simulator) monitorSwarm(ctx context.Context, duration time.Duration) error {
	s.logger.Infof("Monitoring swarm for %.1fs...", duration.Seconds())

	ticker := time.NewTicker(5 * time.Second)
	defer ticker.Stop()

	timeout := time.After(duration)

	for {
		select {
		case <-ctx.Done():
			return nil
		case <-timeout:
			return nil
		case <-ticker.C:
			s.printSwarmStatus()
		}
	}
}

// landSwarm lands all drones
func (s *Simulator) landSwarm(ctx context.Context) error {
	s.logger.Infof("Landing all drones...")

	// Set LAND mode or use throttle down
	for _, d := range s.drones {
		d.SendControlCommand(drone.ControlCommand{
			Type: "set_mode",
			Data: "LAND",
		})
	}

	time.Sleep(10 * time.Second)

	// Disarm
	s.logger.Infof("Disarming all drones...")

	for _, d := range s.drones {
		d.SendControlCommand(drone.ControlCommand{Type: "disarm"})
	}

	return nil
}

// printSwarmStatus prints current status of all drones
func (s *Simulator) printSwarmStatus() {
	connected := 0

	for _, d := range s.drones {
		if d.IsConnected() {
			connected++
		}
	}

	stats := s.networkSim.GetStats()

	s.logger.Infof(
		"Swarm Status: %d/%d drones connected, %d network messages processed",
		connected, len(s.drones), stats.TotalMessages,
	)

	for _, d := range s.drones {
		pos := d.GetPosition()

		status := "disconnected"
		if d.IsConnected() {
			status = "connected"
		}

		s.logger.Infof(
			"Drone %d %s: (%.6f,%.6f,%.1fm)",
			d.ID, status, pos.Lat/1e7, pos.Lon/1e7, pos.Alt/1000,
		)
	}
}

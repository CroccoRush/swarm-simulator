package simulator

import (
	"context"
	"fmt"
	"swarm-simulator/internal/config"
	"swarm-simulator/internal/drone"
	"swarm-simulator/internal/logger"
	"swarm-simulator/internal/mavlink"
	"swarm-simulator/internal/network"
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
func (s *Simulator) Run(ctx context.Context, mode string) error {
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

		go s.runExperimentMode(ctx)
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
func (s *Simulator) runExperimentMode(ctx context.Context) {
	defer s.wg.Done()

	s.logger.Info("Running in experiment mode")

	// Get experiment parameters from environment or use defaults
	expParams := s.getExperimentParameters()

	// Wait for initialization
	time.Sleep(5 * time.Second)

	// Run experiment sequence
	if err := s.runExperimentSequence(ctx, expParams); err != nil {
		s.logger.Errorf("Experiment failed: %v", err)
		return
	}

	s.logger.Infof("Experiment completed successfully")
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

// runExperimentSequence runs the main experiment sequence
func (s *Simulator) runExperimentSequence(ctx context.Context, params ExperimentData) error {
	s.logger.Infof("Starting experiment sequence: %s", params.FlightPattern)

	// Phase 1: Prepare drones
	s.logger.Infof("Phase 1: Preparing drones...")

	if err := s.prepareSwarm(ctx); err != nil {
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
func (s *Simulator) prepareSwarm(ctx context.Context) error {
	s.logger.Infof("Setting GUIDED mode for all drones...")

	for _, d := range s.drones {
		d.SendControlCommand(drone.ControlCommand{
			Type: "set_mode",
			Data: "GUIDED",
		})
	}

	time.Sleep(2 * time.Second)

	s.logger.Info("Arming all drones...")

	for _, d := range s.drones {
		d.SendControlCommand(drone.ControlCommand{Type: "arm"})
	}

	time.Sleep(3 * time.Second)

	return nil
}

// executeFlightPattern executes the specified flight pattern
func (s *Simulator) executeFlightPattern(ctx context.Context, params ExperimentData) error {
	switch params.FlightPattern {
	case "formation_flight":
		return s.executeFormationFlight(ctx, params)
	case "dispersion":
		return s.executeDispersion(ctx, params)
	default:
		return fmt.Errorf("unknown flight pattern: %s", params.FlightPattern)
	}
}

// executeFormationFlight executes formation flight pattern
func (s *Simulator) executeFormationFlight(ctx context.Context, params ExperimentData) error {
	s.logger.Info("Executing formation flight pattern")

	altitude := 10.0
	if alt, ok := params.Parameters["altitude"].(float64); ok {
		altitude = alt
	}

	// Take off
	s.logger.Infof("Taking off to %.1fm...", altitude)

	for _, d := range s.drones {
		d.SendControlCommand(drone.ControlCommand{
			Type: "takeoff",
			Data: altitude,
		})
	}

	time.Sleep(10 * time.Second)

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
		Channel4: 1650, // Yaw right
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
func (s *Simulator) executeDispersion(ctx context.Context, params ExperimentData) error {
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

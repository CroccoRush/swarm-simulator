package main

import (
	"context"
	"flag"
	"fmt"
	"log"
	"os"
	"os/signal"
	"swarm-simulator/internal/config"
	"swarm-simulator/internal/simulator"
	"syscall"
	"time"
)

func main() {
	var (
		configFile = flag.String("config", "config.json", "Configuration file path")
		mode       = flag.String("mode", "experiment", "Simulator mode: experiment or gui")
		logLevel   = flag.String("log-level", "warn", "Log level: panic, fatal, error, warn, info, debug, trace")
	)

	flag.Parse()

	// Load configuration
	cfg, err := config.LoadConfig(*configFile)
	if err != nil {
		log.Fatalf("Failed to load config: %v", err)
	}

	fmt.Printf("Swarm Simulator v2.0 (Go) - %d drones\n", len(cfg.Drones))
	fmt.Printf(
		"Network: range=%.0fm, loss=%.1f%%, mode=%s\n",
		cfg.Network.MaxRange, cfg.Network.BasePacketLoss*100, *mode,
	)
	fmt.Printf("Log level: %s\n", *logLevel)
	fmt.Println("Connecting to configured data sources")
	fmt.Println("Press Ctrl+C to stop...")

	// Create context for graceful shutdown
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	// Handle shutdown signals
	sigCh := make(chan os.Signal, 1)
	signal.Notify(sigCh, syscall.SIGINT, syscall.SIGTERM)

	// Create and start simulator
	sim, err := simulator.NewSimulator(cfg, logLevel)
	if err != nil {
		log.Fatalf("Failed to create simulator: %v", err)
	}

	// Start simulator in background
	go func() {
		if err := sim.Run(ctx, *mode); err != nil {
			log.Printf("Simulator error: %v", err)
			cancel()
		}
	}()

	// Wait for shutdown signal
	select {
	case sig := <-sigCh:
		fmt.Printf("\nReceived signal %s, shutting down...\n", sig)
	case <-ctx.Done():
		fmt.Println("\nContext cancelled, shutting down...")
	}

	// Graceful shutdown
	shutdownCtx, shutdownCancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer shutdownCancel()

	if err = sim.Shutdown(shutdownCtx); err != nil {
		log.Printf("Error during shutdown: %v", err)
	}

	fmt.Println("Simulator stopped gracefully")
}

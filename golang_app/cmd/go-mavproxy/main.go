package main

import (
	"context"
	"flag"
	"fmt"
	"log"
	"os"
	"os/signal"
	"strings"
	"sync"
	"syscall"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
)

// GoMAVProxy represents a simple MAVLink proxy server
type GoMAVProxy struct {
	masterEndpoint string
	sitlEndpoint   string
	outputs        []string

	masterNode  *gomavlib.Node
	outputNodes []*gomavlib.Node

	logger *log.Logger
	ctx    context.Context
	cancel context.CancelFunc
	wg     sync.WaitGroup

	// Statistics
	messagesReceived uint64
	messagesSent     uint64
	startTime        time.Time
}

// NewGoMAVProxy creates a new Go MAVProxy instance
func NewGoMAVProxy(master, sitl string, outputs []string) *GoMAVProxy {
	ctx, cancel := context.WithCancel(context.Background())

	return &GoMAVProxy{
		masterEndpoint: master,
		sitlEndpoint:   sitl,
		outputs:        outputs,
		logger:         log.New(os.Stdout, "[GO-MAVPROXY] ", log.LstdFlags|log.Lmicroseconds),
		ctx:            ctx,
		cancel:         cancel,
		startTime:      time.Now(),
	}
}

// Start starts the MAVProxy
func (mp *GoMAVProxy) Start() error {
	mp.logger.Printf("Starting Go MAVProxy")
	mp.logger.Printf("Master: %s", mp.masterEndpoint)
	mp.logger.Printf("SITL: %s", mp.sitlEndpoint)
	mp.logger.Printf("Outputs: %v", mp.outputs)

	// Create master connection (to ArduPilot SITL)
	masterNode, err := mp.createMasterConnection()
	if err != nil {
		return fmt.Errorf("failed to create master connection: %w", err)
	}

	mp.masterNode = masterNode

	// Create output connections
	for _, output := range mp.outputs {
		outputNode, err := mp.createOutputConnection(output)
		if err != nil {
			mp.logger.Printf("Failed to create output %s: %v", output, err)
			continue
		}

		mp.outputNodes = append(mp.outputNodes, outputNode)
		mp.logger.Printf("Output connected: %s", output)
	}

	if len(mp.outputNodes) == 0 {
		return fmt.Errorf("no output connections created")
	}

	// Start message forwarding
	mp.wg.Add(1)
	go mp.masterMessageHandler()

	// Start each output handler
	for i, node := range mp.outputNodes {
		mp.wg.Add(1)
		go mp.outputMessageHandler(i, node)
	}

	// Start statistics reporter
	mp.wg.Add(1)
	go mp.statisticsReporter()

	mp.logger.Printf("Go MAVProxy started successfully")

	return nil
}

// Stop stops the MAVProxy
func (mp *GoMAVProxy) Stop() {
	mp.logger.Printf("Stopping Go MAVProxy...")

	mp.cancel()

	// Close all connections
	if mp.masterNode != nil {
		mp.masterNode.Close()
	}

	for _, node := range mp.outputNodes {
		if node != nil {
			node.Close()
		}
	}

	// Wait for goroutines
	done := make(chan struct{})
	go func() {
		mp.wg.Wait()
		close(done)
	}()

	select {
	case <-done:
		mp.logger.Printf("Go MAVProxy stopped gracefully")
	case <-time.After(5 * time.Second):
		mp.logger.Printf("Go MAVProxy stop timeout")
	}
}

// createMasterConnection creates connection to ArduPilot SITL
func (mp *GoMAVProxy) createMasterConnection() (*gomavlib.Node, error) {
	mp.logger.Printf("Connecting to master: %s", mp.masterEndpoint)

	node := &gomavlib.Node{
		Endpoints: []gomavlib.EndpointConf{
			gomavlib.EndpointTCPClient{
				Address: mp.masterEndpoint,
			},
		},
		Dialect:     ardupilotmega.Dialect,
		OutVersion:  gomavlib.V2,
		OutSystemID: 255, // Ground Control Station
	}

	if err := node.Initialize(); err != nil {
		return nil, err
	}

	mp.logger.Printf("Master connected: %s", mp.masterEndpoint)

	return node, nil
}

// createOutputConnection creates output connection
func (mp *GoMAVProxy) createOutputConnection(output string) (*gomavlib.Node, error) {
	var endpoint gomavlib.EndpointConf

	if strings.HasPrefix(output, "udp:") {
		// UDP output: udp:127.0.0.1:14500 - создаем UDP сервер для GCS подключений
		address := strings.TrimPrefix(output, "udp:")
		endpoint = gomavlib.EndpointUDPClient{
			Address: address,
		}
	} else if strings.HasPrefix(output, "tcp:") {
		// TCP output: tcp:127.0.0.1:5762 - создаем TCP сервер для GCS подключений
		address := strings.TrimPrefix(output, "tcp:")
		endpoint = gomavlib.EndpointTCPClient{
			Address: address,
		}
	} else {
		return nil, fmt.Errorf("unsupported output format: %s", output)
	}

	node := &gomavlib.Node{
		Endpoints:   []gomavlib.EndpointConf{endpoint},
		Dialect:     ardupilotmega.Dialect,
		OutVersion:  gomavlib.V2,
		OutSystemID: 255,
	}

	if err := node.Initialize(); err != nil {
		return nil, err
	}

	return node, nil
}

// masterMessageHandler handles messages from master (ArduPilot)
func (mp *GoMAVProxy) masterMessageHandler() {
	defer mp.wg.Done()

	mp.logger.Printf("Starting master message handler")

	for {
		select {
		case <-mp.ctx.Done():
			return
		case evt, ok := <-mp.masterNode.Events():
			if !ok {
				return
			}

			if frm, ok := evt.(*gomavlib.EventFrame); ok {
				mp.messagesReceived++

				// Forward to all outputs
				for _, outputNode := range mp.outputNodes {
					if err := outputNode.WriteMessageAll(frm.Message()); err != nil {
						// Log error but continue
						if mp.logger != nil {
							mp.logger.Printf("Failed to forward message to output: %v", err)
						}
					} else {
						mp.messagesSent++
					}
				}
			}
		}
	}
}

// outputMessageHandler handles messages from outputs (back to master)
func (mp *GoMAVProxy) outputMessageHandler(index int, node *gomavlib.Node) {
	defer mp.wg.Done()

	mp.logger.Printf("Starting output message handler %d", index)

	for {
		select {
		case <-mp.ctx.Done():
			return
		case evt, ok := <-node.Events():
			if !ok {
				return
			}

			if frm, ok := evt.(*gomavlib.EventFrame); ok {
				// Forward back to master
				if err := mp.masterNode.WriteMessageAll(frm.Message()); err != nil {
					if mp.logger != nil {
						mp.logger.Printf("Failed to forward message to master: %v", err)
					}
				}
			}
		}
	}
}

// statisticsReporter reports statistics periodically
func (mp *GoMAVProxy) statisticsReporter() {
	defer mp.wg.Done()

	ticker := time.NewTicker(30 * time.Second)
	defer ticker.Stop()

	for {
		select {
		case <-mp.ctx.Done():
			return
		case <-ticker.C:
			uptime := time.Since(mp.startTime)
			mp.logger.Printf(
				"Stats: %d received, %d sent, uptime: %v, outputs: %d",
				mp.messagesReceived, mp.messagesSent, uptime.Round(time.Second), len(mp.outputNodes),
			)
		}
	}
}

func main() {
	var (
		master  = flag.String("master", "127.0.0.1:5760", "Master connection (ArduPilot SITL)")
		sitl    = flag.String("sitl", "127.0.0.1:5501", "SITL connection (optional)")
		outputs = flag.String("out", "udp:127.0.0.1:14500", "Output connections (comma-separated)")
		verbose = flag.Bool("verbose", false, "Enable verbose logging")
	)

	flag.Parse()

	if !*verbose {
		log.SetOutput(os.Stdout)
	}

	// Parse outputs
	outputList := strings.Split(*outputs, ",")
	for i, output := range outputList {
		outputList[i] = strings.TrimSpace(output)
	}

	// Create and start proxy
	proxy := NewGoMAVProxy(*master, *sitl, outputList)

	if err := proxy.Start(); err != nil {
		log.Fatalf("Failed to start Go MAVProxy: %v", err)
	}

	// Handle shutdown
	sigCh := make(chan os.Signal, 1)
	signal.Notify(sigCh, syscall.SIGINT, syscall.SIGTERM)

	fmt.Printf("Go MAVProxy running - Press Ctrl+C to stop\n")
	fmt.Printf("Master: %s\n", *master)
	fmt.Printf("Outputs: %s\n", *outputs)

	<-sigCh
	proxy.Stop()
}

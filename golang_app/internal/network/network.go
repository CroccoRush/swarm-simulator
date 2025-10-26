package network

import (
	"context"
	"math"
	"math/rand"
	"swarm-simulator/internal/config"
	"swarm-simulator/internal/drone"
	"swarm-simulator/internal/logger"
	"sync"
	"sync/atomic"
	"time"

	"github.com/sirupsen/logrus"
	"github.com/tidwall/rtree"
)

// Simulator represents the network simulator
type Simulator struct {
	config    *config.NetworkConfig
	drones    map[int]*drone.Drone
	messageCh chan drone.Message
	logger    *logrus.Logger
	index     *SpatialIndex

	// Statistics
	stats   NetworkStats
	statsmu sync.RWMutex

	// Control channels
	shutdownCh chan struct{}
	wg         sync.WaitGroup
}

// NetworkStats holds network simulation statistics
type NetworkStats struct {
	TotalMessages        int64
	DeliveredMessages    int64
	LostMessages         int64
	OutOfRangeMessages   int64
	DisconnectedMessages int64
}

type SpatialIndex struct {
	tree  rtree.RTree
	mutex sync.RWMutex
}

// DroneItem represents a drone in the spatial index
type DroneItem struct {
	ID       int
	Position drone.Position
}

// Rect returns the bounding box for the drone's position
func (item DroneItem) Rect() (min, max [2]float64) {
	lat := float64(item.Position.Lat)
	lon := float64(item.Position.Lon)
	return [2]float64{lat, lon}, [2]float64{lat, lon}
}

// NewSimulator creates a new network simulator
func NewSimulator(cfg *config.NetworkConfig, logLevel *string) *Simulator {
	log := logger.NewLogger(logLevel, "NETWORK")

	return &Simulator{
		config:     cfg,
		drones:     make(map[int]*drone.Drone),
		messageCh:  make(chan drone.Message, cfg.ChannelBufferSize),
		logger:     log,
		shutdownCh: make(chan struct{}),
		index:      &SpatialIndex{},
	}
}

// AddDrone adds a drone to the network simulation
func (s *Simulator) AddDrone(d *drone.Drone) {
	s.drones[d.ID] = d
	s.logger.Infof("Added drone %d to network simulation", d.ID)
}

// Start starts the network simulation
func (s *Simulator) Start(ctx context.Context) error {
	s.logger.Infof("Starting network simulator with %d drones", len(s.drones))
	s.logger.Infof(
		"Network params: range=%.0fm, loss=%.1f%%, update_rate=%dHz",
		s.config.MaxRange, s.config.BasePacketLoss*100, s.config.UpdateRateHz,
	)

	// Start message processing workers
	numWorkers := max(1, min(s.config.MaxConcurrentMsgs, len(s.drones)*2))

	for i := 0; i < numWorkers; i++ {
		s.wg.Add(1)
		go s.messageWorker(ctx, i)
	}

	// Start statistics reporter
	s.wg.Add(1)
	go s.statsReporter(ctx)

	s.wg.Add(1)
	go s.indexUpdater(ctx)

	s.logger.Infof("Network simulator started with %d workers", numWorkers)

	return nil
}

// Stop stops the network simulation
func (s *Simulator) Stop(ctx context.Context) error {
	s.logger.Infof("Stopping network simulator...")

	close(s.shutdownCh)

	// Wait for workers to finish or timeout
	done := make(chan struct{})
	go func() {
		s.wg.Wait()
		close(done)
	}()

	select {
	case <-done:
		s.logger.Infof("Network simulator stopped gracefully")
	case <-ctx.Done():
		s.logger.Warnf("Network simulator stop timeout")
	}

	s.printFinalStats()

	return nil
}

// GetMessageChannel returns the channel for sending messages
func (s *Simulator) GetMessageChannel() chan<- drone.Message {
	return s.messageCh
}

// calculateDistance calculates the distance between two positions in meters
func (s *Simulator) calculateDistance(pos1, pos2 drone.Position) float64 {
	// Convert from MAVLink format (degrees * 1e7) to degrees
	lat1 := pos1.Lat / 1e7
	lon1 := pos1.Lon / 1e7
	lat2 := pos2.Lat / 1e7
	lon2 := pos2.Lon / 1e7

	// Haversine formula for great-circle distance
	const r = 6_371_000 // Earth's radius in meters

	fi1 := lat1 * math.Pi / 180
	fi2 := lat2 * math.Pi / 180
	delta1 := fi2 - fi1
	delta2 := (lon2 - lon1) * math.Pi / 180
	sinDelta1 := math.Sin(delta1 / 2)
	sinDelta2 := math.Sin(delta2 / 2)
	a := sinDelta1*sinDelta1 + math.Cos(fi1)*math.Cos(fi2)*sinDelta2*sinDelta2
	res := 2 * r * math.Asin(math.Sqrt(a))

	return res
}

// calculateDeliveryProbability calculates message delivery probability based on distance
func (s *Simulator) calculateDeliveryProbability(distance float64) float64 {
	if distance > s.config.MaxRange {
		return 0.0
	}

	// Linear degradation with distance + base packet loss
	rangeFactor := 1.0 - (distance / s.config.MaxRange)

	return rangeFactor * (1.0 - s.config.BasePacketLoss)
}

// processMessage processes a single message through the network simulation
func (s *Simulator) processMessage(msg drone.Message) {
	atomic.AddInt64(&s.stats.TotalMessages, 1)

	sender, exists := s.drones[msg.SenderID]
	if !exists {
		s.logger.Warnf("Unknown sender drone %d", msg.SenderID)
		return
	}

	if !sender.IsConnected() {
		atomic.AddInt64(&s.stats.DisconnectedMessages, 1)
		s.logger.Infof("Drone %d disconnected, message dropped", msg.SenderID)

		return
	}

	// Check for random disconnection
	if rand.Float64() < s.config.DisconnectProbability {
		atomic.AddInt64(&s.stats.DisconnectedMessages, 1)
		s.logger.Debugf("Drone %d randomly disconnected", msg.SenderID)

		return
	}

	senderPos := sender.GetPosition()
	delivered := 0
	lost := 0
	outOfRange := 0

	// Use spatial index to find neighbors
	neighbors := s.findNeighbors(senderPos)

	// Broadcast to neighbors
	for _, receiver := range neighbors {
		if receiver.ID == msg.SenderID || !receiver.IsConnected() {
			continue
		}

		receiverPos := receiver.GetPosition()
		distance := s.calculateDistance(senderPos, receiverPos)

		if distance > s.config.MaxRange {
			outOfRange++

			s.logger.Debugf(
				"%d→%d [OUT_OF_RANGE] (%.1fm > %.1fm)",
				msg.SenderID, receiver.ID, distance, s.config.MaxRange,
			)

			continue
		}

		probability := s.calculateDeliveryProbability(distance)
		if rand.Float64() < probability {
			// Message delivered successfully
			receiver.ReceiveMessage(msg)

			delivered++

			s.logger.Debugf(
				"%d→%d [DELIVERED] (%.1fm, p=%.2f)",
				msg.SenderID, receiver.ID, distance, probability,
			)
		} else {
			// Message lost due to network conditions
			lost++

			s.logger.Debugf(
				"%d→%d [LOST] (%.1fm, p=%.2f)",
				msg.SenderID, receiver.ID, distance, probability,
			)
		}
	}

	// Update statistics
	atomic.AddInt64(&s.stats.DeliveredMessages, int64(delivered))
	atomic.AddInt64(&s.stats.LostMessages, int64(lost))
	atomic.AddInt64(&s.stats.OutOfRangeMessages, int64(outOfRange))
	s.logger.Debugf(
		"Drone %d broadcast: %d delivered, %d lost, %d out-of-range",
		msg.SenderID, delivered, lost, outOfRange,
	)
}

// messageWorker processes messages from the message channel
func (s *Simulator) messageWorker(ctx context.Context, workerID int) {
	defer s.wg.Done()

	s.logger.Debugf("Network worker %d started", workerID)

	for {
		select {
		case <-ctx.Done():
			return
		case <-s.shutdownCh:
			return
		case msg := <-s.messageCh:
			s.processMessage(msg)
		}
	}
}

// indexUpdater periodically updates the spatial index
func (s *Simulator) indexUpdater(ctx context.Context) {
	defer s.wg.Done()
	ticker := time.NewTicker(time.Second / time.Duration(s.config.UpdateRateHz))
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return
		case <-s.shutdownCh:
			return
		case <-ticker.C:
			s.updateIndex()
		}
	}
}

// updateIndex rebuilds the spatial index with current drone positions
func (s *Simulator) updateIndex() {
	s.index.mutex.Lock()
	defer s.index.mutex.Unlock()

	s.index.tree = rtree.RTree{}
	for _, d := range s.drones {
		if d.IsConnected() {
			pos := d.GetPosition()
			s.index.tree.Insert(
				[2]float64{float64(pos.Lat), float64(pos.Lon)},
				[2]float64{float64(pos.Lat), float64(pos.Lon)},
				d.ID,
			)
		}
	}
}

// findNeighbors finds neighbors within MaxRange using the spatial index
func (s *Simulator) findNeighbors(pos drone.Position) []*drone.Drone {
	s.index.mutex.RLock()
	defer s.index.mutex.RUnlock()

	var neighbors []*drone.Drone
	// Approximate search radius in degrees * 1e7
	// 1 degree latitude is approx 111km
	radius := s.config.MaxRange * 1e7 / 111000.0

	searchMin := [2]float64{float64(pos.Lat) - radius, float64(pos.Lon) - radius}
	searchMax := [2]float64{float64(pos.Lat) + radius, float64(pos.Lon) + radius}

	s.index.tree.Search(searchMin, searchMax, func(min, max [2]float64, data interface{}) bool {
		id := data.(int)
		if drone, ok := s.drones[id]; ok {
			neighbors = append(neighbors, drone)
		}
		return true
	})
	return neighbors
}

// statsReporter periodically reports network statistics
func (s *Simulator) statsReporter(ctx context.Context) {
	defer s.wg.Done()

	ticker := time.NewTicker(30 * time.Second) // Report every 30 seconds
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return
		case <-s.shutdownCh:
			return
		case <-ticker.C:
			s.printStats()
		}
	}
}

// printStats prints current network statistics
func (s *Simulator) printStats() {
	s.statsmu.RLock()
	stats := s.stats
	s.statsmu.RUnlock()

	total := atomic.LoadInt64(&stats.TotalMessages)
	delivered := atomic.LoadInt64(&stats.DeliveredMessages)
	lost := atomic.LoadInt64(&stats.LostMessages)
	outOfRange := atomic.LoadInt64(&stats.OutOfRangeMessages)
	disconnected := atomic.LoadInt64(&stats.DisconnectedMessages)

	if total > 0 {
		deliveryRate := float64(delivered) / float64(total) * 100
		s.logger.Infof(
			"Network stats: %d total, %d delivered (%.1f%%), %d lost, %d out-of-range, %d disconnected",
			total, delivered, deliveryRate, lost, outOfRange, disconnected,
		)
	}
}

// printFinalStats prints final network statistics
func (s *Simulator) printFinalStats() {
	s.logger.Infof("\nFINAL NETWORK STATISTICS:")
	s.logger.Infof("═══════════════════════════")

	total := atomic.LoadInt64(&s.stats.TotalMessages)
	delivered := atomic.LoadInt64(&s.stats.DeliveredMessages)
	lost := atomic.LoadInt64(&s.stats.LostMessages)
	outOfRange := atomic.LoadInt64(&s.stats.OutOfRangeMessages)
	disconnected := atomic.LoadInt64(&s.stats.DisconnectedMessages)

	s.logger.Infof("Total messages:      %d", total)
	s.logger.Infof("Delivered:           %d", delivered)
	s.logger.Infof("Lost (network):      %d", lost)
	s.logger.Infof("Out of range:        %d", outOfRange)
	s.logger.Infof("Disconnected:        %d", disconnected)

	if total > 0 {
		deliveryRate := float64(delivered) / float64(total) * 100
		lossRate := float64(lost) / float64(total) * 100

		s.logger.Infof("Delivery rate:       %.2f%%", deliveryRate)
		s.logger.Infof("Network loss rate:   %.2f%%", lossRate)
	}
}

// GetStats returns current network statistics
func (s *Simulator) GetStats() NetworkStats {
	return NetworkStats{
		TotalMessages:        atomic.LoadInt64(&s.stats.TotalMessages),
		DeliveredMessages:    atomic.LoadInt64(&s.stats.DeliveredMessages),
		LostMessages:         atomic.LoadInt64(&s.stats.LostMessages),
		OutOfRangeMessages:   atomic.LoadInt64(&s.stats.OutOfRangeMessages),
		DisconnectedMessages: atomic.LoadInt64(&s.stats.DisconnectedMessages),
	}
}

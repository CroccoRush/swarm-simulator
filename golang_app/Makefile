# Makefile for Swarm Simulator v2.0 (Go)

# Build variables
BINARY_NAME=simulator
CONFIG_GEN_NAME=config-generator
MAVPROXY_NAME=go-mavproxy
PARAM_GEN_NAME=param-generator
BUILD_DIR=bin
GO_FILES=$(shell find . -name "*.go" -type f)

# Version info
VERSION := $(shell git describe --tags --always --dirty 2>/dev/null || echo "dev")
BUILD_TIME := $(shell date -u '+%Y-%m-%d_%H:%M:%S')
GO_VERSION := $(shell go version | cut -d ' ' -f 3)

# Build flags
LDFLAGS=-ldflags "-X main.Version=$(VERSION) -X main.BuildTime=$(BUILD_TIME) -X main.GoVersion=$(GO_VERSION)"

.PHONY: all build clean test run help deps check benchmark param-generator

# Default target
all: clean deps build

# Build the simulator
build: $(BUILD_DIR)/$(BINARY_NAME) $(BUILD_DIR)/$(CONFIG_GEN_NAME) $(BUILD_DIR)/$(MAVPROXY_NAME) $(BUILD_DIR)/$(PARAM_GEN_NAME)

$(BUILD_DIR)/$(BINARY_NAME): $(GO_FILES)
	@echo "Building simulator..."
	@mkdir -p $(BUILD_DIR)
	go build $(LDFLAGS) -o $(BUILD_DIR)/$(BINARY_NAME) ./cmd/simulator
	@echo "Simulator built: $(BUILD_DIR)/$(BINARY_NAME)"

$(BUILD_DIR)/$(CONFIG_GEN_NAME): $(GO_FILES)
	@echo "Building config generator..."
	@mkdir -p $(BUILD_DIR)
	go build $(LDFLAGS) -o $(BUILD_DIR)/$(CONFIG_GEN_NAME) ./cmd/config-generator
	@echo "Config generator built: $(BUILD_DIR)/$(CONFIG_GEN_NAME)"

$(BUILD_DIR)/$(MAVPROXY_NAME): $(GO_FILES)
	@echo "Building Go MAVProxy..."
	@mkdir -p $(BUILD_DIR)
	go build $(LDFLAGS) -o $(BUILD_DIR)/$(MAVPROXY_NAME) ./cmd/go-mavproxy
	@echo "Go MAVProxy built: $(BUILD_DIR)/$(MAVPROXY_NAME)"

$(BUILD_DIR)/$(PARAM_GEN_NAME): $(GO_FILES)
	@echo "Building param generator..."
	@mkdir -p $(BUILD_DIR)
	go build $(LDFLAGS) -o $(BUILD_DIR)/$(PARAM_GEN_NAME) ./cmd/param-generator
	@echo "Param generator built: $(BUILD_DIR)/$(PARAM_GEN_NAME)"

# Alias for param generator
param-generator: $(BUILD_DIR)/$(PARAM_GEN_NAME)

# Alias for config generator
config-generator: $(BUILD_DIR)/$(CONFIG_GEN_NAME)

# Install dependencies
deps:
	@echo "Installing dependencies..."
	go mod tidy
	go mod download
	@echo "Dependencies installed"

# Run tests
test:
	@echo "Running tests..."
	go test -v ./...
	@echo "Tests completed"

# Run benchmarks
benchmark:
	@echo "Running benchmarks..."
	go test -bench=. -benchmem ./...
	@echo "Benchmarks completed"

# Clean build artifacts
clean:
	@echo "Cleaning..."
	rm -rf $(BUILD_DIR)
	go clean
	@echo "Cleaned"

# Check code quality
check:
	@echo "Checking code quality..."
	go fmt ./...
	go vet ./...
	golint ./... 2>/dev/null || echo "Consider installing golint: go install golang.org/x/lint/golint@latest"
	@echo "Code check completed"

# Install tools
install-tools:
	@echo "Installing development tools..."
	go install golang.org/x/lint/golint@latest
	go install github.com/golangci/golangci-lint/cmd/golangci-lint@latest
	@echo "Tools installed"

# Run linter
lint:
	@echo "Running linter..."
	golangci-lint run ./...
	@echo "Linting completed"

# Development build (faster, no optimizations)
dev:
	@echo "Building for development..."
	@mkdir -p $(BUILD_DIR)
	go build -o $(BUILD_DIR)/$(BINARY_NAME) ./cmd/simulator
	go build -o $(BUILD_DIR)/$(CONFIG_GEN_NAME) ./cmd/config-generator
	go build -o $(BUILD_DIR)/$(MAVPROXY_NAME) ./cmd/go-mavproxy
	go build -o $(BUILD_DIR)/$(PARAM_GEN_NAME) ./cmd/param-generator
	@echo "Development build completed"

# Production build (optimized)
prod:
	@echo "Building for production..."
	@mkdir -p $(BUILD_DIR)
	CGO_ENABLED=0 go build -a -installsuffix cgo $(LDFLAGS) -o $(BUILD_DIR)/$(BINARY_NAME) ./cmd/simulator
	CGO_ENABLED=0 go build -a -installsuffix cgo $(LDFLAGS) -o $(BUILD_DIR)/$(CONFIG_GEN_NAME) ./cmd/config-generator
	CGO_ENABLED=0 go build -a -installsuffix cgo $(LDFLAGS) -o $(BUILD_DIR)/$(MAVPROXY_NAME) ./cmd/go-mavproxy
	CGO_ENABLED=0 go build -a -installsuffix cgo $(LDFLAGS) -o $(BUILD_DIR)/$(PARAM_GEN_NAME) ./cmd/param-generator
	@echo "Production build completed"

# Cross-compilation targets
build-linux:
	@echo "Building for Linux..."
	@mkdir -p $(BUILD_DIR)
	GOOS=linux GOARCH=amd64 go build $(LDFLAGS) -o $(BUILD_DIR)/$(BINARY_NAME)-linux ./cmd/simulator
	GOOS=linux GOARCH=amd64 go build $(LDFLAGS) -o $(BUILD_DIR)/$(CONFIG_GEN_NAME)-linux ./cmd/config-generator
	@echo "Linux build completed"

build-windows:
	@echo "Building for Windows..."
	@mkdir -p $(BUILD_DIR)
	GOOS=windows GOARCH=amd64 go build $(LDFLAGS) -o $(BUILD_DIR)/$(BINARY_NAME).exe ./cmd/simulator
	GOOS=windows GOARCH=amd64 go build $(LDFLAGS) -o $(BUILD_DIR)/$(CONFIG_GEN_NAME).exe ./cmd/config-generator
	@echo "Windows build completed"

build-macos:
	@echo "Building for macOS..."
	@mkdir -p $(BUILD_DIR)
	GOOS=darwin GOARCH=amd64 go build $(LDFLAGS) -o $(BUILD_DIR)/$(BINARY_NAME)-macos ./cmd/simulator
	GOOS=darwin GOARCH=amd64 go build $(LDFLAGS) -o $(BUILD_DIR)/$(CONFIG_GEN_NAME)-macos ./cmd/config-generator
	@echo "macOS build completed"

# Build for all platforms
build-all: build-linux build-windows build-macos
	@echo "All platform builds completed"

# Real ArduPilot demos (require SITL)
demo-real: build
	@echo "Real ArduPilot demo (requires SITL running)..."
	./$(BUILD_DIR)/$(BINARY_NAME) -config config.json -mode experiment

demo-large: build
	@echo "Running large demo (100 drones)..."
	./$(BUILD_DIR)/$(CONFIG_GEN_NAME) -drones 100 -formation grid -spacing 30 -output demo_large.json
	./$(BUILD_DIR)/$(BINARY_NAME) -config demo_large.json -mode experiment

# Performance testing
perf-test: build
	@echo "Running performance test..."
	./$(BUILD_DIR)/$(CONFIG_GEN_NAME) -drones 50 -output perf_test.json
	time ./$(BUILD_DIR)/$(BINARY_NAME) -config perf_test.json -mode experiment

# Test Unix Domain Sockets
test-unix-sockets: build
	@echo "Testing Unix Domain Sockets..."
	@echo "Make sure ArduPilot SITL supports Unix sockets and is configured properly"
	./$(BUILD_DIR)/$(BINARY_NAME) -config config_unix_sockets.json -mode experiment

test-mixed-connections: build
	@echo "Testing mixed TCP/Unix connections..."
	@echo "Make sure ArduPilot SITL supports both TCP and Unix sockets"
	./$(BUILD_DIR)/$(BINARY_NAME) -config config_mixed_connections.json -mode experiment

# Generate configurations with different Serial5 types
gen-tcp-config: config-generator
	@echo "Generating TCP configuration..."
	./$(BUILD_DIR)/$(CONFIG_GEN_NAME) -drones 10 -formation grid -spacing 50 -serial5-type tcp -output config_tcp_10.json

gen-unix-config: config-generator
	@echo "Generating Unix sockets configuration..."
	./$(BUILD_DIR)/$(CONFIG_GEN_NAME) -drones 10 -formation grid -spacing 50 -serial5-type unix -output config_unix_10.json

gen-large-unix-config: config-generator
	@echo "Generating large Unix sockets configuration..."
	./$(BUILD_DIR)/$(CONFIG_GEN_NAME) -drones 100 -formation circle -spacing 20 -serial5-type unix -serial5-base-path /tmp/swarm_sitl -output config_unix_100.json

gen-50-drones-tcp: config-generator
	@echo "Generating TCP configuration for 50 drones..."
	./$(BUILD_DIR)/$(CONFIG_GEN_NAME) -drones 50 -formation grid -spacing 30 -serial5-type tcp -output ../config_50_tcp.json

gen-50-drones-unix: config-generator
	@echo "Generating Unix sockets configuration for 50 drones..."
	./$(BUILD_DIR)/$(CONFIG_GEN_NAME) -drones 50 -formation circle -spacing 25 -serial5-type unix -serial5-base-path /tmp/swarm_sitl -output ../config_50_unix.json

# Memory profiling
profile-mem: build
	@echo "Running memory profiling..."
	./$(BUILD_DIR)/$(CONFIG_GEN_NAME) -drones 50 -output profile_test.json
	go run -memprofile=mem.prof ./cmd/simulator -config profile_test.json -mode experiment
	go tool pprof mem.prof

# CPU profiling
profile-cpu: build
	@echo "Running CPU profiling..."
	./$(BUILD_DIR)/$(CONFIG_GEN_NAME) -drones 50 -output profile_test.json
	go run -cpuprofile=cpu.prof ./cmd/simulator -config profile_test.json -mode experiment
	go tool pprof cpu.prof

# Docker build
docker-build:
	@echo "Building Docker image..."
	docker build -t swarm-simulator:latest .
	@echo "Docker image built"

# Create release
release: clean build-all
	@echo "Creating release..."
	@mkdir -p release
	cp $(BUILD_DIR)/* release/
	cp README.md release/
	tar -czf release/swarm-simulator-$(VERSION).tar.gz -C release .
	@echo "Release created: release/swarm-simulator-$(VERSION).tar.gz"

# Run with race detection
race:
	@echo "Running with race detection..."
	go run -race ./cmd/simulator -config config.json -mode experiment

# Update dependencies
update:
	@echo "Updating dependencies..."
	go get -u ./...
	go mod tidy
	@echo "Dependencies updated"

# Show project statistics
stats:
	@echo "Project Statistics:"
	@echo "======================"
	@echo "Go files: $(shell find . -name "*.go" | wc -l)"
	@echo "Lines of code: $(shell find . -name "*.go" -exec cat {} \; | wc -l)"
	@echo "Binary size: $(shell ls -lh $(BUILD_DIR)/$(BINARY_NAME) 2>/dev/null | awk '{print $$5}' || echo 'Not built')"
	@echo "Dependencies: $(shell go list -m all | wc -l)"

# Help
help:
	@echo "Swarm Simulator v2.0 (Go) - Build System"
	@echo "============================================="
	@echo ""
	@echo "Available targets:"
	@echo ""
	@echo "Build targets:"
	@echo "  build      - Build both simulator and config generator"
	@echo "  dev        - Fast development build"
	@echo "  prod       - Optimized production build"
	@echo "  build-all  - Build for all platforms (Linux, Windows, macOS)"
	@echo ""
	@echo "Testing targets:"
	@echo "  test       - Run unit tests"
	@echo "  benchmark  - Run performance benchmarks"
	@echo "  race       - Run with race detection"
	@echo ""
	@echo "Quality targets:"
	@echo "  check      - Run code formatting and basic checks"
	@echo "  lint       - Run comprehensive linting"
	@echo "  install-tools - Install development tools"
	@echo ""
	@echo "Demo targets:"
	@echo "  demo-large  - Run demo with 100 drones"
	@echo "  test-unix-sockets - Test Unix Domain Sockets support"
	@echo "  test-mixed-connections - Test mixed TCP/Unix connections"
	@echo "  gen-tcp-config - Generate TCP configuration (10 drones)"
	@echo "  gen-unix-config - Generate Unix sockets configuration (10 drones)"
	@echo "  gen-large-unix-config - Generate large Unix config (100 drones)"
	@echo "  gen-50-drones-tcp - Generate TCP configuration (50 drones)"
	@echo "  gen-50-drones-unix - Generate Unix sockets configuration (50 drones)"
	@echo ""
	@echo "Profiling targets:"
	@echo "  perf-test   - Basic performance test"
	@echo "  profile-mem - Memory profiling"
	@echo "  profile-cpu - CPU profiling"
	@echo ""
	@echo "Utility targets:"
	@echo "  clean      - Clean build artifacts"
	@echo "  deps       - Install dependencies"
	@echo "  update     - Update dependencies"
	@echo "  stats      - Show project statistics"
	@echo "  help       - Show this help"
	@echo ""
	@echo "Release targets:"
	@echo "  release    - Create release package"
	@echo "  docker-build - Build Docker image"
	@echo ""
	@echo "Examples:"
	@echo "  make build && make demo-large"
	@echo "  make prod && ./bin/simulator -config my_config.json"
	@echo "  make perf-test"
	@echo "  make test-unix-sockets"

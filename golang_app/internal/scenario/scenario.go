package scenario

import (
	"encoding/json"
	"fmt"
	"io/ioutil"
	"os"
	"path/filepath"

	"gopkg.in/yaml.v2"
)

// Scenario defines the structure for an entire experiment scenario file.
// It contains metadata and a list of actions to be executed.
type Scenario struct {
	Name        string   `json:"name" yaml:"name"`
	Description string   `json:"description,omitempty" yaml:"description,omitempty"`
	Actions     []Action `json:"actions" yaml:"actions"`
}

// Action is a flexible type to represent a single command and its parameters.
// It's a map that can hold any key-value pairs from the scenario file.
//
// Example in YAML:
//
//   - command: takeoff
//     drones: all
//     altitude: 10.0
//
//   - command: rc_override
//     drones: [0, 1]
//     roll: 1600
//     throttle: 1550
//
//   - command: wait
//     duration: 5
type Action map[string]interface{}

// LoadScenario reads a scenario file from the given path, automatically detecting
// whether it is JSON or YAML based on the file extension.
func LoadScenario(path string) (*Scenario, error) {
	// 1. Read the file content
	data, err := ioutil.ReadFile(path)
	if err != nil {
		if os.IsNotExist(err) {
			return nil, fmt.Errorf("scenario file not found: %s", path)
		}
		return nil, fmt.Errorf("failed to read scenario file '%s': %w", path, err)
	}

	// 2. Unmarshal based on file extension
	var scenario Scenario
	ext := filepath.Ext(path)

	switch ext {
	case ".json":
		if err := json.Unmarshal(data, &scenario); err != nil {
			return nil, fmt.Errorf("failed to parse JSON scenario file '%s': %w", path, err)
		}
	case ".yaml", ".yml":
		if err := yaml.Unmarshal(data, &scenario); err != nil {
			return nil, fmt.Errorf("failed to parse YAML scenario file '%s': %w", path, err)
		}
	default:
		return nil, fmt.Errorf("unsupported scenario file format: '%s'. Please use .json or .yaml", ext)
	}

	// 3. Basic validation
	if scenario.Name == "" {
		return nil, fmt.Errorf("scenario file '%s' is missing a 'name'", path)
	}
	if len(scenario.Actions) == 0 {
		return nil, fmt.Errorf("scenario file '%s' has no 'actions' defined", path)
	}

	return &scenario, nil
}

package main

import (
	"flag"
	"fmt"
	"io/ioutil"
	"log"
	"os"
	"path/filepath"
	"text/template"
)

type ParamData struct {
	SwarmID int
	SysID   int
}

func main() {
	var (
		count        int
		outputDir    string
		templateFile string
	)

	flag.IntVar(&count, "count", 10, "Number of parameter files to generate")
	flag.StringVar(&outputDir, "output", "../params", "Output directory for parameter files")
	flag.StringVar(&templateFile, "template", "../params/copter_template.parm", "Template file path")
	flag.Parse()

	if count <= 0 {
		log.Fatal("Count must be positive")
	}

	// Read template file
	templateContent, err := ioutil.ReadFile(templateFile)
	if err != nil {
		log.Fatalf("Failed to read template file %s: %v", templateFile, err)
	}

	// Create output directory if it doesn't exist
	if err := os.MkdirAll(outputDir, 0o755); err != nil {
		log.Fatalf("Failed to create output directory: %v", err)
	}

	// Parse template
	tmpl, err := template.New("param").Parse(string(templateContent))
	if err != nil {
		log.Fatalf("Failed to parse template: %v", err)
	}

	log.Printf("Using template: %s\n", templateFile)
	log.Printf("Generating %d parameter files in %s/\n", count, outputDir)

	for i := 0; i < count; i++ {
		data := ParamData{
			SwarmID: i + 1, // Swarm_ID starts from 1
			SysID:   i + 1, // SYSID_THISMAV starts from 1
		}

		filename := fmt.Sprintf("copter_%d.parm", i)
		filePath := filepath.Join(outputDir, filename)

		file, err := os.Create(filePath)
		if err != nil {
			log.Fatalf("Failed to create file %s: %v", filePath, err)
		}

		if err := tmpl.Execute(file, data); err != nil {
			file.Close()
			log.Fatalf("Failed to execute template for %s: %v", filePath, err)
		}

		file.Close()
		log.Printf(
			"Generated %s (Swarm_ID=%d, SYSID_THISMAV=%d)\n",
			filename, data.SwarmID, data.SysID,
		)
	}

	log.Printf("Successfully generated %d parameter files!\n", count)
}

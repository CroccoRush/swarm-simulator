#!/bin/bash

# Script to generate parameter files for N drones

# Default values
COUNT=10
OUTPUT_DIR="../params"
TEMPLATE_FILE="../params/copter_template.parm"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -c|--count)
            COUNT="$2"
            shift 2
            ;;
        -o|--output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        -t|--template)
            TEMPLATE_FILE="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  -c, --count COUNT      Number of parameter files to generate (default: 10)"
            echo "  -o, --output DIR       Output directory (default: ../params)"
            echo "  -t, --template FILE    Template file path (default: ../params/copter_template.parm)"
            echo "  -h, --help             Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                           # Generate 10 files in ../params/"
            echo "  $0 -c 50                     # Generate 50 files"
            echo "  $0 -c 100 -o /tmp            # Generate 100 files in /tmp/"
            echo "  $0 -t custom_template.parm   # Use custom template"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

# Build param generator if not exists
if [ ! -f "bin/param-generator" ]; then
    echo "Building param generator..."
    make param-generator
    if [ $? -ne 0 ]; then
        echo "Failed to build param generator"
        exit 1
    fi
fi

# Run param generator
echo "Generating $COUNT parameter files..."
./bin/param-generator -count $COUNT -output $OUTPUT_DIR -template $TEMPLATE_FILE

if [ $? -eq 0 ]; then
    echo "Parameter files generated successfully!"
    echo "Files location: $OUTPUT_DIR/"
    echo "Files: copter_0.parm to copter_$((COUNT-1)).parm"
else
    echo "Failed to generate parameter files"
    exit 1
fi

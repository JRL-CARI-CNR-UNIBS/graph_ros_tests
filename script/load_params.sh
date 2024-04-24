#!/bin/bash

# Check if an argument was provided
if [ $# -eq 0 ]; then
    echo "Please provide the configuration file name as input."
    exit 1
fi

# Save the input (file path) into a variable
FILE_NAME="$1"

# Get the package path
PACKAGE_PATH=$(rospack find graph_ros1_tests)

# Check if rospack succeeded
if [ -z "$PACKAGE_PATH" ]; then
    echo "Failed to find the ROS package 'graph_ros1_tests'."
    exit 1
fi

# Check if the file exists
if [ ! -f "$PACKAGE_PATH/config/$FILE_NAME" ]; then
    echo "File does not exist: $PACKAGE_PATH/config/$FILE_NAME"
    exit 1
fi

# Load parameters contained in the file
cnr_param_server --path-to-file "$PACKAGE_PATH/config/$FILE_NAME"

# Exit gracefully by returning a status
exit 0

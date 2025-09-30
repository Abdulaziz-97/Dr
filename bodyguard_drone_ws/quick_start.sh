#!/bin/bash
# Quick start script for Bodyguard Drone Simulation

set -e

# Source ROS 2
source /opt/ros/humble/setup.bash 2>/dev/null || echo "Warning: ROS 2 not sourced"

# Source workspace
source install/setup.bash 2>/dev/null || echo "Warning: Workspace not built yet"

# Check arguments
HEADLESS=${1:-false}

echo "================================================"
echo "Starting Bodyguard Drone Simulation"
echo "================================================"

if [ "$HEADLESS" = "headless" ] || [ "$HEADLESS" = "true" ]; then
    echo "Running in HEADLESS mode (no GUI)"
    ros2 launch bodyguard_drone full_demo.launch.py headless:=true
else
    echo "Running with GUI"
    ros2 launch bodyguard_drone full_demo.launch.py
fi

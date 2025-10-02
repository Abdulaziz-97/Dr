#!/bin/bash
# Quick launcher for the tmux drone monitoring system

echo "🚁 Starting Bodyguard Drone Monitoring System..."
echo ""

# Make sure the main script is executable
chmod +x ~/Dr/bodyguard_drone_ws/start_drone_tmux.sh

# Run the main tmux script
~/Dr/bodyguard_drone_ws/start_drone_tmux.sh

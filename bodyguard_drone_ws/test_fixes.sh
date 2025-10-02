#!/bin/bash
# Test script for the fixed mission system

echo "🔧 Testing Bodyguard Drone Fixes"
echo "=================================="

cd ~/Dr/bodyguard_drone_ws

echo "1. Building with fixes..."
colcon build --symlink-install
if [ $? -ne 0 ]; then
    echo "❌ Build failed!"
    exit 1
fi

echo "2. Sourcing workspace..."
source install/setup.bash

echo "3. Testing YOLO model loading..."
python3 test_yolo.py

echo "4. Running mission system with fixes..."
echo "   - YOLO loads before missions start"
echo "   - Enhanced person detection (more objects converted)"
echo "   - Position hold prevents drone falling"
echo "   - Stable head-level positioning"

echo ""
echo "🚀 Starting mission system..."
echo "Expected behavior:"
echo "  - Waits for 10+ detections before starting"
echo "  - Better person detection (vase, umbrella, chair, etc.)"
echo "  - Drone maintains position when no detections"
echo "  - Moves to head level and stays there"
echo ""

ros2 launch bodyguard_drone mission_demo.launch.py

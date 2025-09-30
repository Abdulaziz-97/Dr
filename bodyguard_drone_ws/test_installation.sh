#!/bin/bash
# Test script to verify Bodyguard Drone installation

echo "================================================"
echo "Bodyguard Drone - Installation Verification"
echo "================================================"
echo ""

ERRORS=0

# Test 1: ROS 2 Installation
echo "✓ Checking ROS 2 installation..."
if command -v ros2 &> /dev/null; then
    echo "  ✅ ROS 2 found: $(ros2 --version 2>&1 | head -n1)"
else
    echo "  ❌ ROS 2 not found"
    ERRORS=$((ERRORS + 1))
fi
echo ""

# Test 2: Gazebo Installation
echo "✓ Checking Gazebo installation..."
if command -v gazebo &> /dev/null; then
    echo "  ✅ Gazebo found: $(gazebo --version 2>&1 | head -n1)"
else
    echo "  ❌ Gazebo not found"
    ERRORS=$((ERRORS + 1))
fi
echo ""

# Test 3: Python Dependencies
echo "✓ Checking Python dependencies..."

# Check ultralytics
python3 -c "import ultralytics" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "  ✅ ultralytics (YOLO) installed"
else
    echo "  ❌ ultralytics not found"
    ERRORS=$((ERRORS + 1))
fi

# Check transformers
python3 -c "import transformers" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "  ✅ transformers (DINOv3) installed"
else
    echo "  ❌ transformers not found"
    ERRORS=$((ERRORS + 1))
fi

# Check torch
python3 -c "import torch" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "  ✅ PyTorch installed"
else
    echo "  ❌ PyTorch not found"
    ERRORS=$((ERRORS + 1))
fi

# Check OpenCV
python3 -c "import cv2" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "  ✅ OpenCV installed"
else
    echo "  ❌ OpenCV not found"
    ERRORS=$((ERRORS + 1))
fi

# Check Whisper
python3 -c "import whisper" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "  ✅ Whisper installed"
else
    echo "  ⚠️  Whisper not found (optional)"
fi
echo ""

# Test 4: ROS 2 Workspace
echo "✓ Checking ROS 2 workspace..."
if [ -f "install/setup.bash" ]; then
    source install/setup.bash 2>/dev/null
    
    # Check if package exists
    ros2 pkg list 2>/dev/null | grep -q bodyguard_drone
    if [ $? -eq 0 ]; then
        echo "  ✅ Workspace built successfully"
        echo "  ✅ Package 'bodyguard_drone' found"
    else
        echo "  ❌ Package 'bodyguard_drone' not found"
        echo "  ℹ️  Run: colcon build --symlink-install"
        ERRORS=$((ERRORS + 1))
    fi
else
    echo "  ❌ Workspace not built"
    echo "  ℹ️  Run: colcon build --symlink-install"
    ERRORS=$((ERRORS + 1))
fi
echo ""

# Test 5: Gazebo Models
echo "✓ Checking Gazebo models..."
if [ -f "src/bodyguard_drone/models/drone/model.sdf" ]; then
    echo "  ✅ Drone model found"
else
    echo "  ❌ Drone model not found"
    ERRORS=$((ERRORS + 1))
fi

if [ -f "src/bodyguard_drone/models/person_with_ring/model.sdf" ]; then
    echo "  ✅ Person model found"
else
    echo "  ❌ Person model not found"
    ERRORS=$((ERRORS + 1))
fi

if [ -f "src/bodyguard_drone/worlds/bodyguard_world.sdf" ]; then
    echo "  ✅ World file found"
else
    echo "  ❌ World file not found"
    ERRORS=$((ERRORS + 1))
fi
echo ""

# Test 6: TTS
echo "✓ Checking Text-to-Speech..."
if command -v espeak &> /dev/null; then
    echo "  ✅ espeak found"
elif command -v festival &> /dev/null; then
    echo "  ✅ festival found"
elif command -v say &> /dev/null; then
    echo "  ✅ macOS say found"
else
    echo "  ⚠️  No TTS engine found (will use print-only mode)"
fi
echo ""

# Test 7: Node executables
echo "✓ Checking ROS 2 nodes..."
if [ -f "install/setup.bash" ]; then
    source install/setup.bash 2>/dev/null
    
    NODES=(
        "perception_node"
        "navigation_node"
        "user_event_node"
        "voice_cmd_node"
        "tts_node"
        "drone_control_node"
        "command_parser_node"
    )
    
    for node in "${NODES[@]}"; do
        if ros2 pkg executables bodyguard_drone 2>/dev/null | grep -q "$node"; then
            echo "  ✅ $node"
        else
            echo "  ❌ $node not found"
            ERRORS=$((ERRORS + 1))
        fi
    done
fi
echo ""

# Summary
echo "================================================"
if [ $ERRORS -eq 0 ]; then
    echo "✅ All checks passed! Installation successful."
    echo ""
    echo "Next steps:"
    echo "1. Source workspace: source install/setup.bash"
    echo "2. Launch simulation: ./quick_start.sh headless"
else
    echo "❌ Installation incomplete. $ERRORS error(s) found."
    echo ""
    echo "Please run: ./install_dependencies.sh"
fi
echo "================================================"

exit $ERRORS

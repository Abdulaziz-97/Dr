#!/bin/bash
# Installation script for Bodyguard Drone Simulation
# Target: AWS EC2 Ubuntu 22.04 LTS

set -e

echo "================================================"
echo "Bodyguard Drone - Dependency Installation"
echo "================================================"

# Update system
echo "Updating system packages..."
sudo apt update
sudo apt upgrade -y

# Install system dependencies
echo "Installing system dependencies..."
sudo apt install -y \
    python3-pip \
    python3-dev \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    ffmpeg \
    espeak \
    portaudio19-dev \
    libsndfile1 \
    xterm

# Install Gazebo
echo "Installing Gazebo..."
sudo apt install -y gazebo
sudo apt install -y libgazebo-dev

# Install ROS 2 Humble (if not already installed)
echo "Checking for ROS 2 Humble..."
if ! command -v ros2 &> /dev/null; then
    echo "Installing ROS 2 Humble..."
    
    # Setup sources
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install -y curl
    
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    sudo apt update
    sudo apt install -y ros-humble-desktop
    sudo apt install -y ros-humble-gazebo-ros-pkgs
    sudo apt install -y ros-humble-cv-bridge
    sudo apt install -y python3-colcon-common-extensions
    
    # Source ROS 2
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source /opt/ros/humble/setup.bash
else
    echo "ROS 2 already installed"
    source /opt/ros/humble/setup.bash
fi

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install --upgrade pip
pip3 install -r requirements.txt

# Download YOLO model
echo "Downloading YOLO model..."
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"

# Setup Gazebo model path
echo "Setting up Gazebo model path..."
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/src/bodyguard_drone/models" >> ~/.bashrc

# Build ROS 2 workspace
echo "Building ROS 2 workspace..."
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Source workspace
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc

echo "================================================"
echo "Installation complete!"
echo "================================================"
echo ""
echo "To get started:"
echo "1. Source the workspace: source install/setup.bash"
echo "2. Launch the simulation: ros2 launch bodyguard_drone full_demo.launch.py"
echo ""
echo "For headless mode (no GUI):"
echo "ros2 launch bodyguard_drone full_demo.launch.py headless:=true"
echo ""

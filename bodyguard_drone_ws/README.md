# Bodyguard Drone Simulation

A sophisticated bodyguard drone simulation built with **Gazebo**, **ROS 2**, and **Machine Learning** for autonomous navigation, user assistance, and emergency response.

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Architecture](#architecture)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [ROS 2 Topics](#ros-2-topics)
- [Troubleshooting](#troubleshooting)
- [AWS Deployment](#aws-deployment)

## 🚁 Overview

The Bodyguard Drone is an intelligent autonomous drone designed to assist and protect users in indoor environments. It uses computer vision, voice commands, and sensor data to:

- Navigate autonomously to landmarks (e.g., coffee shops)
- Detect and avoid obstacles
- Respond to emergency events (panic button, fall detection)
- Provide voice-guided navigation assistance
- Monitor user health through smart ring sensor

## ✨ Features

### Perception Layer
- **YOLOv8** object detection for person, obstacles, and landmarks
- **DINOv3** feature extraction for place recognition
- Real-time camera feed processing (CPU-optimized)

### User Interaction
- **Smart Ring Simulation** - Panic/fall/heartbeat detection
- **Voice Commands** - Whisper-based speech-to-text
- **Text-to-Speech** - Audio feedback for navigation instructions

### Navigation
- Autonomous pathfinding to landmarks
- Obstacle detection and avoidance
- Emergency response protocols
- Static instruction generation

### Simulation
- Realistic Gazebo world with walls, doors, furniture
- Quadrotor drone model with camera and IMU
- Human model with smart ring visualization

## 🏗 Architecture

```
┌─────────────────┐
│  Gazebo World   │
│  - Drone Model  │
│  - Person Model │
│  - Obstacles    │
└────────┬────────┘
         │
    ┌────▼────────────────────┐
    │   Perception Layer      │
    │  - YOLO Detection       │
    │  - DINOv3 Features      │
    └────────┬────────────────┘
             │
    ┌────────▼────────────────┐
    │   Navigation Layer      │
    │  - Path Planning        │
    │  - Instruction Gen      │
    └────────┬────────────────┘
             │
    ┌────────▼────────────────┐
    │   Control Layer         │
    │  - Drone Control        │
    │  - Motor Commands       │
    └─────────────────────────┘
             │
    ┌────────▼────────────────┐
    │   User Interface        │
    │  - Ring Events          │
    │  - Voice Commands       │
    │  - TTS Output           │
    └─────────────────────────┘
```

## 📦 Requirements

### Hardware
- **AWS EC2**: m7i-flex.large or t2.micro (Free Tier)
- **OS**: Ubuntu 22.04 LTS
- **CPU**: Optimized for non-GPU instances
- **RAM**: Minimum 4GB

### Software
- ROS 2 Humble
- Gazebo Classic or Ignition
- Python 3.8+
- OpenCV
- PyTorch (CPU)
- Ultralytics (YOLO)
- Transformers (DINOv3)
- Whisper (OpenAI)

## 🚀 Installation

### Option 1: Automated Installation (Recommended)

```bash
# Clone the repository
cd ~/
git clone <your-repo-url>
cd bodyguard_drone_ws

# Run installation script
chmod +x install_dependencies.sh
./install_dependencies.sh
```

### Option 2: Manual Installation

#### Step 1: Install System Dependencies

```bash
sudo apt update && sudo apt upgrade -y

# Install essential packages
sudo apt install -y \
    python3-pip \
    python3-dev \
    build-essential \
    cmake \
    git \
    wget \
    ffmpeg \
    espeak \
    portaudio19-dev \
    libsndfile1 \
    xterm
```

#### Step 2: Install Gazebo

```bash
sudo apt install -y gazebo libgazebo-dev
```

#### Step 3: Install ROS 2 Humble

```bash
# Setup sources
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-gazebo-ros-pkgs
sudo apt install -y ros-humble-cv-bridge
sudo apt install -y python3-colcon-common-extensions

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

#### Step 4: Install Python Dependencies

```bash
pip3 install --upgrade pip
pip3 install -r requirements.txt

# Download YOLO model
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

#### Step 5: Build the Workspace

```bash
cd ~/bodyguard_drone_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Source workspace
echo "source ~/bodyguard_drone_ws/install/setup.bash" >> ~/.bashrc
source ~/bodyguard_drone_ws/install/setup.bash
```

## 🎮 Usage

### Quick Start

```bash
# Option 1: Use quick start script
./quick_start.sh

# Option 2: Manual launch
source install/setup.bash
ros2 launch bodyguard_drone full_demo.launch.py
```

### Headless Mode (No GUI)

For AWS EC2 instances without display:

```bash
./quick_start.sh headless
# OR
ros2 launch bodyguard_drone full_demo.launch.py headless:=true
```

### Interactive Controls

#### Ring Sensor Events (User Event Node)
Press keys in the user_event_node terminal:
- `p` - Trigger PANIC event
- `f` - Trigger FALL event
- `h` - Send HEARTBEAT data
- `n` - Return to NORMAL status
- `q` - Quit

#### Voice Commands (Simulated)
The system automatically simulates these commands:
- "Guide me to coffee shop"
- "What's ahead"
- "Emergency help needed"

### Manual Node Testing

Launch individual nodes for testing:

```bash
# Perception node
ros2 run bodyguard_drone perception_node

# Navigation node
ros2 run bodyguard_drone navigation_node

# User event simulator
ros2 run bodyguard_drone user_event_node

# TTS node
ros2 run bodyguard_drone tts_node
```

## 📁 Project Structure

```
bodyguard_drone_ws/
├── src/
│   └── bodyguard_drone/
│       ├── bodyguard_drone/          # Python nodes
│       │   ├── perception_node.py
│       │   ├── navigation_node.py
│       │   ├── user_event_node.py
│       │   ├── voice_cmd_node.py
│       │   ├── tts_node.py
│       │   ├── drone_control_node.py
│       │   └── command_parser_node.py
│       ├── launch/                   # Launch files
│       │   └── full_demo.launch.py
│       ├── msg/                      # Custom messages
│       │   ├── Detection.msg
│       │   ├── DetectionArray.msg
│       │   ├── UserEvent.msg
│       │   ├── VoiceCommand.msg
│       │   ├── NavigationInstruction.msg
│       │   └── DINOv3Features.msg
│       ├── models/                   # Gazebo models
│       │   ├── drone/
│       │   ├── person_with_ring/
│       │   ├── coffee_shop/
│       │   └── obstacles/
│       ├── worlds/                   # Gazebo worlds
│       │   └── bodyguard_world.sdf
│       ├── config/                   # Configuration files
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── setup.py
├── requirements.txt
├── install_dependencies.sh
├── quick_start.sh
└── README.md
```

## 📡 ROS 2 Topics

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/detections` | `DetectionArray` | YOLO object detections |
| `/dinov3_features` | `DINOv3Features` | DINOv3 feature embeddings |
| `/user_event` | `UserEvent` | Ring sensor events |
| `/voice_cmd` | `VoiceCommand` | Transcribed voice commands |
| `/tts_instruction` | `NavigationInstruction` | Navigation instructions |
| `/drone/cmd_vel` | `Twist` | Velocity commands |
| `/drone/odom` | `Odometry` | Drone odometry |
| `/drone/imu` | `Imu` | IMU sensor data |
| `/drone/camera/image_raw` | `Image` | Camera feed |

### Subscribed Topics

Nodes subscribe to relevant topics based on their function. See individual node documentation for details.

## 🔧 Troubleshooting

### Common Issues

#### 1. Gazebo Not Starting
```bash
# Check if Gazebo is installed
gazebo --version

# Kill existing Gazebo processes
killall gzserver gzclient

# Restart
ros2 launch bodyguard_drone full_demo.launch.py
```

#### 2. YOLO Model Not Found
```bash
# Manually download YOLO model
python3 -c "from ultralytics import YOLO; model = YOLO('yolov8n.pt')"
```

#### 3. Camera Feed Not Publishing
```bash
# Check camera topic
ros2 topic list | grep camera

# Echo camera topic
ros2 topic echo /drone/camera/image_raw --no-arr
```

#### 4. Slow Performance on CPU
The system is optimized for CPU but may be slow:
- Reduce frame processing rate in `perception_node.py` (increase `process_every_n_frames`)
- Use smaller YOLO model: `yolov8n.pt` (already using nano)
- Disable DINOv3 if not needed

#### 5. TTS Not Working
```bash
# Install espeak
sudo apt install espeak

# Test espeak
espeak "Hello world"
```

## ☁️ AWS Deployment

### EC2 Instance Setup

#### 1. Launch Instance
- **AMI**: Ubuntu 22.04 LTS
- **Instance Type**: m7i-flex.large (or t2.micro for testing)
- **Storage**: 20GB minimum
- **Security Group**: Allow SSH (port 22)

#### 2. Connect to Instance
```bash
ssh -i your-key.pem ubuntu@<ec2-public-ip>
```

#### 3. Install VNC (Optional, for GUI)
```bash
sudo apt install xfce4 xfce4-goodies tightvncserver -y

# Configure VNC
vncserver
vncserver -kill :1
echo "startxfce4 &" >> ~/.vnc/xstartup
chmod +x ~/.vnc/xstartup
vncserver
```

#### 4. Install Bodyguard Drone
```bash
# Clone repository
git clone <your-repo-url>
cd bodyguard_drone_ws

# Run installation
chmod +x install_dependencies.sh
./install_dependencies.sh
```

#### 5. Run Simulation
```bash
# Headless mode (recommended for EC2)
./quick_start.sh headless
```

### Performance Tips

1. **CPU Optimization**: Already configured for CPU-only inference
2. **Frame Rate**: Adjust processing frequency in perception node
3. **Memory**: Monitor with `htop`, scale instance if needed
4. **GPU Option**: Switch to g4dn.xlarge for real-time performance

## 📊 Monitoring

### ROS 2 Tools

```bash
# View node graph
rqt_graph

# Monitor topics
ros2 topic list
ros2 topic hz /detections

# Check node status
ros2 node list
ros2 node info /perception_node

# View logs
ros2 run rqt_console rqt_console
```

### System Monitoring

```bash
# CPU/Memory usage
htop

# GPU usage (if applicable)
nvidia-smi

# Network
iftop
```

## 🛠 Development

### Adding New Features

1. **New Detection Class**
   - Modify `perception_node.py`
   - Add class to YOLO training or use existing classes

2. **New Voice Command**
   - Update `command_parser_node.py`
   - Add pattern to `command_patterns` dict

3. **New Navigation Behavior**
   - Extend `navigation_node.py`
   - Implement new goal type

### Testing

```bash
# Run tests
colcon test

# Specific package tests
colcon test --packages-select bodyguard_drone
```

## 📝 License

MIT License - See LICENSE file for details

## 👥 Contributors

Bodyguard Drone Team

## 📧 Support

For issues and questions:
- GitHub Issues: [Create Issue]
- Email: team@bodyguarddrone.com

---

**Built with ❤️ using ROS 2, Gazebo, and Machine Learning**

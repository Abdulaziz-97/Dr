# Getting Started with Bodyguard Drone

## 🚀 Quick Start Guide

This guide will get you up and running with the Bodyguard Drone simulation in minutes.

## Prerequisites

You need:
- **Ubuntu 22.04 LTS** (AWS EC2 or local machine)
- **Internet connection** (for downloading dependencies)
- **4GB+ RAM** recommended
- **10GB+ disk space**

## Installation (5 Steps)

### Step 1: Navigate to Project
```bash
cd ~/bodyguard_drone_ws
```

### Step 2: Run Installation Script
```bash
chmod +x install_dependencies.sh
./install_dependencies.sh
```

⏱️ **This takes 20-30 minutes** - Perfect time for a coffee break! ☕

### Step 3: Verify Installation
```bash
chmod +x test_installation.sh
./test_installation.sh
```

You should see all green checkmarks ✅

### Step 4: Source Workspace
```bash
source install/setup.bash
# Or permanently add to bashrc:
echo "source ~/bodyguard_drone_ws/install/setup.bash" >> ~/.bashrc
```

### Step 5: Launch!
```bash
chmod +x quick_start.sh
./quick_start.sh headless
```

## 🎮 Using the Simulation

### Interacting with the Drone

Once launched, you'll see multiple terminals with different nodes running.

#### User Event Node (Ring Sensor)
Find the terminal showing `user_event_node` and press:

| Key | Action | Result |
|-----|--------|--------|
| `p` | Panic | Drone stops, announces emergency |
| `f` | Fall detection | Emergency response activated |
| `h` | Heartbeat | Normal health data sent |
| `n` | Normal | Return to normal operation |
| `q` | Quit | Stop the node |

#### Voice Commands (Automatic)
The system simulates these commands automatically:
1. "Guide me to coffee shop" - Drone navigates to landmark
2. "What's ahead" - Drone describes visible objects
3. "Emergency help needed" - Emergency response

#### Monitoring

Open a new terminal and try these commands:

```bash
# List all active topics
ros2 topic list

# See object detections
ros2 topic echo /detections

# Monitor navigation instructions
ros2 topic echo /tts_instruction

# View system graph
rqt_graph
```

## 📊 What to Expect

### First Launch
1. **Gazebo starts** - World loads with drone and person
2. **Nodes initialize** - All 7 nodes start up
3. **Models download** - YOLO model downloads (first time only)
4. **Perception starts** - Camera begins processing

### During Operation
- **Console output**: Navigation instructions appear
- **TTS audio**: If espeak is installed, you'll hear voice guidance
- **Detection logs**: Object detection results printed
- **Event responses**: User events trigger appropriate actions

### Performance Notes
- **CPU mode**: Slow but functional (~1 FPS detection)
- **Frame processing**: Every 30th frame analyzed
- **Expected delay**: 1-2 seconds between detections

## 🔍 Troubleshooting Quick Fixes

### Problem: "ros2: command not found"
```bash
source /opt/ros/humble/setup.bash
source ~/bodyguard_drone_ws/install/setup.bash
```

### Problem: "Package 'bodyguard_drone' not found"
```bash
cd ~/bodyguard_drone_ws
colcon build --symlink-install
source install/setup.bash
```

### Problem: Gazebo won't start
```bash
killall gzserver gzclient
./quick_start.sh headless
```

### Problem: Out of memory
```bash
# Add swap space
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

### Problem: No detections appearing
This is normal! Detection runs slowly on CPU. Wait 30-60 seconds after launch.

## 📖 Next Steps

### Learn More
- Read `README.md` for complete documentation
- Check `DEPLOYMENT.md` for AWS setup details
- See `PROJECT_SUMMARY.md` for technical details

### Customize
1. **Change detection rate**: Edit `perception_node.py`, modify `process_every_n_frames`
2. **Add voice commands**: Edit `command_parser_node.py`, add patterns
3. **Modify world**: Edit `worlds/bodyguard_world.sdf`, add objects

### Test Individual Components

```bash
# Terminal 1: Launch just Gazebo
gazebo src/bodyguard_drone/worlds/bodyguard_world.sdf

# Terminal 2: Run single node
ros2 run bodyguard_drone perception_node

# Terminal 3: Monitor
ros2 topic echo /detections
```

## 🎯 Common Use Cases

### Use Case 1: Test Emergency Response
```bash
1. Launch simulation: ./quick_start.sh headless
2. Wait for initialization (~30 seconds)
3. Press 'p' in user_event terminal
4. Observe: Drone stops, emergency announcement
5. Press 'n' to return to normal
```

### Use Case 2: Test Object Detection
```bash
1. Launch simulation
2. Wait 60 seconds for first detections
3. Run: ros2 topic echo /detections
4. See detected objects: person, table, coffee_shop
```

### Use Case 3: Test Navigation
```bash
1. Launch simulation
2. Wait for simulated voice command: "Guide me to coffee shop"
3. Observe navigation instructions in TTS output
4. Monitor: ros2 topic echo /tts_instruction
```

## 💡 Tips & Best Practices

### On AWS EC2
- ✅ Use headless mode: `./quick_start.sh headless`
- ✅ Monitor resources: `htop`
- ✅ Stop instance when not in use
- ✅ Use screen/tmux for persistent sessions

### For Development
- ✅ Use `--symlink-install` when building
- ✅ Check logs: `ros2 run rqt_console rqt_console`
- ✅ Visualize: `rqt_graph`
- ✅ Test nodes individually before full launch

### For Performance
- ✅ Increase frame skip for faster response
- ✅ Disable DINOv3 if not needed
- ✅ Use GPU instance (g4dn.xlarge) for real-time
- ✅ Monitor with: `ros2 topic hz /detections`

## 🆘 Getting Help

### Check Logs
```bash
# ROS logs
ros2 run rqt_console rqt_console

# Build logs
cat log/latest_build/bodyguard_drone/stdout.log

# System logs
journalctl -xe
```

### Common Questions

**Q: How long until I see detections?**
A: 30-60 seconds after launch (CPU is slow)

**Q: Can I use this on Windows?**
A: Not directly. Use WSL2 with Ubuntu 22.04 or AWS EC2

**Q: Do I need a GPU?**
A: No! Works on CPU (slow but functional)

**Q: How do I stop the simulation?**
A: Press Ctrl+C in the launch terminal

**Q: Can I add my own landmarks?**
A: Yes! Edit `worlds/bodyguard_world.sdf`

## 📞 Support Resources

- **Documentation**: `README.md` (main docs)
- **Deployment**: `DEPLOYMENT.md` (AWS guide)
- **Technical**: `PROJECT_SUMMARY.md` (architecture)
- **Verification**: `test_installation.sh` (check install)

## ✨ What's Next?

1. ✅ Got it running? Great!
2. 📚 Read the full `README.md`
3. 🔧 Customize for your needs
4. 🚀 Deploy to AWS EC2
5. 💪 Contribute improvements!

---

**Welcome to Bodyguard Drone! Happy flying! 🚁**

*Having issues? Run `./test_installation.sh` and check the output.*

# AWS EC2 Deployment Guide

## Quick Deployment Checklist

### Pre-Deployment
- [ ] AWS Account setup
- [ ] SSH key pair created
- [ ] Security group configured (port 22 for SSH)
- [ ] EC2 instance type selected (m7i-flex.large or t2.micro)

### Deployment Steps

#### 1. Launch EC2 Instance

```bash
# Via AWS Console:
1. Navigate to EC2 Dashboard
2. Click "Launch Instance"
3. Select: Ubuntu Server 22.04 LTS (64-bit x86)
4. Instance Type: m7i-flex.large (recommended) or t2.micro (free tier)
5. Storage: 20GB minimum
6. Security Group: Allow SSH (port 22)
7. Launch with your key pair
```

#### 2. Connect to Instance

```bash
# SSH into instance
ssh -i /path/to/your-key.pem ubuntu@<EC2-PUBLIC-IP>
```

#### 3. Update System

```bash
sudo apt update && sudo apt upgrade -y
```

#### 4. Clone Repository

```bash
cd ~
git clone <YOUR-REPO-URL> bodyguard_drone_ws
cd bodyguard_drone_ws
```

#### 5. Run Installation Script

```bash
chmod +x install_dependencies.sh
./install_dependencies.sh
```

**Note**: This will take 20-30 minutes depending on instance type.

#### 6. Verify Installation

```bash
# Source workspace
source ~/bodyguard_drone_ws/install/setup.bash

# Check ROS 2
ros2 --version

# Check Gazebo
gazebo --version

# List ROS packages
ros2 pkg list | grep bodyguard_drone
```

#### 7. Launch Simulation

```bash
# Headless mode (recommended for EC2 without GUI)
cd ~/bodyguard_drone_ws
./quick_start.sh headless
```

### Optional: VNC Setup for GUI Access

If you want to use Gazebo GUI on EC2:

#### 1. Install VNC Server

```bash
sudo apt install -y xfce4 xfce4-goodies tightvncserver
```

#### 2. Configure VNC

```bash
# Start VNC server (will prompt for password)
vncserver

# Kill server
vncserver -kill :1

# Configure startup
echo "startxfce4 &" >> ~/.vnc/xstartup
chmod +x ~/.vnc/xstartup

# Restart VNC
vncserver -geometry 1920x1080 -depth 24
```

#### 3. SSH Tunnel (on local machine)

```bash
ssh -L 5901:localhost:5901 -i /path/to/key.pem ubuntu@<EC2-IP>
```

#### 4. Connect VNC Client

- Use VNC viewer (RealVNC, TigerVNC, etc.)
- Connect to: `localhost:5901`
- Enter VNC password

#### 5. Launch with GUI

```bash
cd ~/bodyguard_drone_ws
./quick_start.sh  # Without headless flag
```

## Performance Optimization

### CPU-Only Mode (Default)

The project is configured for CPU-only inference:
- YOLO: yolov8n.pt (nano model)
- DINOv3: Small variant
- Frame processing: Every 30th frame

### Improve Performance

#### 1. Reduce Frame Processing Rate

Edit `bodyguard_drone/perception_node.py`:
```python
self.process_every_n_frames = 60  # Process less frequently
```

#### 2. Disable DINOv3 (if not needed)

Comment out DINOv3 in `perception_node.py`:
```python
# if DINOV3_AVAILABLE:
#     self.extract_features(cv_image, msg.header)
```

#### 3. Use GPU Instance (for real-time)

Switch to g4dn.xlarge:
- Install CUDA drivers
- Update torch installation: `pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118`
- Change device in nodes: `.to('cuda')` instead of `.to('cpu')`

## Monitoring

### System Resources

```bash
# Install htop
sudo apt install htop

# Monitor CPU/Memory
htop
```

### ROS 2 Monitoring

```bash
# List active nodes
ros2 node list

# Monitor topics
ros2 topic hz /detections
ros2 topic hz /drone/camera/image_raw

# View logs
ros2 run rqt_console rqt_console
```

### Check Perception Performance

```bash
# Subscribe to detections
ros2 topic echo /detections
```

## Cost Optimization

### Free Tier (t2.micro)
- Good for: Testing, development
- Limitations: Slow ML inference (~1-2s per frame)
- Monthly: 750 hours free

### m7i-flex.large
- Good for: Full functionality testing
- Performance: ~500ms per frame
- Cost: ~$0.07/hour

### g4dn.xlarge (GPU)
- Good for: Real-time demo
- Performance: ~30 FPS
- Cost: ~$0.52/hour

**Tip**: Start/stop instances as needed. Terminate when not in use.

## Troubleshooting

### Issue: Out of Memory

```bash
# Check memory usage
free -h

# If needed, add swap
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

### Issue: Gazebo Crashes

```bash
# Kill existing processes
killall gzserver gzclient

# Restart simulation
./quick_start.sh headless
```

### Issue: YOLO Model Not Found

```bash
# Download model manually
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

### Issue: ROS 2 Nodes Not Starting

```bash
# Source workspace
source /opt/ros/humble/setup.bash
source ~/bodyguard_drone_ws/install/setup.bash

# Rebuild
cd ~/bodyguard_drone_ws
colcon build --symlink-install
```

## Security Best Practices

1. **SSH Key**: Never commit private keys
2. **Security Groups**: Restrict to your IP only
3. **Updates**: Regularly update system packages
4. **Firewall**: Enable UFW if needed

```bash
sudo ufw allow OpenSSH
sudo ufw enable
```

## Backup & Restore

### Backup

```bash
# Create AMI from EC2 instance
# AWS Console -> EC2 -> Select Instance -> Actions -> Image -> Create Image
```

### Restore

```bash
# Launch new instance from saved AMI
```

## Scaling

### Horizontal Scaling
- Run multiple drones by spawning different namespaces
- Load balancing with ROS 2 multi-machine setup

### Vertical Scaling
- Upgrade instance type as needed
- Use GPU instances for real-time processing

## Support

For deployment issues:
- Check logs: `~/bodyguard_drone_ws/log/`
- ROS 2 logs: `ros2 run rqt_console rqt_console`
- System logs: `journalctl -xe`

---

**Happy Deploying! 🚀**

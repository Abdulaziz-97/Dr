#!/bin/bash

# Bodyguard Drone tmux Monitoring and Control Script
# This script creates a comprehensive monitoring environment for the drone system

SESSION="bodyguard_drone"

# Kill existing session if it exists
tmux kill-session -t $SESSION 2>/dev/null

echo "Starting Bodyguard Drone tmux session..."

# Create new session with first window
tmux new-session -d -s $SESSION -n 'Main'

# Window 0: Main Simulation
tmux send-keys -t $SESSION:0 'echo "=== MAIN SIMULATION WINDOW ==="' C-m
tmux send-keys -t $SESSION:0 'export DISPLAY=:1' C-m
tmux send-keys -t $SESSION:0 'export QT_X11_NO_MITSHM=1' C-m
tmux send-keys -t $SESSION:0 'export LIBGL_ALWAYS_SOFTWARE=0' C-m
tmux send-keys -t $SESSION:0 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t $SESSION:0 'source ~/Dr/bodyguard_drone_ws/install/setup.bash' C-m
tmux send-keys -t $SESSION:0 'cd ~/Dr/bodyguard_drone_ws' C-m
tmux send-keys -t $SESSION:0 'echo "Ready to launch simulation. Press Enter to start:"' C-m
tmux send-keys -t $SESSION:0 'read -p "Press Enter to launch Gazebo + RViz: " && vglrun ros2 launch bodyguard_drone full_demo.launch.py headless:=false' C-m

# Window 1: Control Panel (User Events)
tmux new-window -t $SESSION -n 'Control'
tmux send-keys -t $SESSION:1 'echo "=== DRONE CONTROL PANEL ==="' C-m
tmux send-keys -t $SESSION:1 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t $SESSION:1 'source ~/Dr/bodyguard_drone_ws/install/setup.bash' C-m
tmux send-keys -t $SESSION:1 'echo ""' C-m
tmux send-keys -t $SESSION:1 'echo "🚁 BODYGUARD DRONE CONTROL COMMANDS:"' C-m
tmux send-keys -t $SESSION:1 'echo "  p = PANIC MODE (Emergency stop + hover)"' C-m
tmux send-keys -t $SESSION:1 'echo "  f = FALL DETECTION (User fell down)"' C-m
tmux send-keys -t $SESSION:1 'echo "  h = HEARTBEAT (Send health signal)"' C-m
tmux send-keys -t $SESSION:1 'echo "  n = NORMAL MODE (Resume operations)"' C-m
tmux send-keys -t $SESSION:1 'echo "  q = QUIT control node"' C-m
tmux send-keys -t $SESSION:1 'echo ""' C-m
tmux send-keys -t $SESSION:1 'echo "Wait for main simulation to start, then press Enter:"' C-m
tmux send-keys -t $SESSION:1 'read -p "Press Enter to start control interface: " && ros2 run bodyguard_drone user_event_node.py' C-m

# Window 2: Monitoring Dashboard
tmux new-window -t $SESSION -n 'Monitor'
tmux send-keys -t $SESSION:2 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t $SESSION:2 'source ~/Dr/bodyguard_drone_ws/install/setup.bash' C-m

# Split window 2 into 4 panes for different monitoring
tmux split-window -h -t $SESSION:2
tmux split-window -v -t $SESSION:2.0
tmux split-window -v -t $SESSION:2.1

# Pane 2.0: Object Detections
tmux select-pane -t $SESSION:2.0
tmux send-keys -t $SESSION:2.0 'echo "=== OBJECT DETECTIONS ==="' C-m
tmux send-keys -t $SESSION:2.0 'echo "Waiting for simulation to start..."' C-m
tmux send-keys -t $SESSION:2.0 'sleep 5 && echo "Monitoring /detections topic:" && ros2 topic echo /detections --no-arr' C-m

# Pane 2.1: TTS Instructions
tmux select-pane -t $SESSION:2.1
tmux send-keys -t $SESSION:2.1 'echo "=== TTS INSTRUCTIONS ==="' C-m
tmux send-keys -t $SESSION:2.1 'echo "Waiting for simulation to start..."' C-m
tmux send-keys -t $SESSION:2.1 'sleep 5 && echo "Monitoring /tts_instruction topic:" && ros2 topic echo /tts_instruction' C-m

# Pane 2.2: System Status
tmux select-pane -t $SESSION:2.2
tmux send-keys -t $SESSION:2.2 'echo "=== SYSTEM STATUS ==="' C-m
tmux send-keys -t $SESSION:2.2 'echo "Waiting for simulation to start..."' C-m
tmux send-keys -t $SESSION:2.2 'sleep 10 && watch -n 2 "echo \"=== ACTIVE NODES ===\" && ros2 node list && echo \"\" && echo \"=== ACTIVE TOPICS ===\" && ros2 topic list | grep -E \"(detections|tts|user_event|camera)\" && echo \"\" && echo \"=== TOPIC RATES ===\" && timeout 3 ros2 topic hz /detections 2>/dev/null || echo \"No detections yet\""' C-m

# Pane 2.3: Camera & Performance
tmux select-pane -t $SESSION:2.3
tmux send-keys -t $SESSION:2.3 'echo "=== CAMERA & PERFORMANCE ==="' C-m
tmux send-keys -t $SESSION:2.3 'echo "Waiting for simulation to start..."' C-m
tmux send-keys -t $SESSION:2.3 'sleep 8 && watch -n 3 "echo \"=== CAMERA STATUS ===\" && timeout 2 ros2 topic hz /drone/camera/image_raw 2>/dev/null || echo \"Camera not ready\" && echo \"\" && echo \"=== SYSTEM RESOURCES ===\" && free -h | head -2 && echo \"\" && echo \"=== GAZEBO FPS ===\" && ps aux | grep -E \"(gzserver|gzclient)\" | grep -v grep | head -2"' C-m

# Window 3: Debug Console
tmux new-window -t $SESSION -n 'Debug'
tmux send-keys -t $SESSION:3 'echo "=== DEBUG CONSOLE ==="' C-m
tmux send-keys -t $SESSION:3 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t $SESSION:3 'source ~/Dr/bodyguard_drone_ws/install/setup.bash' C-m
tmux send-keys -t $SESSION:3 'echo ""' C-m
tmux send-keys -t $SESSION:3 'echo "🔧 DEBUG COMMANDS:"' C-m
tmux send-keys -t $SESSION:3 'echo "  ros2 node list                    # List all nodes"' C-m
tmux send-keys -t $SESSION:3 'echo "  ros2 topic list                   # List all topics"' C-m
tmux send-keys -t $SESSION:3 'echo "  ros2 topic echo /detections       # See detections"' C-m
tmux send-keys -t $SESSION:3 'echo "  ros2 topic echo /user_event       # See user events"' C-m
tmux send-keys -t $SESSION:3 'echo "  ros2 node info /perception_node   # Node details"' C-m
tmux send-keys -t $SESSION:3 'echo "  rqt_graph                         # Visual node graph"' C-m
tmux send-keys -t $SESSION:3 'echo ""' C-m
tmux send-keys -t $SESSION:3 'echo "Ready for debug commands..."' C-m

# Window 4: Individual Nodes (for manual control)
tmux new-window -t $SESSION -n 'Nodes'
tmux send-keys -t $SESSION:4 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t $SESSION:4 'source ~/Dr/bodyguard_drone_ws/install/setup.bash' C-m

# Split into 3 panes for individual node testing
tmux split-window -h -t $SESSION:4
tmux split-window -v -t $SESSION:4.0

# Pane 4.0: Perception Node
tmux select-pane -t $SESSION:4.0
tmux send-keys -t $SESSION:4.0 'echo "=== PERCEPTION NODE ==="' C-m
tmux send-keys -t $SESSION:4.0 'echo "Manual perception node control"' C-m
tmux send-keys -t $SESSION:4.0 'echo "Run: ros2 run bodyguard_drone perception_node.py"' C-m

# Pane 4.1: Navigation Node  
tmux select-pane -t $SESSION:4.1
tmux send-keys -t $SESSION:4.1 'echo "=== NAVIGATION NODE ==="' C-m
tmux send-keys -t $SESSION:4.1 'echo "Manual navigation node control"' C-m
tmux send-keys -t $SESSION:4.1 'echo "Run: ros2 run bodyguard_drone navigation_node.py"' C-m

# Pane 4.2: TTS Node
tmux select-pane -t $SESSION:4.2
tmux send-keys -t $SESSION:4.2 'echo "=== TTS NODE ==="' C-m
tmux send-keys -t $SESSION:4.2 'echo "Manual TTS node control"' C-m
tmux send-keys -t $SESSION:4.2 'echo "Run: ros2 run bodyguard_drone tts_node.py"' C-m

# Set up status line
tmux set-option -t $SESSION status-left-length 40
tmux set-option -t $SESSION status-left "#[fg=green]🚁 Bodyguard Drone #[fg=yellow]#S #[fg=white]| "
tmux set-option -t $SESSION status-right "#[fg=cyan]%H:%M:%S #[fg=white]| #[fg=magenta]%Y-%m-%d"

# Select the main window
tmux select-window -t $SESSION:0

echo ""
echo "🚁 Bodyguard Drone tmux session created!"
echo ""
echo "📋 WINDOWS:"
echo "  0: Main      - Primary simulation (Gazebo + RViz)"
echo "  1: Control   - Drone commands (panic, fall, etc.)"
echo "  2: Monitor   - System monitoring dashboard"  
echo "  3: Debug     - Debug console for manual commands"
echo "  4: Nodes     - Individual node control"
echo ""
echo "🎮 NAVIGATION:"
echo "  Ctrl+b + 0-4    - Switch to window 0-4"
echo "  Ctrl+b + arrow  - Switch between panes"
echo "  Ctrl+b + z      - Zoom pane (toggle fullscreen)"
echo "  Ctrl+b + d      - Detach (keeps running)"
echo ""
echo "🚀 USAGE:"
echo "  1. Go to window 0 (Main) and start simulation"
echo "  2. Go to window 1 (Control) and start control interface"
echo "  3. Use window 2 (Monitor) to watch system status"
echo "  4. Press 'p' in Control window for PANIC mode!"
echo ""

# Attach to the session
tmux attach-session -t $SESSION

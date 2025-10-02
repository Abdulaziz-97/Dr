#!/bin/bash
# Setup audio playback through VNC/SSH tunnel

echo "🔊 Setting up audio for VNC..."

# Install PulseAudio and audio tools
sudo apt-get update
sudo apt-get install -y pulseaudio pulseaudio-utils alsa-utils

# Configure PulseAudio for network audio
echo "# Network audio configuration for VNC" >> ~/.pulse/default.pa
echo "load-module module-native-protocol-tcp auth-ip-acl=127.0.0.1 port=4713" >> ~/.pulse/default.pa

# Start PulseAudio
pulseaudio --start --verbose

# Set up audio environment variables
echo "export PULSE_RUNTIME_PATH=/tmp/pulse-\$USER" >> ~/.bashrc
echo "export PULSE_SERVER=tcp:localhost:4713" >> ~/.bashrc

# Test audio
echo "Testing audio setup..."
speaker-test -t sine -f 1000 -l 1 &
sleep 2
killall speaker-test

echo "✅ Audio setup complete!"
echo ""
echo "📋 To enable audio in your VNC client:"
echo "  1. On Windows: Use TurboVNC with audio forwarding"
echo "  2. SSH tunnel: ssh -L 4713:localhost:4713 -L 5901:localhost:5901 ..."
echo "  3. Test with: espeak 'Hello from drone'"
echo ""

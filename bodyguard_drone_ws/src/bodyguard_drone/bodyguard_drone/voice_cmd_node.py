#!/usr/bin/env python3
"""
Voice Command Node - Speech-to-Text using Whisper
"""

import rclpy
from rclpy.node import Node
from bodyguard_drone.msg import VoiceCommand

try:
    import whisper
    import soundfile as sf
    import numpy as np
    WHISPER_AVAILABLE = True
except ImportError:
    WHISPER_AVAILABLE = False
    print("Warning: whisper/soundfile not available, voice commands disabled")


class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_cmd_node')
        
        # Publisher
        self.voice_cmd_pub = self.create_publisher(VoiceCommand, '/voice_cmd', 10)
        
        # Initialize Whisper model (CPU)
        if WHISPER_AVAILABLE:
            self.get_logger().info('Loading Whisper model (CPU mode, may be slow)...')
            self.whisper_model = whisper.load_model("base")
        else:
            self.whisper_model = None
            self.get_logger().warn('Whisper not available')
        
        # For demonstration, we'll use a timer to simulate voice commands
        # In real deployment, this would record and transcribe microphone input
        self.demo_commands = [
            "Guide me to coffee shop",
            "What's ahead",
            "Emergency help needed"
        ]
        self.demo_index = 0
        
        # Simulate voice input every 30 seconds
        self.create_timer(30.0, self.simulate_voice_input)
        
        self.get_logger().info('Voice Command Node initialized (demo mode)')

    def simulate_voice_input(self):
        """Simulate voice input for demonstration"""
        if self.demo_index >= len(self.demo_commands):
            return
            
        command_text = self.demo_commands[self.demo_index]
        self.demo_index += 1
        
        # Publish voice command
        msg = VoiceCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.text = command_text
        msg.confidence = 0.95
        
        self.voice_cmd_pub.publish(msg)
        self.get_logger().info(f'Simulated voice command: "{command_text}"')

    def transcribe_audio(self, audio_path):
        """Transcribe audio file using Whisper"""
        if not WHISPER_AVAILABLE:
            return None
            
        try:
            result = self.whisper_model.transcribe(audio_path)
            return result['text']
        except Exception as e:
            self.get_logger().error(f'Transcription error: {str(e)}')
            return None

    def process_microphone_input(self):
        """Process real-time microphone input (placeholder)"""
        # This would be implemented with pyaudio or similar
        # For now, this is a placeholder for future implementation
        pass


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

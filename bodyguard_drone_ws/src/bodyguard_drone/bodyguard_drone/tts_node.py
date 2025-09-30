#!/usr/bin/env python3
"""
TTS Node - Text-to-Speech using VibeVoice (or fallback)
"""

import rclpy
from rclpy.node import Node
from bodyguard_drone.msg import NavigationInstruction
import subprocess
import os


class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        
        # Subscriber
        self.instruction_sub = self.create_subscription(
            NavigationInstruction,
            '/tts_instruction',
            self.instruction_callback,
            10
        )
        
        # Check for TTS availability
        self.tts_method = self.detect_tts_method()
        
        self.get_logger().info(f'TTS Node initialized (method: {self.tts_method})')

    def detect_tts_method(self):
        """Detect available TTS method"""
        # Check for espeak (common on Linux)
        try:
            subprocess.run(['espeak', '--version'], 
                         capture_output=True, check=True)
            return 'espeak'
        except:
            pass
        
        # Check for festival
        try:
            subprocess.run(['festival', '--version'], 
                         capture_output=True, check=True)
            return 'festival'
        except:
            pass
        
        # Check for macOS 'say'
        try:
            subprocess.run(['say', '-v', '?'], 
                         capture_output=True, check=True)
            return 'macos_say'
        except:
            pass
        
        # Fallback to print only
        return 'print_only'

    def instruction_callback(self, msg):
        """Handle navigation instructions and speak them"""
        priority_names = {0: 'LOW', 1: 'NORMAL', 2: 'HIGH', 3: 'EMERGENCY'}
        priority_str = priority_names.get(msg.priority, 'UNKNOWN')
        
        self.get_logger().info(f'[{priority_str}] {msg.instruction}')
        
        # Speak the instruction
        self.speak(msg.instruction, msg.priority)

    def speak(self, text, priority=1):
        """Speak text using available TTS method"""
        try:
            if self.tts_method == 'espeak':
                # Adjust speed based on priority
                speed = 150 if priority >= 2 else 120
                subprocess.Popen(['espeak', '-s', str(speed), text],
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
                
            elif self.tts_method == 'festival':
                subprocess.Popen(['festival', '--tts'],
                               stdin=subprocess.PIPE,
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL).communicate(text.encode())
                
            elif self.tts_method == 'macos_say':
                rate = 250 if priority >= 2 else 200
                subprocess.Popen(['say', '-r', str(rate), text],
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
                
            else:
                # Print only (fallback)
                print(f'🔊 TTS: {text}')
                
        except Exception as e:
            self.get_logger().error(f'TTS error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

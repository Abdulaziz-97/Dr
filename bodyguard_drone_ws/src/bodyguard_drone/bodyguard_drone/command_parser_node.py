#!/usr/bin/env python3
"""
Command Parser Node - Parse voice commands and trigger actions
"""

import rclpy
from rclpy.node import Node
from bodyguard_drone.msg import VoiceCommand, NavigationInstruction
import re


class CommandParserNode(Node):
    def __init__(self):
        super().__init__('command_parser_node')
        
        # Subscriber
        self.voice_cmd_sub = self.create_subscription(
            VoiceCommand,
            '/voice_cmd',
            self.voice_cmd_callback,
            10
        )
        
        # Publisher for navigation goals
        self.nav_goal_pub = self.create_publisher(
            NavigationInstruction,
            '/nav_goal',
            10
        )
        
        # Command patterns
        self.command_patterns = {
            'goto': [
                (r'guide me to (.+)', 'goto_landmark'),
                (r'take me to (.+)', 'goto_landmark'),
                (r'go to (.+)', 'goto_landmark'),
                (r'navigate to (.+)', 'goto_landmark'),
            ],
            'describe': [
                (r"what'?s? ahead", 'describe_ahead'),
                (r'what do you see', 'describe_ahead'),
                (r'describe surroundings?', 'describe_ahead'),
            ],
            'emergency': [
                (r'emergency', 'emergency'),
                (r'help', 'emergency'),
                (r'assist', 'emergency'),
            ],
            'follow': [
                (r'follow me', 'follow_user'),
                (r'stay close', 'follow_user'),
            ],
        }
        
        self.get_logger().info('Command Parser Node initialized')

    def voice_cmd_callback(self, msg):
        """Parse voice command and trigger appropriate action"""
        text = msg.text.lower().strip()
        self.get_logger().info(f'Parsing command: "{text}"')
        
        command_type, param = self.parse_command(text)
        
        if command_type:
            self.execute_command(command_type, param)
        else:
            self.get_logger().warn(f'Unknown command: "{text}"')

    def parse_command(self, text):
        """Parse command text and extract intent and parameters"""
        for category, patterns in self.command_patterns.items():
            for pattern, command_type in patterns:
                match = re.search(pattern, text, re.IGNORECASE)
                if match:
                    param = match.group(1) if match.groups() else None
                    return command_type, param
        
        return None, None

    def execute_command(self, command_type, param=None):
        """Execute parsed command"""
        self.get_logger().info(f'Executing: {command_type} with param: {param}')
        
        # Create navigation goal message
        goal = NavigationInstruction()
        goal.header.stamp = self.get_clock().now().to_msg()
        
        if command_type == 'goto_landmark':
            # Navigate to a specific landmark
            landmark = param.strip() if param else 'unknown'
            goal.instruction = f'goto:{landmark}'
            goal.priority = 1
            self.nav_goal_pub.publish(goal)
            
            # Directly publish to navigation node through a service call
            # For simplicity, we'll use the shared navigation interface
            self.trigger_navigation_goal('goto_landmark', landmark)
            
        elif command_type == 'describe_ahead':
            goal.instruction = 'describe:ahead'
            goal.priority = 1
            self.nav_goal_pub.publish(goal)
            self.trigger_navigation_goal('describe_ahead', None)
            
        elif command_type == 'emergency':
            goal.instruction = 'emergency:user_request'
            goal.priority = 3
            self.nav_goal_pub.publish(goal)
            
        elif command_type == 'follow_user':
            goal.instruction = 'follow:user'
            goal.priority = 1
            self.nav_goal_pub.publish(goal)

    def trigger_navigation_goal(self, goal_type, goal_data):
        """Trigger navigation goal (connects to navigation node)"""
        # This would ideally use a service call to navigation node
        # For now, we publish to a topic that navigation node subscribes to
        pass


def main(args=None):
    rclpy.init(args=args)
    node = CommandParserNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
User Event Node - Simulates ring sensor events (panic, fall, heartbeat)
"""

import rclpy
from rclpy.node import Node
from bodyguard_drone.msg import UserEvent
import sys
import select
import termios
import tty


class UserEventNode(Node):
    def __init__(self):
        super().__init__('user_event_node')
        
        # Publisher
        self.event_pub = self.create_publisher(UserEvent, '/user_event', 10)
        
        # Timer for keyboard input checking
        self.create_timer(0.1, self.check_keyboard_input)
        
        # Store terminal settings
        self.settings = None
        try:
            self.settings = termios.tcgetattr(sys.stdin)
        except:
            self.get_logger().warn('Terminal settings not available (non-interactive mode)')
        
        self.get_logger().info('User Event Node initialized')
        self.get_logger().info('Press: [p] panic, [f] fall, [h] heartbeat, [n] normal, [q] quit')

    def check_keyboard_input(self):
        """Check for keyboard input and publish events"""
        if self.settings is None:
            return
            
        # Check if input is available
        if select.select([sys.stdin], [], [], 0)[0]:
            try:
                # Set terminal to raw mode
                tty.setraw(sys.stdin.fileno())
                key = sys.stdin.read(1)
                
                # Restore terminal settings
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
                
                # Process key
                event = UserEvent()
                event.header.stamp = self.get_clock().now().to_msg()
                
                if key == 'p':
                    event.event_type = 'panic'
                    event.data = 'User pressed panic button'
                    self.event_pub.publish(event)
                    self.get_logger().info('Published PANIC event')
                    
                elif key == 'f':
                    event.event_type = 'fall'
                    event.data = 'Fall detected by accelerometer'
                    self.event_pub.publish(event)
                    self.get_logger().info('Published FALL event')
                    
                elif key == 'h':
                    event.event_type = 'heartbeat'
                    event.data = 'Heart rate: 75 bpm'
                    self.event_pub.publish(event)
                    self.get_logger().info('Published HEARTBEAT event')
                    
                elif key == 'n':
                    event.event_type = 'normal'
                    event.data = 'User status normal'
                    self.event_pub.publish(event)
                    self.get_logger().info('Published NORMAL event')
                    
                elif key == 'q':
                    self.get_logger().info('Quit requested')
                    raise KeyboardInterrupt
                    
            except Exception as e:
                if self.settings:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
                if not isinstance(e, KeyboardInterrupt):
                    self.get_logger().error(f'Keyboard input error: {str(e)}')
                else:
                    raise

    def publish_event(self, event_type, data=''):
        """Publish a user event (for external API use)"""
        event = UserEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.event_type = event_type
        event.data = data
        self.event_pub.publish(event)
        self.get_logger().info(f'Published {event_type} event')

    def destroy_node(self):
        """Restore terminal settings on shutdown"""
        if self.settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UserEventNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

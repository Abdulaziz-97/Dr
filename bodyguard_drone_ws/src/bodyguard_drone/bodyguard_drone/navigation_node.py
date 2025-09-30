#!/usr/bin/env python3
"""
Navigation Node - Path planning and instruction generation
"""

import rclpy
from rclpy.node import Node
from bodyguard_drone.msg import DetectionArray, NavigationInstruction, UserEvent
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        # Navigation state
        self.detected_objects = {}
        self.current_goal = None
        self.drone_pose = None
        self.emergency_mode = False
        
        # Subscribers
        self.detections_sub = self.create_subscription(
            DetectionArray,
            '/detections',
            self.detections_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            10
        )
        
        self.user_event_sub = self.create_subscription(
            UserEvent,
            '/user_event',
            self.user_event_callback,
            10
        )
        
        # Publishers
        self.instruction_pub = self.create_publisher(
            NavigationInstruction,
            '/tts_instruction',
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/drone/cmd_vel',
            10
        )
        
        # Timer for periodic navigation updates
        self.create_timer(1.0, self.navigation_loop)
        
        self.get_logger().info('Navigation node initialized')

    def detections_callback(self, msg):
        """Update detected objects map"""
        self.detected_objects.clear()
        
        for detection in msg.detections:
            if detection.class_name not in self.detected_objects:
                self.detected_objects[detection.class_name] = []
            self.detected_objects[detection.class_name].append({
                'bbox': (detection.x_min, detection.y_min, detection.x_max, detection.y_max),
                'confidence': detection.confidence,
                'distance': detection.distance
            })
        
        self.get_logger().debug(f'Updated object map: {list(self.detected_objects.keys())}')

    def odom_callback(self, msg):
        """Update drone position"""
        self.drone_pose = msg.pose.pose

    def user_event_callback(self, msg):
        """Handle user events from ring sensor"""
        self.get_logger().info(f'User event received: {msg.event_type}')
        
        if msg.event_type in ['panic', 'fall']:
            self.emergency_mode = True
            instruction = NavigationInstruction()
            instruction.header.stamp = self.get_clock().now().to_msg()
            instruction.instruction = f"Emergency detected: {msg.event_type}. Hovering nearby for assistance."
            instruction.priority = 3  # Emergency priority
            self.instruction_pub.publish(instruction)
            
            # Stop movement in emergency
            self.stop_drone()
        elif msg.event_type == 'normal':
            self.emergency_mode = False

    def set_navigation_goal(self, goal_type, goal_data):
        """Set navigation goal based on command"""
        self.current_goal = {'type': goal_type, 'data': goal_data}
        self.get_logger().info(f'Navigation goal set: {goal_type}')

    def navigation_loop(self):
        """Periodic navigation logic"""
        if self.emergency_mode:
            return
            
        if self.current_goal is None:
            return
        
        goal_type = self.current_goal['type']
        
        if goal_type == 'goto_landmark':
            self.navigate_to_landmark(self.current_goal['data'])
        elif goal_type == 'describe_ahead':
            self.describe_ahead()
        elif goal_type == 'avoid_obstacle':
            self.avoid_obstacles()

    def navigate_to_landmark(self, landmark_name):
        """Navigate to a specific landmark"""
        if landmark_name in self.detected_objects:
            objects = self.detected_objects[landmark_name]
            if objects:
                closest = min(objects, key=lambda x: x['distance'])
                distance = closest['distance']
                
                instruction = NavigationInstruction()
                instruction.header.stamp = self.get_clock().now().to_msg()
                
                if distance > 2.0:
                    instruction.instruction = f"Heading towards {landmark_name}. Distance: {distance:.1f} meters."
                    instruction.priority = 1
                    self.move_forward()
                elif distance > 0.5:
                    instruction.instruction = f"{landmark_name} is ahead. Almost there."
                    instruction.priority = 1
                    self.move_forward_slow()
                else:
                    instruction.instruction = f"Destination reached: {landmark_name}."
                    instruction.priority = 2
                    self.stop_drone()
                    self.current_goal = None
                
                self.instruction_pub.publish(instruction)
        else:
            instruction = NavigationInstruction()
            instruction.header.stamp = self.get_clock().now().to_msg()
            instruction.instruction = f"Searching for {landmark_name}."
            instruction.priority = 1
            self.instruction_pub.publish(instruction)

    def describe_ahead(self):
        """Describe what's ahead of the drone"""
        instruction = NavigationInstruction()
        instruction.header.stamp = self.get_clock().now().to_msg()
        
        if not self.detected_objects:
            instruction.instruction = "No objects detected ahead."
        else:
            objects_str = ', '.join(self.detected_objects.keys())
            instruction.instruction = f"Ahead I see: {objects_str}."
        
        instruction.priority = 1
        self.instruction_pub.publish(instruction)
        self.current_goal = None

    def avoid_obstacles(self):
        """Check for and avoid obstacles"""
        if 'person' in self.detected_objects:
            # Stay clear of person
            instruction = NavigationInstruction()
            instruction.header.stamp = self.get_clock().now().to_msg()
            instruction.instruction = "Person detected. Maintaining safe distance."
            instruction.priority = 2
            self.instruction_pub.publish(instruction)
            self.stop_drone()
            return
        
        # Check for obstacles
        obstacle_classes = ['obstacle_box', 'table', 'chair']
        obstacles_detected = any(cls in self.detected_objects for cls in obstacle_classes)
        
        if obstacles_detected:
            instruction = NavigationInstruction()
            instruction.header.stamp = self.get_clock().now().to_msg()
            instruction.instruction = "Obstacle ahead. Adjusting path."
            instruction.priority = 2
            self.instruction_pub.publish(instruction)
            self.turn_left()

    def move_forward(self):
        """Move drone forward"""
        cmd = Twist()
        cmd.linear.x = 0.5
        self.cmd_vel_pub.publish(cmd)

    def move_forward_slow(self):
        """Move drone forward slowly"""
        cmd = Twist()
        cmd.linear.x = 0.2
        self.cmd_vel_pub.publish(cmd)

    def turn_left(self):
        """Turn drone left"""
        cmd = Twist()
        cmd.angular.z = 0.5
        self.cmd_vel_pub.publish(cmd)

    def stop_drone(self):
        """Stop drone movement"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

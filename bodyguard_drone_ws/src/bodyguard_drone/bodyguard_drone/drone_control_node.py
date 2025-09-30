#!/usr/bin/env python3
"""
Drone Control Node - Low-level flight control and stabilization
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math


class DroneControlNode(Node):
    def __init__(self):
        super().__init__('drone_control_node')
        
        # Drone state
        self.current_velocity = Twist()
        self.imu_data = None
        self.altitude = 0.5  # Default hover altitude
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/drone/imu',
            self.imu_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            10
        )
        
        # Command velocity subscriber (from navigation)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/drone/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher for actual motor commands (this would go to Gazebo plugin)
        self.motor_cmd_pub = self.create_publisher(
            Twist,
            '/drone/cmd_vel_actual',
            10
        )
        
        # Control loop timer (100 Hz)
        self.create_timer(0.01, self.control_loop)
        
        self.get_logger().info('Drone Control Node initialized')

    def imu_callback(self, msg):
        """Update IMU data"""
        self.imu_data = msg

    def odom_callback(self, msg):
        """Update odometry data"""
        self.altitude = msg.pose.pose.position.z

    def cmd_vel_callback(self, msg):
        """Receive velocity commands from navigation"""
        self.current_velocity = msg

    def control_loop(self):
        """Main control loop for stabilization and command execution"""
        # In a real implementation, this would:
        # 1. Read IMU data
        # 2. Apply PID control for stabilization
        # 3. Convert velocity commands to motor speeds
        # 4. Publish motor commands
        
        # For this simulation, we just pass through the velocity commands
        cmd = Twist()
        cmd.linear.x = self.current_velocity.linear.x
        cmd.linear.y = self.current_velocity.linear.y
        cmd.linear.z = self.current_velocity.linear.z
        cmd.angular.x = self.current_velocity.angular.x
        cmd.angular.y = self.current_velocity.angular.y
        cmd.angular.z = self.current_velocity.angular.z
        
        # Ensure we maintain altitude (simple altitude hold)
        if self.altitude < 0.4:
            cmd.linear.z = 0.1  # Climb
        elif self.altitude > 0.6:
            cmd.linear.z = -0.1  # Descend
        
        self.motor_cmd_pub.publish(cmd)

    def hover(self):
        """Command drone to hover in place"""
        cmd = Twist()
        self.motor_cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DroneControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

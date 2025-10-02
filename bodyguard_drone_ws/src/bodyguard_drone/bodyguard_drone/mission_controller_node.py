#!/usr/bin/env python3
"""
Mission Controller Node - Manages sequential mission execution
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from bodyguard_drone.msg import UserEvent, NavigationInstruction, DetectionArray
import time
import threading


class MissionControllerNode(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # Mission state
        self.current_mission = 0
        self.mission_start_time = None
        self.person_detected = False
        self.mission_active = False
        self.perception_ready = False
        self.detections_received = 0
        
        # Publishers
        self.user_event_pub = self.create_publisher(UserEvent, '/user_event', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.tts_pub = self.create_publisher(NavigationInstruction, '/tts_instruction', 10)
        
        # Subscribers
        self.detections_sub = self.create_subscription(
            DetectionArray, '/detections', self.detections_callback, 10)
        
        # Mission timer
        self.mission_timer = self.create_timer(1.0, self.mission_loop)
        
        self.get_logger().info('Mission Controller initialized - Waiting for perception node...')
        
        # Wait for perception to be ready before starting missions
        self.readiness_timer = self.create_timer(2.0, self.check_readiness)

    def start_mission_sequence(self):
        """Start the mission sequence"""
        self.mission_active = True
        self.current_mission = 1
        self.mission_start_time = time.time()
        self.get_logger().info('🚨 STARTING MISSION SEQUENCE 🚨')
        
        # Start Mission 1 after 5 seconds
        threading.Timer(5.0, self.execute_mission_1).start()

    def check_readiness(self):
        """Check if perception node is ready"""
        if self.detections_received >= 3 and not self.perception_ready:
            self.perception_ready = True
            self.get_logger().info(' Perception node is ready! Starting mission sequence...')
            self.readiness_timer.cancel()  # Stop checking
            self.start_mission_sequence()
        elif self.detections_received < 3:
            self.get_logger().info(f'Waiting for perception... ({self.detections_received}/3 detections received)')

    def detections_callback(self, msg):
        """Monitor person detections"""
        self.detections_received += 1
        self.person_detected = False
        for detection in msg.detections:
            if detection.class_name == 'person':
                self.person_detected = True
                break

    def execute_mission_1(self):
        """Mission 1: Kid gets kidnapped - trigger panic"""
        if self.current_mission != 1:
            return
            
        self.get_logger().info('MISSION 1: KIDNAPPING DETECTED')
        
        # Trigger panic event
        panic_event = UserEvent()
        panic_event.header.stamp = self.get_clock().now().to_msg()
        panic_event.event_type = 'panic'
        panic_event.data = 'KIDNAPPING_DETECTED'
        self.user_event_pub.publish(panic_event)
        
        # Announce emergency
        tts_msg = NavigationInstruction()
        tts_msg.header.stamp = self.get_clock().now().to_msg()
        tts_msg.instruction = "EMERGENCY ALERT! Kidnapping detected! Initiating emergency protocol!"
        tts_msg.priority = 3  # Emergency priority
        self.tts_pub.publish(tts_msg)
        
        self.get_logger().info('Mission 1 executed - Panic mode activated')
        
        # Schedule Mission 2 after 15 seconds
        threading.Timer(15.0, self.execute_mission_2).start()

    def execute_mission_2(self):
        """Mission 2: User asks about coffee shop"""
        self.current_mission = 2
        self.get_logger().info(' MISSION 2: COFFEE SHOP')
        
        # Step 1: Simulate user STT query
        self.simulate_user_stt_query()
        
        # Step 2: Move drone to head level (immediate response)
        threading.Timer(1.0, self.move_to_head_level).start()
        
        # Step 3: Process query and provide answer
        threading.Timer(4.0, self.process_stt_and_respond).start()

    def simulate_user_stt_query(self):
        """Simulate user speaking and STT processing"""
        self.get_logger().info('User speaks: "Is there a coffee shop nearby?"')
        
        # Announce STT processing
        tts_msg = NavigationInstruction()
        tts_msg.header.stamp = self.get_clock().now().to_msg()
        tts_msg.instruction = "I heard you ask about a coffee shop. Let me check my sensors..."
        tts_msg.priority = 2
        self.tts_pub.publish(tts_msg)

    def process_stt_and_respond(self):
        """Process the STT query and provide detailed response"""
        self.get_logger().info('Processing STT query and scanning environment...')
        
        # Simulate processing time
        tts_msg = NavigationInstruction()
        tts_msg.header.stamp = self.get_clock().now().to_msg()
        tts_msg.instruction = "Scanning environment for coffee shop landmarks..."
        tts_msg.priority = 2
        self.tts_pub.publish(tts_msg)
        
        # Provide detailed answer after 2 seconds
        threading.Timer(2.0, self.provide_coffee_shop_answer).start()

    def provide_coffee_shop_answer(self):
        """Provide audio answer about coffee shop"""
        self.get_logger().info('Coffee shop detected in environment!')
        
        tts_msg = NavigationInstruction()
        tts_msg.header.stamp = self.get_clock().now().to_msg()
        tts_msg.instruction = "Yes! I have located a coffee shop landmark at coordinate:(X,y,z) It's approximately 50 meters southeast of our current position. The coffee shop appears to be open and accessible. Would you like me to guide you there safely?"
        tts_msg.priority = 2
        self.tts_pub.publish(tts_msg)
        
        # Follow-up positioning announcement
        threading.Timer(6.0, self.announce_positioning_complete).start()
        
        self.get_logger().info('Mission 2 completed - STT query processed and answered')

    def announce_positioning_complete(self):
        """Announce that drone is positioned and ready"""
        tts_msg = NavigationInstruction()
        tts_msg.header.stamp = self.get_clock().now().to_msg()
        tts_msg.instruction = "I am now positioned at your head level for optimal communication. I'm ready to assist with navigation or answer any other questions."
        tts_msg.priority = 1
        self.tts_pub.publish(tts_msg)

    def move_to_head_level(self):
        """Move drone to head level near person"""
        if not self.person_detected:
            self.get_logger().warn('No person detected for head-level positioning')
            return
            
        self.get_logger().info('Moving drone to head level near person')
        
        # Move drone to head level (1.7m height) and closer to person
        cmd = Twist()
        
        # Move forward towards person (assuming person is in front)
        cmd.linear.x = 0.5  # Move forward
        cmd.linear.z = 0.2  # Move up to head level
        
        # Publish movement command for 3 seconds
        for i in range(30):  # 3 seconds at 10Hz
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.1)
        
        # Stop movement
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        # Announce positioning
        tts_msg = NavigationInstruction()
        tts_msg.header.stamp = self.get_clock().now().to_msg()
        tts_msg.instruction = "I am now positioned at your head level for better communication."
        tts_msg.priority = 1
        self.tts_pub.publish(tts_msg)
        
        self.get_logger().info('Drone positioned at head level')

    def mission_loop(self):
        """Main mission loop - monitors mission progress"""
        if not self.mission_active:
            if not self.perception_ready:
                return  # Still waiting for perception
            return
            
        current_time = time.time()
        elapsed = current_time - self.mission_start_time if self.mission_start_time else 0
        
        # Log mission status every 15 seconds
        if int(elapsed) % 15 == 0 and elapsed > 0:
            mission_names = {0: "Waiting", 1: "Kidnapping Response", 2: "Coffee Shop Query"}
            current_name = mission_names.get(self.current_mission, "Unknown")
            self.get_logger().info(f'Mission Status - {current_name} | Elapsed: {elapsed:.0f}s | Person: {self.person_detected} | Detections: {self.detections_received}')
        
        # Auto-complete missions after timeout (safety)
        if self.current_mission == 1 and elapsed > 45:
            self.get_logger().info('Mission 1 timeout - proceeding to Mission 2')
            self.execute_mission_2()
        elif self.current_mission == 2 and elapsed > 90:
            self.get_logger().info('🏁 Mission sequence completed successfully')
            self.mission_active = False


def main(args=None):
    rclpy.init(args=args)
    node = MissionControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

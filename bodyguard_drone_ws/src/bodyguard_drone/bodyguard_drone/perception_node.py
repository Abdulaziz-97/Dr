#!/usr/bin/env python3
"""
Perception Node - YOLO and DINOv3 object detection and feature extraction
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from bodyguard_drone.msg import Detection, DetectionArray, DINOv3Features

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("Warning: ultralytics not available, YOLO detection disabled")

try:
    import torch
    from transformers import AutoImageProcessor, AutoModel
    DINOV3_AVAILABLE = True
except ImportError:
    DINOV3_AVAILABLE = False
    print("Warning: transformers/torch not available, DINOv3 disabled")


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        self.bridge = CvBridge()
        
        # Initialize YOLO model
        if YOLO_AVAILABLE:
            self.get_logger().info('Loading YOLO model (CPU mode)...')
            self.yolo_model = YOLO('yolov8n.pt')  # Nano model for CPU
            self.yolo_model.to('cpu')
        else:
            self.yolo_model = None
            
        # Initialize DINOv3 model
        if DINOV3_AVAILABLE:
            self.get_logger().info('Loading DINOv3 model (CPU mode)...')
            self.dinov3_processor = AutoImageProcessor.from_pretrained('facebook/dinov2-small')
            self.dinov3_model = AutoModel.from_pretrained('facebook/dinov2-small')
            self.dinov3_model.eval()
        else:
            self.dinov3_processor = None
            self.dinov3_model = None
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/drone/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.detections_pub = self.create_publisher(DetectionArray, '/detections', 10)
        self.features_pub = self.create_publisher(DINOv3Features, '/dinov3_features', 10)
        
        # Processing rate limiter (CPU is slow)
        self.frame_count = 0
        self.process_every_n_frames = 30  # Process every 30th frame
        
        self.get_logger().info('Perception node initialized')

    def image_callback(self, msg):
        """Process incoming camera images"""
        self.frame_count += 1
        
        # Skip frames to reduce CPU load
        if self.frame_count % self.process_every_n_frames != 0:
            return
            
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO detection
            if self.yolo_model is not None:
                self.detect_objects(cv_image, msg.header)
            
            # Run DINOv3 feature extraction
            if self.dinov3_model is not None:
                self.extract_features(cv_image, msg.header)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def detect_objects(self, image, header):
        """Detect objects using YOLO"""
        try:
            results = self.yolo_model(image, verbose=False)
            
            detection_array = DetectionArray()
            detection_array.header = header
            
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    detection = Detection()
                    detection.header = header
                    
                    # Get bounding box
                    xyxy = box.xyxy[0].cpu().numpy()
                    detection.x_min = float(xyxy[0])
                    detection.y_min = float(xyxy[1])
                    detection.x_max = float(xyxy[2])
                    detection.y_max = float(xyxy[3])
                    
                    # Get class and confidence
                    detection.class_name = result.names[int(box.cls)]
                    detection.confidence = float(box.conf)
                    
                    # Estimate distance (simplified)
                    detection.distance = self.estimate_distance(xyxy)
                    
                    detection_array.detections.append(detection)
            
            self.detections_pub.publish(detection_array)
            self.get_logger().info(f'Published {len(detection_array.detections)} detections')
            
        except Exception as e:
            self.get_logger().error(f'YOLO detection error: {str(e)}')

    def extract_features(self, image, header):
        """Extract features using DINOv3"""
        try:
            # Preprocess image
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            inputs = self.dinov3_processor(images=rgb_image, return_tensors="pt")
            
            # Extract features
            with torch.no_grad():
                outputs = self.dinov3_model(**inputs)
                features = outputs.last_hidden_state
            
            # Create message
            features_msg = DINOv3Features()
            features_msg.header = header
            features_msg.height = features.shape[1]
            features_msg.width = 1
            features_msg.channels = features.shape[2]
            features_msg.features = features.flatten().cpu().numpy().tolist()
            
            self.features_pub.publish(features_msg)
            self.get_logger().info('Published DINOv3 features')
            
        except Exception as e:
            self.get_logger().error(f'DINOv3 feature extraction error: {str(e)}')

    def estimate_distance(self, bbox):
        """Simple distance estimation based on bounding box size"""
        height = bbox[3] - bbox[1]
        # Rough approximation: larger objects are closer
        if height > 0:
            return 10.0 / height  # Simplified formula
        return 0.0


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

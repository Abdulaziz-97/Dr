#!/usr/bin/env python3
"""
Test YOLO detection manually to debug issues
"""

import cv2
from ultralytics import YOLO
import numpy as np

def test_yolo():
    print("Loading YOLOv11 model...")
    try:
        model = YOLO('yolo11n.pt')
        print("✅ Successfully loaded YOLOv11 nano model")
    except Exception as e:
        print(f"⚠️  YOLOv11 not available ({e}), falling back to YOLOv8")
        model = YOLO('yolov8n.pt')
    
    print(f"YOLO classes: {list(model.names.values())}")
    print(f"Person class ID: {list(model.names.keys())[list(model.names.values()).index('person')]}")
    
    # Test with a simple test image
    print("\nCreating test image with person-like shape...")
    
    # Create a simple test image (640x480, same as camera)
    test_img = np.zeros((480, 640, 3), dtype=np.uint8)
    
    # Draw a simple person-like shape
    cv2.rectangle(test_img, (250, 150), (390, 450), (100, 100, 100), -1)  # Body
    cv2.circle(test_img, (320, 120), 30, (150, 150, 150), -1)  # Head
    
    # Save test image
    cv2.imwrite('/tmp/test_person.jpg', test_img)
    print("Saved test image to /tmp/test_person.jpg")
    
    # Test YOLO on this image
    print("\nTesting YOLO on synthetic image...")
    results = model(test_img, conf=0.01, verbose=True)  # Very low confidence
    
    for result in results:
        boxes = result.boxes
        if boxes is not None:
            print(f"Found {len(boxes)} detections:")
            for box in boxes:
                class_id = int(box.cls)
                class_name = result.names[class_id]
                confidence = float(box.conf)
                print(f"  - {class_name}: {confidence:.3f}")
        else:
            print("No detections found")
    
    # Test on a real camera image if available
    try:
        print("\nTesting on saved camera image...")
        import glob
        camera_images = glob.glob("left*.jpg")
        if camera_images:
            latest_img = max(camera_images)
            print(f"Testing on {latest_img}")
            
            img = cv2.imread(latest_img)
            if img is not None:
                print(f"Image shape: {img.shape}")
                results = model(img, conf=0.01, verbose=True)
                
                for result in results:
                    boxes = result.boxes
                    if boxes is not None:
                        print(f"Found {len(boxes)} detections in camera image:")
                        for box in boxes:
                            class_id = int(box.cls)
                            class_name = result.names[class_id]
                            confidence = float(box.conf)
                            xyxy = box.xyxy[0].cpu().numpy()
                            print(f"  - {class_name}: {confidence:.3f} at [{xyxy[0]:.0f},{xyxy[1]:.0f},{xyxy[2]:.0f},{xyxy[3]:.0f}]")
                    else:
                        print("No detections in camera image")
            else:
                print("Could not load camera image")
        else:
            print("No camera images found. Run image_saver first.")
    except Exception as e:
        print(f"Error testing camera image: {e}")

if __name__ == "__main__":
    test_yolo()

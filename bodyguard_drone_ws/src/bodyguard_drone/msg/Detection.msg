# Detection message for YOLO object detection
std_msgs/Header header

# Bounding box coordinates (normalized 0-1 or pixel coordinates)
float32 x_min
float32 y_min
float32 x_max
float32 y_max

# Detection class and confidence
string class_name
float32 confidence

# Optional: distance estimate from camera
float32 distance

# User event message from ring sensor or other input
std_msgs/Header header

# Event types: "panic", "fall", "heartbeat", "normal"
string event_type

# Additional data (e.g., heart rate value)
string data

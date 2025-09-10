# Message for representing a 2D scalar field
std_msgs/Header header  # Standard ROS header with timestamp and frame_id
nav_msgs/MapMetaData info  # Map metadata: resolution, width, height, origin, etc.
float32[] data  # Scalar field data stored in row-major order
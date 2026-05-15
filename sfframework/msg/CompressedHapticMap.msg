# CompressedHapticMap.msg
# The complete, ultra-lightweight 2.5D topological map for a single frame.

# Standard ROS2 header (contains timestamp and frame_id, e.g., "base_link")
std_msgs/Header header

# Array of simplified polygon vertices
# Each vertex also stores the state/variance of the edge that starts there
StochasticVertex[] vertices

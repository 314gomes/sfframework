# StochasticVertex.msg
# Represents a single 2D vertex of the compressed polygon

# Chosen coordinates of each vertex - usually the centroid
float32 x
float32 y

# Constants for the semantic state of the edge starting at this vertex
uint8 STATE_OBSTACLE=0     # A physical barrier (apply haptic force)
uint8 STATE_FREE_SPACE=1   # Open space, sensor reached max range (no force)
uint8 STATE_OCCLUDED=2     # Blind spot or missing data (optional: apply "spongy" warning force)

# The state flag for the edge from this vertex to the next
uint8 state

# The variance of the orthogonal distances of the culled points
# relative to the simplified edge starting at this vertex.
# Set to 0.0 if state is FREE_SPACE.
float32 perpendicular_variance

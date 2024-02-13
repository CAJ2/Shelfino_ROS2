std_msgs/Header header

# Array of points to represent nodes
geometry_msgs/Point[] nodes
# Same length as nodes, with each edge specifying an array of connected nodes
RoadmapEdge[] edges
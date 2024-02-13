std_msgs/Header header

Roadmap roadmap

# Name of the node/service that created the roadmap
string generator
# Roadmap generation time in milliseconds
uint64 roadmap_duration

# The starting position of the robot
geometry_msgs/PoseWithCovarianceStamped robot_pose

# Position of the gate
geometry_msgs/Pose gate

# The obstacle info
obstacles_msgs/ObstacleArrayMsg obstacles

# The victims info
obstacles_msgs/ObstacleArrayMsg victims

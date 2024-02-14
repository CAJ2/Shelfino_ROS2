std_msgs/Header header

Roadmap roadmap

# Name of the node/service that created the roadmap
string generator
# Roadmap generation time in milliseconds
uint64 roadmap_duration

# Name of the node/service that performed path planning
string path_planner
# Path planning time in milliseconds
uint64 path_planning_duration

uint64 dubins_duration

# The starting position of the robot
geometry_msgs/PoseWithCovarianceStamped robot_pose

# Position of the gate
geometry_msgs/Pose gate

# The obstacle info
obstacles_msgs/ObstacleArrayMsg obstacles

# The victims info
obstacles_msgs/ObstacleArrayMsg victims

# A planned path without headings
Point2D[] graph_path_points

# A planned path with headings
nav_msgs/Path graph_path

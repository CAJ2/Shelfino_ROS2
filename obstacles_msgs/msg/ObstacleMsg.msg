std_msgs/Header header

# Obstacle footprint (polygon descriptions)
geometry_msgs/Polygon polygon

# Specify the radius for circular/point obstacles
float64 radius
# Location
float64 x
float64 y
# Dimensions
float64 dx
float64 dy

# Either CYLINDER or BOX
string type

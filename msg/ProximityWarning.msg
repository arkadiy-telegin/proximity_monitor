# ProximityWarning.msg

# Flag indicating if an object is closer than the threshold
bool warning

# Proximity (0 means the object is right next to the camera, 1 means the object is at the threshold)
float32 proximity

# Camera side (left or right)
bool side
bool left_side=false
bool right_side=true
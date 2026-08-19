# Line-of-sight measurement to the tracked target, fed into the IBVS MPC as the
# bearing quaternion f1 and range r1. Produced by the Gazebo ground-truth
# bounding-box + range plugin (gz_bbox_truth).

std_msgs/Header header

# Bearing (line-of-sight) as a unit quaternion rotating the camera boresight
# onto the target direction (model state f1).
geometry_msgs/Quaternion bearing

# Range r1 from the camera to the target [m] (ground-truth from sim).
float64 range

# Whether this measurement is valid (target detected and in range).
bool valid

# header time aligns with the time of the received image
Header header

# focus are points used for visual servoing
vision_msgs/VisualFeature feature_meas

# pose is the relative pose in the current body frame
vision_msgs/StateEstimate feature_pose

# gate is a 3D planar object in the current body frame
vision_msgs/GateFeature gate

# trajectory is a 3D curve that starts with the origin and ends with pose
vision_msgs/Trajectory reference_trajectory

vision_msgs/AutopilotConfig autopilot_config

vision_msgs/TrackingFeedback tracking_feedback

bool replan_triggered
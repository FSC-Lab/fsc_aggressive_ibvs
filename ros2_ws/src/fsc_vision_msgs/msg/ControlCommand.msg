# Quadrotor control command (ROS2). Output of mpc_controller_node; consumed by
# px4_offboard_bridge. Focused on the VELOCITY-body + 1-axis pitch-gimbal
# variant, but the legacy mode enums are kept for interoperability.

# control mode enums
uint8 NONE=0
uint8 ATTITUDE=1
uint8 BODY_RATES=2
uint8 ANGULAR_ACCELERATIONS=3
uint8 ROTOR_THRUSTS=4
uint8 VELOCITY=5

std_msgs/Header header

# Control mode as defined above
uint8 control_mode

# Flag whether controller is allowed to arm
bool armed

# Time at which this command should be executed
builtin_interfaces/Time expected_execution_time

# --- VELOCITY control mode (velocity-controlled body + gimbal variant) ---
# Commanded world-frame (ENU) linear velocity [m/s]
geometry_msgs/Vector3 velocity
# Commanded yaw rate [rad/s] (pans the camera)
float64 yaw_rate
# Commanded 1-axis gimbal pitch (tilt) angle [rad]
float64 gimbal_angle

# MPC visibility / time-to-collision slack variables (diagnostics)
float64[] slack_variables

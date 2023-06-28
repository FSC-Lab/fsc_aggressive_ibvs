Header header

# Autopilot status
uint8 UNINIT=0
uint8 BOOT=1
uint8 CALIBRATING=2
uint8 STANDBY=3
uint8 ACTIVE=4
uint8 CRITICAL=5
uint8 EMERGENCY=6
uint8 POWEROFF=7
uint8 FLIGHT_TERMINATION=8

uint8 PX4_BODYRATES=0
uint8 PX4_POSITION=1
uint8 FSC_POSITION=2

uint8 POSITION_TRACK=0
uint8 TRAJECTORY_TRACK=1
uint8 VELOCITY_TRACK=2
uint8 FEATURE_TRACK=3
uint8 FEATURE_REACH=4
uint8 HOME=5
uint8 LAND=6

uint8 autopilot_state
uint8 control_mode
uint8 flight_mode

# State estimate used to compute the control command
vision_msgs/StateEstimate state_estimate

# Desired state used to compute the control command
vision_msgs/TrajectoryPoint reference_state

# Last control command
vision_msgs/ControlCommand control_command

# Control command delay
duration control_command_delay

# Controller computation time [s]
duration control_computation_time

# MPC solver status
bool mpc_success




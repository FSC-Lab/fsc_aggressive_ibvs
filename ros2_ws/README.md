# ROS2 (Jazzy) port — velocity-body + pitch-gimbal IBVS NMPC on PX4 SITL

This workspace ports the perception-aware IBVS NMPC to **ROS2 Jazzy**, running
against **PX4 SITL + Gazebo (gz Harmonic)** instead of the RotorS simulator.
The numerical MPC library (ACADO-generated solver + qpOASES + the wrapper/
controller math) is kept **unchanged in behavior** — it is only decoupled from
ROS so it can be wrapped by a thin rclcpp node.

Target platform: **ROS2 Jazzy**, **gz Harmonic**, **PX4 SITL** with the
**uXRCE-DDS** bridge (`px4_msgs` / `px4_ros_com`). Bounding-box detection and
target range are taken from **Gazebo ground truth** (plugins), not a learned
detector.

## Design

The control loop is:

```
PX4 SITL (gz) ──odometry──▶ px4_offboard_bridge ──StateEstimate──▶ mpc_controller_node
   ▲                                                                      │
   │  TrajectorySetpoint (velocity + yawspeed), OffboardControlMode        │ ControlCommand
   └──────────────────────────── px4_offboard_bridge ◀────────────────────┘
                                          │ gimbal_angle
                                          ▼
                              gz gimbal joint (ros_gz_bridge Float64)

gz ground-truth bbox plugin ──▶ bbox + range ──▶ mpc_controller_node (bearing f1, range r1)
```

The VELOCITY command maps naturally onto PX4 offboard
`TrajectorySetpoint` (world velocity + yawspeed); the gimbal angle drives a
revolute joint in Gazebo. ENU/FLU (ROS) ↔ NED/FRD (PX4) conversion is isolated
inside `px4_offboard_bridge`.

## Packages (staged)

| Stage | Package | Status |
|-------|---------|--------|
| 1 | `mpc_core` — ROS-free MPC library (wrapper + controller + params) | **done** (needs ACADO model, see below) |
| 2 | `fsc_vision_msgs` — ROS2 msgs (`ControlCommand`, `TargetBearing`) | **done** |
| 3 | `mpc_controller_node` — rclcpp node wrapping `mpc_core` | **done** |
| 4 | `px4_offboard_bridge` — VELOCITY→PX4 offboard, arm state machine, frames | todo |
| 5 | `gz_gimbal` + `gz_bbox_truth` — gimbal joint bridge, ground-truth bbox/range | todo |
| 6 | `fsc_bringup` — launch + params for SITL + DDS agent + bridge + node | todo |

Stages 1–3 have been compile- and run-verified end-to-end against a **stub**
ACADO solver (correct dimensions, no-op numerics) since ACADO cannot be run in
this environment; swap in the real generated model to get real control output.

## `mpc_core` and the ACADO model

`mpc_core` is ROS-free (Eigen only). It compiles only against the
**regenerated** velocity+gimbal ACADO model
(`ACADO_NX==15, NU==7, NY==21, NYN==14, NOD==23`). That generated code is not
checked in — drop it into `src/mpc_core/model/pose_track_mpc_gimbal/` (see
`src/mpc_core/model/README.md`). Until then the build stops with an explanatory
error. qpOASES is vendored in `src/mpc_core/externals_qpoases/`.

The ROS→ROS-free changes vs. the ROS1 sources:
- `ros/ros.h` logging → `mpc_core/logging.h` (printf shim).
- `MpcParams::loadParameters(ros::NodeHandle)` → `MpcParams::configure(MpcParamsConfig)`
  (plain struct; the node fills it from ROS2 params).
- `vision_common::{StateEstimate,Trajectory,TrajectoryPoint,ControlCommand,
  PolynomialTrajectory}` → POD equivalents in `mpc_core/types.h`.
- `ros::Time` → plain `double` seconds; the node re-stamps ROS messages.

## Build

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
```

Two environment notes on this machine:

- **Python:** a conda Python is first on `PATH` and lacks `catkin_pkg`, which
  breaks the ament/rosidl generators. Force the system interpreter:
  `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3` (or `conda deactivate`
  before building).
- **ACADO model:** `mpc_core` needs the generated solver in
  `src/mpc_core/model/pose_track_mpc_gimbal/` (see `src/mpc_core/model/README.md`).
  Until then, point the build at a directory with the ACADO export via
  `--cmake-args -DMPC_MODEL_DIR=<path>`.

Example (system Python + explicit model dir):

```bash
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
                          -DMPC_MODEL_DIR=/abs/path/to/pose_track_mpc_gimbal
```

# fsc_bringup — full-stack launch for IBVS NMPC on PX4 SITL + gz

`ibvs_sitl.launch.py` composes the ROS2 side of the stack and, optionally, the
uXRCE-DDS agent and the ros_gz_bridge. PX4 SITL and the gz server are started
separately (they live in the PX4-Autopilot tree).

## Dataflow

```
gz (target GT pose, gimbal state)  ──ros_gz_bridge──▶ /target/pose_gt, /gimbal/angle_state
                                                            │
PX4 SITL ──uXRCE-DDS──▶ /fmu/out/*  ──px4_offboard_bridge──▶ /px4/odometry
                                                            │
/px4/odometry + /target/pose_gt + /gimbal/angle_state ─▶ target_bearing_node
                                                            │  /target/bearing, /target/point
                                                            ▼
                                                    mpc_controller_node
                                                            │  /mpc/control_command
                                                            ▼
                                    px4_offboard_bridge ──▶ /fmu/in/* (offboard, arm)
                                                            └─▶ /gimbal/angle_cmd ──gz bridge──▶ gz gimbal joint
```

## Bring-up order

1. **PX4 SITL + gz** (from PX4-Autopilot), with the gimbal + target spawned
   (see `fsc_perception/gz/README.md`):
   ```bash
   make px4_sitl gz_x500        # then spawn gimbal_camera.sdf + moving_target.sdf
   ```
2. **This launch** (starts the DDS agent, gz bridge, and the three nodes):
   ```bash
   ros2 launch fsc_bringup ibvs_sitl.launch.py
   ```
   Toggle sub-components: `dds_agent:=false`, `gz_bridge:=false`,
   `dds_port:=8888`.

## Prerequisites

- `px4_msgs` built in this workspace (matching your PX4 firmware) — needed by
  `px4_offboard_bridge`. See `../README.md`.
- The regenerated ACADO gimbal model in
  `src/mpc_core/model/pose_track_mpc_gimbal/` — needed by `mpc_core`.
- `MicroXRCEAgent` on `PATH` (Micro-XRCE-DDS-Agent) and `ros_gz_bridge`
  installed for gz Harmonic.

## Shared parameters

Camera mount (`t_B_C`, `q_B_M`) must be identical in `mpc_controller_node` and
`target_bearing_node`, and `target_position` should match the target's spawn
pose. See `config/ibvs_common.yaml`.

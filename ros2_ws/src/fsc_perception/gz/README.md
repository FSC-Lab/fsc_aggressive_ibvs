# gz (Harmonic) assets — gimbal + ground-truth target

These are the Gazebo-side pieces for Stage 5. They are **assets + a bridge
config**, not built code (gz plugins here are stock gz-sim systems referenced by
name). The only compiled Stage-5 code is `target_bearing_node`.

## Files

- `gimbal_camera.sdf` — a one-axis **pitch** gimbal (`gimbal_tilt_joint`, axis
  y, travel −90°…+30°) carrying a camera. Driven by a `JointPositionController`
  on a position command topic; `JointStatePublisher` publishes the angle back.
  `t_B_C = (0,0,0.15)` matches the MPC params.
- `moving_target.sdf` — the tracked target (bright box) with a `PosePublisher`
  that gives the **ground-truth** world pose (→ bbox + range, no detector).
- `ros_gz_bridge.yaml` — maps the gz topics to the ROS topics the ROS2 stack
  uses.

## Ground-truth "bounding box" / range

The user chose **ground-truth bbox + ground-truth range** (no learned detector).
With the target's true pose and the drone odometry, `target_bearing_node`
computes the exact line-of-sight bearing `f1` and range `r1` the model expects —
so no image, PnP, or detector network is required for control. The camera sensor
in `gimbal_camera.sdf` is retained only for visualization or a future real
detector. (If you later want an actual 2D bbox from the ground-truth pose,
project the target's 8 corners with the camera intrinsics — the geometry mirrors
`aprilfake`'s `projToImg`.)

## Wiring it up

1. Attach the gimbal to the PX4 x500. Either `<include>` `gimbal_camera.sdf`
   into the x500 model and add a fixed joint `base_link → gimbal_base`, or spawn
   it and weld it. Keep the model name `x500_gimbal` (or update the gz topic
   names in `ros_gz_bridge.yaml`).
2. Spawn `moving_target.sdf` into the world.
3. Start the bridge:
   ```bash
   ros2 run ros_gz_bridge parameter_bridge \
     --ros-args -p config_file:=<install>/share/fsc_perception/gz/ros_gz_bridge.yaml
   ```

## Topic map

| ROS topic            | gz topic                                                   | dir |
|----------------------|------------------------------------------------------------|-----|
| `/gimbal/angle_cmd`  | `/model/x500_gimbal/joint/gimbal_tilt_joint/cmd_pos`       | →gz |
| `/gimbal/angle_state`| `/model/x500_gimbal/joint/gimbal_tilt_joint/state`         | →ros|
| `/target/pose_gt`    | `/model/moving_target/pose`                                | →ros|
| `/clock`             | `/clock`                                                    | →ros|

`/gimbal/angle_cmd` is published by `px4_offboard_bridge`; `/gimbal/angle_state`
feeds both `mpc_controller_node` (gimbal feedback) and `target_bearing_node`.

Note: the exact gz JointStatePublisher output topic/type may need a small relay
to expose the single joint angle as `std_msgs/Float64`; adjust the bridge entry
to your gz version if the joint state is published as `gz.msgs.Model`/JointState
rather than a scalar.

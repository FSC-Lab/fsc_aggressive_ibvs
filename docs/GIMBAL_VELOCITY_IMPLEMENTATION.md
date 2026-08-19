# Velocity-Body + 1-Axis Pitch-Gimbal IBVS — Implementation Notes

This documents the variant that replaces the original CTBR (collective-thrust +
body-rate) controller with a **velocity-controlled body** plus a **1-axis pitch
(tilt) gimbal**. Camera **pan** is done with the body **yaw**; camera **tilt**
is done with the **gimbal**.

See `docs/METHOD_SUMMARY.md` for the original method. This doc only covers the
delta.

---

## 1. What changed conceptually

| | Original (CTBR) | This variant (velocity + gimbal) |
|---|---|---|
| Body control | collective thrust + body rates | commanded **world velocity** (first-order tracked) |
| Heading | part of full attitude quaternion | scalar **yaw state** `psi`, driven by `psi_rate` (camera **pan**) |
| Camera pointing | fixed on body | body yaw (pan) + **1-axis pitch gimbal** `theta_g` (tilt) |
| Output command | `BODY_RATES` | `VELOCITY` (world velocity + yaw rate + gimbal angle) |
| Bearing/range IBVS dynamics | — | **unchanged** (frame-agnostic; only `vc`/`wc`/`p_est` recomputed) |

The camera orientation used inside the model is
`R_WC = Rz(psi) · R_B_M · R_pitch(theta_g)`, where `R_B_M` is the fixed
body→gimbal-base mount rotation (online data `q_B_M`).

---

## 2. Files changed

### New ACADO model
- `utils/pose_track_mpc_gimbal.cpp` — the OCP for this variant.
  - **States (15):** `p(3), psi, v(3), f1(4), r1, theta_g, dy1, dy2`
  - **Controls (7):** `vx_cmd, vy_cmd, vz_cmd, psi_rate, omega_g, sv1, sv2`
  - **Online data (23):** `t_B_C(3), q_B_M(4), focus_p(3), poly ax..dz(12), r_start(1)`
  - **Cost `h` (21):** state-cost part (14) + input-cost part (7)
  - **End cost `hN` (14):** the state-cost part only
  - Exports to `pose_track_mpc_gimbal`.

### Message / command type
- `fsc_vision_common/vision_msgs/msg/ControlCommand.msg` — added `VELOCITY=5`
  mode and `velocity`, `yaw_rate`, `gimbal_angle` fields.
- `fsc_vision_common/vision_common/include/vision_common/control_command.h` —
  added `VELOCITY` to `ControlMode` + the three fields.
- `fsc_vision_common/vision_common/src/control_command.cpp` — handled the new
  mode/fields in the default ctor, msg ctor, `zero()`, and `toRosMessage()`.

### MPC bridge
- `fsc_autopilot/include/mpc/mpc_wrapper.h` — new `StateIndex` /
  `ReferenceIndex` / `ControlIndex` / `OnlineDataIndex` layouts; input-bound maps
  sized to `kInputSize`; new default `W_` (21-diag); 7-element `kHoverInput_`;
  new `setLimits` / `setCameraParameters` / `setFocusParameters` signatures;
  removed `setGateParameters` / `setSizeParameters`.
- `fsc_autopilot/src/mpc/mpc_wrapper.cpp` — new hover state/ref; velocity/yaw/
  gimbal input bounds; mount rotation written to `MOUNT_R`; focus writes
  `FOC_P` only; removed the quaternion-hemisphere flip in `update()`; fixed the
  sv-scaling indices for the 7-input layout.
- `fsc_autopilot/include/mpc/mpc_controller.h` — `static_assert` now 15/7; added
  `setGimbalAngle()` and `gimbal_angle_` member.
- `fsc_autopilot/src/mpc/mpc_controller.cpp`:
  - `setState()` packs `p / psi(from yaw) / v / f1 / r1 / theta_g`.
  - `setReference()` builds the 14-cost / 7-input layout, no quaternion flips.
  - `getControlCommand()` emits a `VELOCITY` command (velocity + yaw_rate +
    gimbal_angle).
  - `getPredictedState()` reconstructs a level-body pose from `psi`.
  - `setNewParams()` calls the new setters (no gate/size).
  - `run()` warm-starts `gimbal_angle_` from the model's own prediction.
- `fsc_autopilot/include/mpc/mpc_params.h` — `Q(14)` / `R(7)` layout; new
  weight setters (`setYawWeight`, `setGimbalWeight`); velocity/yaw-rate/
  gimbal-rate/gimbal-angle limits; mount `q_B_M_`; `focus_p_` only; removed
  thrust/attitude/gate params.

### Parameters
- `fsc_autopilot/parameters/rotors_control_params.yaml` — added
  `Q_yaw, Q_gimbal, R_vel, R_yaw_rate, R_gimbal_rate`,
  `max_velocity, max_yaw_rate, max_gimbal_rate, gimbal_angle_min,
  gimbal_angle_max`, and `q_B_M`. `control_mode` set to `5` (VELOCITY).

---

## 3. Build / regeneration steps (REQUIRED before this compiles)

The MPC bridge is compiled against the **ACADO-generated** solver headers
(`acado_common.h`, etc.) whose dimensions (`ACADO_NX`, `ACADO_NU`, ...) come
from the model. The bridge changes above assume the **new** model. Therefore:

1. Build and run `utils/pose_track_mpc_gimbal.cpp` through ACADO (same toolchain
   used to generate the original `pose_track_mpc`). It exports the solver into
   `pose_track_mpc_gimbal/`.
2. Point the autopilot's CMake/include path at the newly generated directory
   (replace the old `pose_track_mpc` export the wrapper currently includes via
   `acado_common.h` / `acado_auxiliary_functions.h`).
3. `catkin build` (or `catkin_make`) the workspace.

Until step 1–2 are done, the `static_assert(kStateSize == 15 && kInputSize == 7)`
in `mpc_controller.h` will (intentionally) fail against the old generated code —
that assert is the guard that the bridge and the generated model agree.

> Note: this branch was authored without a local ACADO install, so the C++ has
> **not been compiled** here. The dimensions/index maps were derived directly
> from `pose_track_mpc_gimbal.cpp`; re-check them against the generated
> `acado_common.h` after regeneration.

---

## 4. Remaining wiring (NOT done in this branch)

1. **Command transport / simulator interface.** `fsc_rotors_interface` currently
   converts a `BODY_RATES` command into rotor torques/thrust. A `VELOCITY`
   command needs either:
   - an outer velocity+level-attitude loop that feeds the existing rotor
     allocation, or
   - publishing the world-velocity setpoint to a velocity-capable autopilot/
     flight stack.
   Add a `case VELOCITY:` in the interface accordingly.
2. **Gimbal actuation.** Publish `command.gimbal_angle` to the gimbal
   joint/plugin (e.g. a `std_msgs/Float64` joint-position topic). If the gimbal
   has encoder feedback, feed it back via `MpcController::setGimbalAngle()`;
   otherwise the controller warm-starts from its own prediction.
3. **Autopilot emission (`fsc_autopilot_inl.h`).** Confirm the command queue is
   published on the path that reaches the interface/gimbal for VELOCITY mode.
4. **`focus_p` semantics.** `setFocus(global_point_, ...)` still supplies the
   target's estimated world position as the reconstruction anchor — unchanged,
   but re-verify against `aprilfake` / the real detector.

---

## 5. Things to verify after regeneration

- `ACADO_NX==15`, `ACADO_NU==7`, `ACADO_NY==21`, `ACADO_NYN==14`, `ACADO_NOD==23`.
- The `acadoVariables.lbValues` / `ubValues` arrays are `kInputSize`-per-node
  (the bound maps in `mpc_wrapper.h` now assume `kInputSize`, not a literal 6).
- Cost-weight ordering in `W_` / `Q_` matches the `h` output ordering in the
  model exactly.
- Sign/quat conventions for `q_B_M` (mount) match the physical camera.

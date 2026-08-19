# FSC Aggressive IBVS — Method Summary

Perception-Aware Image-Based Visual Servoing (IBVS) of aggressive quadrotor UAVs.

> Qin, Yu, Go, Liu. *"Perception-Aware Image-Based Visual Servoing of Aggressive
> Quadrotor UAVs."* IEEE/ASME Transactions on Mechatronics, 2023.

This document answers: **what is the method, what goes in, what comes out, and
which poses does it actually need?**

---

## 1. One-paragraph description

The controller is a **nonlinear model-predictive controller (NMPC / VPC — "visual
predictive control")**. Unlike a classic position tracker, its internal model
couples the **3D quadrotor rigid-body dynamics** with the **image-plane dynamics
of the tracked feature** (the interaction / feature-Jacobian model of IBVS). It
optimizes over a 1 s horizon (`N = 20` nodes, `dt = 0.05 s`) to drive the
quadrotor toward a target while an explicit **visibility constraint keeps the
target's projection inside the camera image**, even during agile, high-body-rate
maneuvers. The solver is ACADO + qpOASES (Gauss–Newton, multiple shooting),
generated from `utils/pose_track_mpc.cpp`.

So it is a **hybrid** method — *not* purely image-plane:

- The **dynamics/tracking** live in 3D (position, attitude, velocity).
- The **perception objective and constraint** live in the **image plane**
  (normalized image coordinates and a bearing vector).

---

## 2. Algorithm inputs (per control tick, ~solved every cycle)

The MPC `run()` call (`fsc_autopilot/src/mpc/mpc_controller.cpp`) consumes two
things each iteration — a **state estimate** and a **reference** — plus a set of
slowly-varying **online parameters**.

### 2a. State estimate `x₀` (the initial state fed to the OCP)

Set in `MpcController::setState()`. The state vector is **19-dimensional**:

| Group | Symbols | Source | Meaning |
|-------|---------|--------|---------|
| Position | `p_x, p_y, p_z` | drone odometry | quadrotor position in world frame |
| Orientation | `q_w, q_x, q_y, q_z` | drone odometry | quadrotor attitude (world→body quaternion) |
| Velocity | `v_x, v_y, v_z` | drone odometry | quadrotor linear velocity |
| Thrust | `Th` | controller state | current collective thrust |
| **Feature bearing** | `f1_w, f1_x, f1_y, f1_z` | **camera measurement** | quaternion encoding the **line-of-sight / bearing vector** to the target in the camera frame |
| **Feature range** | `r1` | camera + PnP | **distance** from camera to target |
| Slack integrators | `dy1, dy2` | internal | dummy states for the soft constraints |

So the **drone's 6-DOF pose (position + orientation) IS required** — it comes from
the quadrotor's own odometry / state estimator (`odom_topic` →
`vision_common::StateEstimate`).

The **target contribution is NOT a full 6-DOF pose**. Only two things about the
target enter the state:
1. its **bearing** (where it appears in the image — a unit line-of-sight
   direction, stored as the `f1` quaternion / `normal_vector`), and
2. its **range** `r1` (scalar distance).

The target's orientation / full pose is never part of the servo state.

### 2b. Reference / desired behavior

Set in `MpcController::setReference()` from a `vision_common::Trajectory`.
In practice (`goMPC` in `fsc_autopilot_inl.h`) the reference is built from:

- a **desired stand-off distance** to the target (`reference_distance_`, set live
  from the RQT slider / RC), which defines a reference point, and
- a short time-optimal polynomial trajectory from the current pose to that point,
- a desired **distance rate** (closing speed), `r(16) = rdot1 = -2.0` by default.

### 2c. Online parameters (`setNewParams`, updated when they change)

These are `OnlineData` in the ACADO model — constants for the horizon, not
optimized:

- **Camera extrinsics** `t_B_C`, `q_B_C` — body→camera transform.
- **Focus** `focus_p`, `focus_q` — an anchor point; in `goMPC` this is set to the
  target's **estimated global 3D position** (`global_point_`). Used to reconstruct
  the target position from the bearing+range inside the model.
- **Gate** pose `box_p`, `box_q` + `box_width`, `box_height`, `quad_radius` —
  used by an **optional collision-box constraint** (currently commented out in the
  model). Not required for basic target tracking.
- **Distance-parameterized reference polynomial** coefficients `ax…dz`, `r_start`.

---

## 3. Algorithm outputs

`MpcController::getControlCommand()` produces a **`vision_common::ControlCommand`
in `BODY_RATES` mode** — i.e. a **CTBR (collective-thrust + body-rate)** command,
the standard interface for aggressive flight:

| Output | Symbol | Notes |
|--------|--------|-------|
| Collective thrust | `Th` | mass-normalized thrust (integrated from `dTh`) |
| Thrust rate | `dTh` | |
| Body rates | `w_x, w_y, w_z` | roll / pitch / yaw angular rates |
| (attitude) | `q_w…q_z` | predicted orientation, carried in the command |
| Slack values | `sv1, sv2` | diagnostics for the soft constraints |

It also returns the **predicted state trajectory** over the horizon (positions,
attitudes, and the predicted feature bearing/range), used for visualization and
for the state predictor.

The optimization decision variables (control inputs) are:
`dTh` (thrust rate), `w_x, w_y, w_z` (body rates), and two slack variables
`sv1` (visibility), `sv2` (time-to-collision / collision).

---

## 4. What makes it "perception-aware" (the image-plane part)

The feature is carried through the horizon by the **IBVS interaction dynamics**
(`dot(f1_*)`, `dot(r1)` in the model) — the bearing rotates and the range changes
as a function of the camera's linear/angular velocity (`vc`, `wc`). Two
perception mechanisms then act in normalized image coordinates
`(u, v) = (vec1_x/vec1_z, vec1_y/vec1_z)`:

1. **Cost term** — `(u, v)` are penalized toward the image center (`Q_img_u`,
   `Q_img_v`), keeping the target well-framed.
2. **Hard visibility constraint (softened by slack `sv1`)** — the model enforces
   ```
   img_u_min ≤ u ≤ img_u_max
   img_v_min ≤ v ≤ img_v_max      (±0.588836 ≈ camera half-FOV, normalized)
   ```
   This is the "keep the target inside the rectangular image bound" behavior seen
   in the RViz demo. A second slack `sv2` handles a **time-to-collision** bound
   (`ttc1_inv`) and the optional gate collision box.

Because these constraints are expressed in image / bearing coordinates, the method
is genuinely IBVS — but the platform is still stabilized with a full 3D dynamic
model, which is what allows *aggressive* (non-hover) tracking.

---

## 5. Direct answers to the posed questions

- **What is the input?** The quadrotor's own **6-DOF pose + velocity + thrust**
  (from odometry), plus the **target's bearing and range** measured from the
  camera (not its pose), plus a desired stand-off distance and camera extrinsics.
- **What is the output?** A **collective-thrust + body-rate command** (CTBR) at
  each tick, plus the predicted state trajectory.
- **Does it need the drone's 6-DOF pose?** **Yes.** Position and orientation are
  part of the state and come from the drone's state estimator.
- **Does it need the target's 6-DOF pose?** **No.** It needs only the target's
  **line-of-sight bearing** (image projection) and a **scalar range** estimate.
  A target *position* estimate (`focus_p`) is used as a convenience anchor, and a
  target/gate *pose* is only needed if the optional collision-box constraint is
  enabled — which it is not, in the default target-tracking configuration.
- **Does it work entirely in the image plane?** **Not entirely.** It is a hybrid:
  the **visibility objective and constraint are in the image plane**, while the
  **quadrotor dynamics, tracking, and range control are in 3D**. This coupling is
  the core contribution.

---

## 6. Where to look in the code

| Concern | File |
|---------|------|
| OCP / model definition (dynamics, costs, constraints) | `utils/pose_track_mpc.cpp` |
| State ↔ MPC bridging, command extraction | `fsc_autopilot/src/mpc/mpc_controller.cpp` |
| Cost weights, online params, limits | `fsc_autopilot/include/mpc/mpc_params.h` |
| Autopilot loop, reference/distance handling (`goMPC`) | `fsc_autopilot/include/fsc_autopilot/fsc_autopilot_inl.h` |
| Fake feature/bearing generator (sim) | `aprilfake/src/aprilfake_nodelet.cpp` |
| Feature messages | `fsc_vision_common/vision_msgs/msg/{VisualFeature,GateFeature,StateEstimate}.msg` |
</content>
</invoke>

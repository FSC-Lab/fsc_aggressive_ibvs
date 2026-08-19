# Engagement State Machine (target loss + terminal guidance)

A supervisory FSM that sits **above** the perception-aware IBVS MPC and decides
*how* to engage a target when the camera measurement is good, degraded, or lost.

Implementation: `fsc_autopilot/include/fsc_autopilot/engagement_supervisor.h`
(header-only, ROS-free, Eigen only — unit-testable in isolation).

> Why a separate layer? The MPC is pure feedback: each tick it consumes a fresh
> bearing `f1` + range `r1` and has **no memory and no target motion model**. So
> "what happens when the box disappears / is partial / the target overflows the
> frame at close range" cannot be answered inside the MPC. It's answered here.

---

## 1. States

| State | Meaning | Feature source | Closing? |
|-------|---------|----------------|----------|
| `IDLE` | not engaging; hold position | none | no |
| `SEARCH` | no lock yet; sweep yaw + gimbal to acquire | none | no |
| `TRACK` | locked, far field; IBVS approach at stand-off | **measured** | yes |
| `TERMINAL` | locked, near field; committed final approach to contact | **measured** | yes |
| `COAST` | lost/partial; dead-reckon on last LOS, **bounded** | **predicted** | guarded |
| `REACQUIRE` | loss persists; hold + active search around last LOS | none | no |
| `ABORT` | give up; hand back to a safe hover | none | no |

## 2. Transitions

```
IDLE --want_engage--> SEARCH --good lock--> TRACK
TRACK --range < terminal_radius--> TERMINAL
TRACK/TERMINAL --lock lost--> COAST
COAST --good lock--> TRACK/TERMINAL          (reacquired)
COAST --distance_to_go <= stop_distance--> ABORT     [ODOMETRY HARD STOP]
COAST --blind_travel >= budget--> ABORT              [blind-travel budget]
COAST(far) --coast_timeout--> REACQUIRE
REACQUIRE --good lock--> TRACK/TERMINAL
REACQUIRE --reacquire_timeout--> ABORT
any --!want_engage--> IDLE
```

## 3. The anti-"fly to infinity" guarantee

The failure you never want: lose the box, keep commanding closing velocity, and
fly forward forever. This is prevented **without the camera**:

- At each good detection the supervisor latches the target world position and the
  approach direction (drone → target).
- While blind (`COAST`), it computes `distance_to_go` = remaining along-track
  distance to the *last known* target, from odometry alone.
- When `distance_to_go <= stop_distance`, or blind travel exceeds the budget, it
  transitions to `ABORT` and reports `closing_allowed = false`.

`closing_allowed = false` is a hard contract: the autopilot **must** zero the
forward closing velocity that tick. So even with the detector permanently dead,
the drone stops at the last known target plane instead of running away.

## 4. Terminal guidance vs. far-field loss

The two loss regimes behave differently on purpose:

- **Far field (`TRACK` → `COAST`):** loss is unexpected. Coast briefly on the
  predicted feature (`coast_timeout`), then `REACQUIRE` (hold + search). No long
  blind runs.
- **Near field (`TERMINAL` → `COAST`):** loss is *expected* — the target overflows
  the frame and the box goes partial/absent right at contact. Here we **commit
  forward** on the latched LOS for a bounded distance (`terminal_coast_distance`),
  still under the odometry hard stop. This is what lets you finish an engagement
  even though vision drops out in the last meter, while guaranteeing you halt.

Partial boxes are gated out of `LOCK` by default (`partial_box_is_usable=false`),
so a clipped detection near contact triggers the committed coast rather than
yanking the bearing with an unreliable centroid.

## 5. Wiring into the existing autopilot

Everything below refers to `fsc_autopilot/include/fsc_autopilot/fsc_autopilot_inl.h`.

1. **Own an instance** next to the other members in `fsc_autopilot.h`:
   ```cpp
   #include <fsc_autopilot/engagement_supervisor.h>
   ...
   EngagementSupervisor engagement_;
   ```

2. **Feed it every feature tick.** Today `pointCallback()` (line ~414) sets
   `point_`, `global_point_`, `time_last_point_received_`. Build an
   `EngagementInput` there (or in `trackFeature`) from:
   - `stamp`         = `ros::Time::now().toSec()`
   - `detection_valid` = a detection arrived and passed your basic checks
   - `partial_box`   = box touches an image edge / `u,v` near `image_*_bound_`
   - `confidence`    = detector score (add to the point message, or derive)
   - `range`         = `point_.toPoint().norm()`
   - `drone_pos`     = `state_est_.position`
   - `target_pos`    = `global_point_`
   - `want_engage`   = currently in the engage flight mode

3. **Drive `trackFeature()` from the decision.** Replace the unconditional
   `run()` at line ~1591 with a switch on `decision.behavior`:
   - `HOLD` / `SEARCH_SWEEP` → `setFlightMode(POSITION_TRACK)` (SEARCH additionally
     commands a yaw/gimbal sweep) — this is the safe fallback the code already
     does today at watchdog line ~886, just made explicit and stateful.
   - `ENGAGE` → current path: `computeReferencePosition(decision.reference_distance)`
     then `base_controller_.run(...)`. Note `reference_distance` now comes from the
     supervisor (`standoff_distance` in TRACK, `contact_distance` in TERMINAL).
   - `COAST_FORWARD` → run the MPC but feed the **predicted** feature instead of a
     live one (see step 4), and if `!decision.closing_allowed`, zero the forward
     component of the commanded velocity before sending.

4. **Predicted feature during COAST.** `base_controller_.run()` already returns a
   predicted state trajectory (`mpc_states_` / the predicted `f1,r1`). On a blind
   tick, feed node 1's predicted bearing/range back into `state_est_.gates` in
   `getStateForFeatureTrack()` (line ~1702) instead of returning `false` when
   `point_available_` is stale. That is the dead-reckoning.

5. **Replace the crude loss handler** in `watchdogThread()` (lines ~880-893). Right
   now stale point → immediate `POSITION_TRACK`. Instead let the supervisor decide
   (its `COAST`/`REACQUIRE`/`ABORT` subsumes this), and only force `POSITION_TRACK`
   on `ABORT` or on odometry loss.

6. **Optional but recommended:** add an `EngagementState` field to
   `AutopilotFeedback` and publish `toString(decision.state)` + `distance_to_go`
   for logging / RViz.

No new `FlightMode` enum value is strictly required — the supervisor can drive the
existing `FEATURE_TRACK` path. Add one (`FEATURE_ENGAGE`) only if you want to keep
the old behavior available side by side.

## 6. Key tuning knobs (`EngagementConfig`)

| Field | Default | Effect |
|-------|---------|--------|
| `standoff_distance` | 3.0 m | reference distance in `TRACK` |
| `terminal_radius` | 2.0 m | range at which `TRACK` → `TERMINAL` |
| `contact_distance` | 0.0 m | reference distance in `TERMINAL` (set >0 to stop short) |
| `terminal_coast_distance` | 1.5 m | max blind forward travel once committed |
| `coast_travel_budget` | 0.5 m | max blind travel in a far-field coast |
| `stop_distance` | 0.2 m | odometry hard-stop margin |
| `coast_timeout` | 0.4 s | brief-loss window before searching |
| `reacquire_timeout` | 2.0 s | total loss window before aborting |
| `min_confidence` | 0.30 | detection gate |
| `partial_box_is_usable` | false | trust clipped boxes as a lock? |

#pragma once

// ============================================================================
//  EngagementSupervisor
// ----------------------------------------------------------------------------
//  A small, ROS-free finite-state machine that sits ABOVE the perception-aware
//  IBVS MPC and decides HOW the drone should engage a target when the camera
//  measurement is good, degraded, or lost.
//
//  The MPC itself is pure feedback: every tick it consumes a fresh bearing (f1)
//  and range (r1) as part of the state, and it has no memory or motion model.
//  So "what to do when we lose the box / only get a partial box / get too close
//  to keep the target framed" is NOT an MPC concern -- it lives here.
//
//  The single most important guarantee this class provides:
//
//      >> the drone can never "fly to infinity" after losing the target <<
//
//  because during any camera-blind phase the closing motion is bounded by an
//  ODOMETRY-only hard stop (distance-to-go along the last known line of sight)
//  and a blind-travel budget. Neither depends on the camera.
//
//  Design: pure logic. Inputs come in as an EngagementInput (timestamps in
//  seconds, world-frame vectors), outputs go out as an EngagementDecision that
//  tells the autopilot which sub-behavior to run and whether closing is allowed.
//  It has no ROS / ACADO dependencies so it can be unit-tested in isolation.
// ============================================================================

#include <Eigen/Dense>
#include <algorithm>
#include <limits>
#include <string>

namespace fsc {

// ----------------------------------------------------------------------------
//  States
// ----------------------------------------------------------------------------
enum class EngagementState {
  IDLE = 0,       // not engaging; autopilot holds position
  SEARCH = 1,     // no lock yet; sweep camera (yaw pan + gimbal tilt) to acquire
  TRACK = 2,      // valid lock, far field; normal IBVS approach at stand-off
  TERMINAL = 3,   // valid lock, near field; committed final approach to contact
  COAST = 4,      // measurement lost/partial; dead-reckon on last LOS (bounded)
  REACQUIRE = 5,  // loss persists; hold and actively search around last LOS
  ABORT = 6,      // give up engaging; hand control back to a safe hover
};

// Where the bearing/range fed to the MPC should come from this tick.
enum class FeatureSource {
  NONE = 0,       // do not run IBVS this tick (hold / search)
  MEASURED = 1,   // use the live camera measurement
  PREDICTED = 2,  // use the MPC's own predicted feature (dead reckoning)
};

// The concrete sub-behavior the autopilot should execute this tick.
enum class EngagementBehavior {
  HOLD = 0,           // position hold at current pose
  SEARCH_SWEEP = 1,   // position hold + sweep gimbal/yaw to (re)find target
  ENGAGE = 2,         // run IBVS MPC toward target using MEASURED feature
  COAST_FORWARD = 3,  // run IBVS MPC using PREDICTED feature, odometry-guarded
};

// ----------------------------------------------------------------------------
//  Tuning
// ----------------------------------------------------------------------------
struct EngagementConfig {
  // --- timing (seconds) ---
  double coast_timeout = 0.4;      // brief-loss window before we start searching
  double reacquire_timeout = 2.0;  // total loss window before we abort (from last
                                   //   good detection)

  // --- geometry (meters) ---
  double standoff_distance = 3.0;  // reference distance while in TRACK (far field)
  double terminal_radius = 2.0;    // enter TERMINAL when range drops below this
  double contact_distance = 0.0;   // reference distance while in TERMINAL
  double terminal_coast_distance = 1.5;  // max BLIND forward travel once committed
                                         //   in TERMINAL (the "one more meter" rule)
  double coast_travel_budget = 0.5;      // max blind travel in a far-field COAST
  double stop_distance = 0.2;      // odometry hard stop: abort when the remaining
                                   //   along-track distance to the last known
                                   //   target drops to/below this

  // --- gating ---
  double min_confidence = 0.30;    // reject detections weaker than this
  bool require_innovation_gate = true;  // reject detections that jump away from
                                        //   the predicted bearing (outliers)
  bool partial_box_is_usable = false;   // treat a clipped/partial detection as a
                                        //   trustworthy lock? Default: no -> coast
};

// ----------------------------------------------------------------------------
//  Per-tick input
// ----------------------------------------------------------------------------
struct EngagementInput {
  double stamp = 0.0;              // wall time [s] (e.g. ros::Time::now().toSec())
  bool want_engage = false;        // operator/mission requests engagement

  // detection this tick
  bool detection_valid = false;    // a detection arrived this tick
  bool partial_box = false;        // detection is clipped / target overflows FOV
  bool innovation_ok = true;       // measured bearing agrees with prediction
  double confidence = 1.0;         // detector confidence in [0, 1]
  double range = 0.0;              // measured range r1 [m] (valid iff detection)

  // odometry (always available; independent of the camera)
  Eigen::Vector3d drone_pos = Eigen::Vector3d::Zero();   // world
  Eigen::Vector3d target_pos = Eigen::Vector3d::Zero();  // world (valid iff detection)
};

// ----------------------------------------------------------------------------
//  Per-tick output
// ----------------------------------------------------------------------------
struct EngagementDecision {
  EngagementState state = EngagementState::IDLE;
  EngagementBehavior behavior = EngagementBehavior::HOLD;
  FeatureSource feature_source = FeatureSource::NONE;

  bool closing_allowed = false;    // false => autopilot MUST zero forward closing
                                   //   velocity (odometry guard tripped)
  double reference_distance = 0.0; // stand-off to command this tick
  double distance_to_go = 0.0;     // odometry along-track distance to last target
  const char* reason = "";         // human-readable transition reason (for logs)
};

// ----------------------------------------------------------------------------
//  The supervisor
// ----------------------------------------------------------------------------
class EngagementSupervisor {
 public:
  EngagementSupervisor() = default;
  explicit EngagementSupervisor(const EngagementConfig& cfg) : cfg_(cfg) {}

  void setConfig(const EngagementConfig& cfg) { cfg_ = cfg; }
  const EngagementConfig& config() const { return cfg_; }

  EngagementState state() const { return state_; }

  // Force back to IDLE (e.g. operator disengages, or after handling an ABORT).
  void reset() {
    state_ = EngagementState::IDLE;
    have_lock_ = false;
    came_from_terminal_ = false;
  }

  // Advance the machine one tick and return what the autopilot should do.
  EngagementDecision update(const EngagementInput& in) {
    // A detection is a trustworthy LOCK only if it passes every gate. A partial
    // box near contact is the classic false-confidence case, so by default we
    // do NOT treat it as a lock -- we coast on prediction instead.
    const bool good = in.detection_valid && (in.confidence >= cfg_.min_confidence) &&
                      (!cfg_.require_innovation_gate || in.innovation_ok) &&
                      (cfg_.partial_box_is_usable || !in.partial_box);

    if (good) {
      // Latch everything we know so the blind phases have something to steer by.
      last_target_pos_ = in.target_pos;
      last_range_ = in.range;
      const Eigen::Vector3d los = in.target_pos - in.drone_pos;
      if (los.norm() > 1e-3) approach_dir_ = los.normalized();
      t_last_good_ = in.stamp;
      have_lock_ = true;
    }

    EngagementDecision d;
    d.distance_to_go = distanceToGo(in.drone_pos);

    switch (state_) {
      // ---- not engaging -------------------------------------------------
      case EngagementState::IDLE:
        if (in.want_engage) enter(EngagementState::SEARCH, in, "engage requested");
        break;

      // ---- looking for the target --------------------------------------
      case EngagementState::SEARCH:
        if (!in.want_engage)      enter(EngagementState::IDLE, in, "disengaged");
        else if (good)            enter(rangeState(in.range), in, "target acquired");
        break;

      // ---- locked, far field -------------------------------------------
      case EngagementState::TRACK:
        if (!in.want_engage)      enter(EngagementState::IDLE, in, "disengaged");
        else if (!good)           startCoast(in, /*from_terminal=*/false, "lock lost");
        else if (in.range <= cfg_.terminal_radius)
                                  enter(EngagementState::TERMINAL, in, "entered terminal");
        break;

      // ---- locked, near field: committed final approach ----------------
      case EngagementState::TERMINAL:
        if (!in.want_engage)      enter(EngagementState::IDLE, in, "disengaged");
        else if (!good)           startCoast(in, /*from_terminal=*/true, "terminal lock lost");
        // stay in TERMINAL while good; range may fall to contact
        break;

      // ---- lost: dead-reckon on the last line of sight (bounded) -------
      case EngagementState::COAST: {
        const double blind_travel = (in.drone_pos - coast_entry_pos_).norm();
        const double max_blind = came_from_terminal_ ? cfg_.terminal_coast_distance
                                                     : cfg_.coast_travel_budget;
        if (!in.want_engage)                     enter(EngagementState::IDLE, in, "disengaged");
        else if (good)                           enter(rangeState(in.range), in, "reacquired");
        // ---- odometry-only hard stop: this is the anti-"fly to infinity" ----
        else if (d.distance_to_go <= cfg_.stop_distance)
                                                 enter(EngagementState::ABORT, in, "odometry hard stop: reached/passed target");
        else if (blind_travel >= max_blind)      enter(EngagementState::ABORT, in, "blind-travel budget exhausted");
        // far-field losses time out into an active search; terminal commits stay
        // coasting forward until one of the odometry bounds above fires.
        else if (!came_from_terminal_ && (in.stamp - t_last_good_) > cfg_.coast_timeout)
                                                 enter(EngagementState::REACQUIRE, in, "coast timed out");
        break;
      }

      // ---- lost for a while: hold and actively search around last LOS --
      case EngagementState::REACQUIRE:
        if (!in.want_engage)      enter(EngagementState::IDLE, in, "disengaged");
        else if (good)            enter(rangeState(in.range), in, "reacquired");
        else if ((in.stamp - t_last_good_) > cfg_.reacquire_timeout)
                                  enter(EngagementState::ABORT, in, "reacquire timed out");
        break;

      // ---- give up: caller hands back to a safe hover ------------------
      case EngagementState::ABORT:
        if (!in.want_engage) enter(EngagementState::IDLE, in, "disengaged");
        break;
    }

    return fill(d, in);
  }

 private:
  // Along-track distance from the drone to the last known target, measured along
  // the approach direction. Positive => still short of the target; <= 0 => at or
  // past it. Uses odometry + latched target only -- no camera.
  double distanceToGo(const Eigen::Vector3d& drone_pos) const {
    if (!have_lock_) return std::numeric_limits<double>::infinity();
    return approach_dir_.dot(last_target_pos_ - drone_pos);
  }

  // Which locked state a given range belongs to.
  EngagementState rangeState(double range) const {
    return (range <= cfg_.terminal_radius) ? EngagementState::TERMINAL
                                           : EngagementState::TRACK;
  }

  void enter(EngagementState s, const EngagementInput& in, const char* why) {
    state_ = s;
    t_state_entered_ = in.stamp;
    pending_reason_ = why;
    if (s == EngagementState::IDLE || s == EngagementState::ABORT) {
      came_from_terminal_ = false;
    }
  }

  void startCoast(const EngagementInput& in, bool from_terminal, const char* why) {
    came_from_terminal_ = from_terminal;
    coast_entry_pos_ = in.drone_pos;
    enter(EngagementState::COAST, in, why);
  }

  // Map the current state to the concrete behavior + guards handed to the caller.
  EngagementDecision fill(EngagementDecision d, const EngagementInput& in) const {
    d.state = state_;
    d.reason = pending_reason_;
    d.distance_to_go = distanceToGo(in.drone_pos);
    // Closing is allowed only while we still have room along the last LOS.
    d.closing_allowed = have_lock_ && d.distance_to_go > cfg_.stop_distance;

    switch (state_) {
      case EngagementState::IDLE:
      case EngagementState::ABORT:
        d.behavior = EngagementBehavior::HOLD;
        d.feature_source = FeatureSource::NONE;
        d.closing_allowed = false;
        d.reference_distance = cfg_.standoff_distance;
        break;

      case EngagementState::SEARCH:
      case EngagementState::REACQUIRE:
        d.behavior = EngagementBehavior::SEARCH_SWEEP;
        d.feature_source = FeatureSource::NONE;
        d.closing_allowed = false;  // never fly forward without a lock
        d.reference_distance = cfg_.standoff_distance;
        break;

      case EngagementState::TRACK:
        d.behavior = EngagementBehavior::ENGAGE;
        d.feature_source = FeatureSource::MEASURED;
        d.reference_distance = cfg_.standoff_distance;
        break;

      case EngagementState::TERMINAL:
        d.behavior = EngagementBehavior::ENGAGE;
        d.feature_source = FeatureSource::MEASURED;
        d.reference_distance = cfg_.contact_distance;
        break;

      case EngagementState::COAST:
        d.behavior = EngagementBehavior::COAST_FORWARD;
        d.feature_source = FeatureSource::PREDICTED;
        d.reference_distance =
            came_from_terminal_ ? cfg_.contact_distance : cfg_.standoff_distance;
        break;
    }
    return d;
  }

  EngagementConfig cfg_{};
  EngagementState state_ = EngagementState::IDLE;

  bool have_lock_ = false;          // have we ever had a valid detection to latch?
  bool came_from_terminal_ = false; // did the current coast start from TERMINAL?

  double t_last_good_ = 0.0;        // stamp of last trustworthy detection
  double t_state_entered_ = 0.0;
  const char* pending_reason_ = "init";

  double last_range_ = 0.0;
  Eigen::Vector3d last_target_pos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d approach_dir_ = Eigen::Vector3d::UnitX();
  Eigen::Vector3d coast_entry_pos_ = Eigen::Vector3d::Zero();
};

inline const char* toString(EngagementState s) {
  switch (s) {
    case EngagementState::IDLE:      return "IDLE";
    case EngagementState::SEARCH:    return "SEARCH";
    case EngagementState::TRACK:     return "TRACK";
    case EngagementState::TERMINAL:  return "TERMINAL";
    case EngagementState::COAST:     return "COAST";
    case EngagementState::REACQUIRE: return "REACQUIRE";
    case EngagementState::ABORT:     return "ABORT";
  }
  return "?";
}

}  // namespace fsc

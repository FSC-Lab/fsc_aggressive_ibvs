#pragma once

namespace fsc {

enum class AutopilotState {
  UNINIT = 0,
  BOOT = 1,
  CALIBRATING = 2,
  STANDBY = 3,
  ACTIVE = 4,
  CRITICAL = 5,
  EMERGENCY = 6,
  POWEROFF = 7,
  FLIGHT_TERMINATION = 8,
};

enum class FlightMode {
  POSITION_TRACK = 0,
  TRAJECTORY_TRACK = 1,
  VELOCITY_TRACK = 2,
  FEATURE_TRACK = 3,
  FEATURE_REACH = 4,
  HOME = 5,
  LAND = 6
};

enum class ControlMode {
  PX4_BODYRATES = 0,
  PX4_POSITION = 1,
  FSC_POSITION = 2
};

}  // namespace fsc
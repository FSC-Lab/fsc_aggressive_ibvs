#pragma once

// ROS-free logging shim for mpc_core.
//
// The numerical MPC library must not depend on any ROS API. The original ROS1
// code used ROS_ERROR / ROS_WARN / ROS_INFO; here those are replaced by these
// printf-style macros that write to stderr/stdout with an [mpc_core] tag. The
// ROS2 node layer is free to redirect/replace these if it wants rclcpp logging.

#include <cstdio>

#define MPC_LOG_ERROR(...)                       \
  do {                                           \
    std::fprintf(stderr, "[mpc_core][ERROR] ");  \
    std::fprintf(stderr, __VA_ARGS__);           \
    std::fprintf(stderr, "\n");                  \
  } while (0)

#define MPC_LOG_WARN(...)                        \
  do {                                           \
    std::fprintf(stderr, "[mpc_core][WARN]  ");  \
    std::fprintf(stderr, __VA_ARGS__);           \
    std::fprintf(stderr, "\n");                  \
  } while (0)

#define MPC_LOG_INFO(...)                        \
  do {                                           \
    std::fprintf(stdout, "[mpc_core][INFO]  ");  \
    std::fprintf(stdout, __VA_ARGS__);           \
    std::fprintf(stdout, "\n");                  \
  } while (0)

// Throttling is a ROS concept; the core just logs. The node layer can throttle.
#define MPC_LOG_INFO_THROTTLE(period_s, ...) MPC_LOG_INFO(__VA_ARGS__)

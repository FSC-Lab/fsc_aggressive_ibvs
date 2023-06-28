#pragma once

#include <polynomial_trajectories/polynomial_trajectory.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <vision_common/trajectory.h>
#include <vision_common/trajectory_point.h>
#include <Eigen/Dense>

namespace trajectory_generation_helper {

namespace polynomials {

// Constrained Polynomials
vision_common::Trajectory computeTimeOptimalTrajectory(
    const vision_common::TrajectoryPoint& s0,
    const vision_common::TrajectoryPoint& s1, const int order_of_continuity,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate, const double sampling_frequency);

vision_common::Trajectory computeFixedTimeTrajectory(
    const vision_common::TrajectoryPoint& s0,
    const vision_common::TrajectoryPoint& s1, const int order_of_continuity,
    const double execution_time, const double sampling_frequency);

// Minimum Snap Style Polynomials
vision_common::Trajectory generateMinimumSnapTrajectory(
    const Eigen::VectorXd& segment_times,
    const vision_common::TrajectoryPoint& start_state,
    const vision_common::TrajectoryPoint& end_state,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double sampling_frequency);
vision_common::Trajectory generateMinimumSnapTrajectory(
    const Eigen::VectorXd& initial_segment_times,
    const vision_common::TrajectoryPoint& start_state,
    const vision_common::TrajectoryPoint& end_state,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate, const double sampling_frequency);

vision_common::Trajectory generateMinimumSnapTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const vision_common::TrajectoryPoint& start_state,
    const vision_common::TrajectoryPoint& end_state,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double sampling_frequency);
vision_common::Trajectory generateMinimumSnapTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const vision_common::TrajectoryPoint& start_state,
    const vision_common::TrajectoryPoint& end_state,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate, const double sampling_frequency);

vision_common::Trajectory generateMinimumSnapRingTrajectory(
    const Eigen::VectorXd& segment_times,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double sampling_frequency);
vision_common::Trajectory generateMinimumSnapRingTrajectory(
    const Eigen::VectorXd& initial_segment_times,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate, const double sampling_frequency);

vision_common::Trajectory
generateMinimumSnapRingTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double sampling_frequency);
vision_common::Trajectory
generateMinimumSnapRingTrajectoryWithSegmentRefinement(
    const Eigen::VectorXd& initial_segment_times,
    const polynomial_trajectories::PolynomialTrajectorySettings&
        trajectory_settings,
    const double max_velocity, const double max_normalized_thrust,
    const double max_roll_pitch_rate, const double sampling_frequency);

// Sampling function
vision_common::Trajectory samplePolynomial(
    const polynomial_trajectories::PolynomialTrajectory& polynomial,
    const double sampling_frequency);

}  // namespace polynomials

}  // namespace trajectory_generation_helper

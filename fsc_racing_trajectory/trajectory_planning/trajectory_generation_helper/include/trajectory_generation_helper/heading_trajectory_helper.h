#pragma once

#include <vision_common/trajectory.h>

namespace trajectory_generation_helper {

namespace heading {

void addConstantHeading(const double heading,
                        vision_common::Trajectory* trajectory);
void addConstantHeadingRate(const double initial_heading,
                            const double final_heading,
                            vision_common::Trajectory* trajectory);

}  // namespace heading

}  // namespace trajectory_generation_helper

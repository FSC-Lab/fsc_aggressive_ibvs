cmake_minimum_required(VERSION 3.8)
project(mpc_core)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------------------------
# ACADO-generated solver directory (velocity-body + pitch-gimbal model).
#
# This code is NOT checked in: it is produced by running
#   utils/pose_track_mpc_gimbal.cpp
# through the ACADO code generator and copying its export directory here. See
# model/README.md. The build errors out with instructions if it is missing.
# ---------------------------------------------------------------------------
set(MPC_MODEL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/model/pose_track_mpc_gimbal"
    CACHE PATH "Directory holding the ACADO-generated solver sources")

if(NOT EXISTS "${MPC_MODEL_DIR}/acado_solver.c")
  message(FATAL_ERROR
    "mpc_core: ACADO-generated model not found at\n"
    "  ${MPC_MODEL_DIR}\n"
    "Generate it from utils/pose_track_mpc_gimbal.cpp with ACADO and copy the\n"
    "export directory there (see ros2_ws/src/mpc_core/model/README.md), or set\n"
    "-DMPC_MODEL_DIR=<path>.")
endif()

set(QPOASES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/externals_qpoases")

# ---------------------------------------------------------------------------
# mpc_solver: qpOASES + the ACADO-generated integrator/solver. ROS-agnostic.
# ---------------------------------------------------------------------------
add_library(mpc_solver
  ${QPOASES_DIR}/SRC/Bounds.cpp
  ${QPOASES_DIR}/SRC/Constraints.cpp
  ${QPOASES_DIR}/SRC/CyclingManager.cpp
  ${QPOASES_DIR}/SRC/Indexlist.cpp
  ${QPOASES_DIR}/SRC/MessageHandling.cpp
  ${QPOASES_DIR}/SRC/QProblem.cpp
  ${QPOASES_DIR}/SRC/QProblemB.cpp
  ${QPOASES_DIR}/SRC/SubjectTo.cpp
  ${QPOASES_DIR}/SRC/Utils.cpp
  ${QPOASES_DIR}/SRC/EXTRAS/SolutionAnalysis.cpp
  ${MPC_MODEL_DIR}/acado_qpoases_interface.cpp
  ${MPC_MODEL_DIR}/acado_integrator.c
  ${MPC_MODEL_DIR}/acado_solver.c
  ${MPC_MODEL_DIR}/acado_auxiliary_functions.c
)
target_include_directories(mpc_solver PUBLIC
  $<BUILD_INTERFACE:${MPC_MODEL_DIR}>
  $<BUILD_INTERFACE:${QPOASES_DIR}>
  $<BUILD_INTERFACE:${QPOASES_DIR}/INCLUDE>
  $<BUILD_INTERFACE:${QPOASES_DIR}/SRC>
)
target_link_libraries(mpc_solver Eigen3::Eigen)

# ---------------------------------------------------------------------------
# mpc_core: the ROS-free MPC wrapper + controller (Eigen only).
# ---------------------------------------------------------------------------
add_library(mpc_core
  src/mpc_wrapper.cpp
  src/mpc_controller.cpp
)
target_include_directories(mpc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mpc_core mpc_solver Eigen3::Eigen)

# ---------------------------------------------------------------------------
# Install / export so downstream ROS2 packages (node, bridge) can link.
# ---------------------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include)
install(TARGETS mpc_core mpc_solver
  EXPORT export_mpc_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_mpc_core HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

ament_package()

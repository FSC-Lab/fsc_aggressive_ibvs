# ACADO-generated solver (drop-in)

This directory must contain the **ACADO code-generation output** for the
velocity-body + 1-axis pitch-gimbal model, in a subdirectory named
`pose_track_mpc_gimbal/`.

That code is **not** checked into git (it is machine-generated and large). The
`mpc_core` build refuses to configure until it is present.

## How to generate it

1. Build/run `utils/pose_track_mpc_gimbal.cpp` with your ACADO toolkit — the
   same toolchain used to generate the original `pose_track_mpc`. It calls
   `mpc.exportCode("pose_track_mpc_gimbal")`.
2. Copy the resulting export directory here:

   ```
   cp -r <acado_export>/pose_track_mpc_gimbal \
         ros2_ws/src/mpc_core/model/pose_track_mpc_gimbal
   ```

   It must contain at least:
   - `acado_common.h`, `acado_auxiliary_functions.h`
   - `acado_solver.c`, `acado_integrator.c`, `acado_auxiliary_functions.c`
   - `acado_qpoases_interface.cpp`

3. Verify the exported dimensions match the wrapper's index maps
   (see `docs/GIMBAL_VELOCITY_IMPLEMENTATION.md` §5):
   `ACADO_NX==15`, `ACADO_NU==7`, `ACADO_NY==21`, `ACADO_NYN==14`,
   `ACADO_NOD==23`.

If you generate into a different location, point the build at it with
`-DMPC_MODEL_DIR=<abs-path>`.

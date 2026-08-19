// ============================================================================
//  Perception-aware IBVS  ---  VELOCITY-controlled body + 1-axis PITCH gimbal
// ----------------------------------------------------------------------------
//  Variant of utils/pose_track_mpc.cpp.
//
//  Differences from the original (CTBR) model:
//    * The body is VELOCITY controlled (no thrust / no full attitude).
//        - control  = commanded world velocity (vx_cmd, vy_cmd, vz_cmd)
//        - the body tracks it with a first-order lag:  dot(v) = (v_cmd - v)/tau
//        - the body flies ~level; roll/pitch are left to the autopilot.
//    * Heading (yaw) is a state, commanded by a yaw-rate input  ->  PANS camera.
//    * The camera sits on a 1-axis PITCH (tilt) gimbal:
//        - gimbal angle theta_g is a state, gimbal rate omega_g is a control.
//        - camera orientation = Rz(psi) * R_B_M * R_pitch(theta_g)   -> TILTS camera.
//    * Camera velocities vc / wc are recomputed for this frame; the bearing
//      (f1) and range (r1) IBVS dynamics are IDENTICAL to the original.
//
//  Camera pointing is therefore full pan (body yaw) + tilt (gimbal), while the
//  body just chases velocity setpoints.  Valid for moderate accelerations
//  (the "level body" assumption breaks down under very aggressive accel).
//
//  Build: run this once through ACADO (like the original) to regenerate the
//  solver C code, then point the wrapper/CMake at the new export directory.
// ============================================================================

#include <memory>
#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>

#define USE_CODE_GEN 1

int main() {
  USING_NAMESPACE_ACADO

  // --------------------------------------------------------------------------
  // States  (15)
  // --------------------------------------------------------------------------
  DifferentialState  p_x, p_y, p_z;   // world position
  DifferentialState  psi;             // body heading (yaw)  -> camera pan
  DifferentialState  v_x, v_y, v_z;   // world velocity
  DifferentialState  f1_w, f1_x, f1_y, f1_z;  // bearing (LOS) quaternion, camera frame
  DifferentialState  r1;              // range to target
  DifferentialState  theta_g;         // gimbal PITCH angle -> camera tilt
  DifferentialState  dy1, dy2;        // slack integrators (parity with original)

  // --------------------------------------------------------------------------
  // Controls  (7)
  // --------------------------------------------------------------------------
  Control  vx_cmd, vy_cmd, vz_cmd;    // commanded world velocity
  Control  psi_rate;                  // yaw rate  -> pans camera
  Control  omega_g;                   // gimbal pitch rate -> tilts camera
  Control  sv1, sv2;                  // slack variables (visibility / ttc)

  DifferentialEquation  f;
  Function              h, hN;

  // --------------------------------------------------------------------------
  // Constants
  // --------------------------------------------------------------------------
  const double t_start = 0.0;
  const double t_end   = 1.0;
  const double dt      = 0.05;
  const int    N       = round(t_end / dt);

  const double tau_v   = 0.20;        // body velocity-tracking time constant [s]

  const double v_max        = 8.0;    // max commanded speed per axis [m/s]
  const double psi_rate_max = 1.5;    // max yaw rate [rad/s]

  const double theta_g_min  = -1.5708; // gimbal travel [rad]  (-90 deg .. +30 deg)
  const double theta_g_max  =  0.5236;
  const double omega_g_max  =  3.0;    // gimbal slew rate [rad/s]

  const double sv_min = 0.0;
  const double sv_max = 1000.0;

  const double img_u_min = -0.588836;  // normalized image bounds (~half FOV)
  const double img_u_max =  0.588836;
  const double img_v_min = -0.588836;
  const double img_v_max =  0.588836;

  const double r1_min  = 0.1;
  const double epsilon = 1e-2;

  const double ttc_max     = -4.0;         // time-to-collision bound
  const double ttc_max_inv = 1.0 / ttc_max;

  // --------------------------------------------------------------------------
  // Online data  (23)   -- constants over the horizon, set at run time
  // --------------------------------------------------------------------------
  // body -> gimbal-base (mount) translation & rotation
  OnlineData  t_B_C_x, t_B_C_y, t_B_C_z;
  OnlineData  q_B_M_w, q_B_M_x, q_B_M_y, q_B_M_z;
  // target position anchor (world) used to reconstruct the drone position
  OnlineData  focus_p_x, focus_p_y, focus_p_z;
  // distance-parameterized reference trajectory (drone path)
  OnlineData  ax, bx, cx, dx;
  OnlineData  ay, by, cy, dy;
  OnlineData  az, bz, cz, dz;
  OnlineData  r_start;

  // ==========================================================================
  //  Camera orientation:   R_WC = Rz(psi) * R_BM * R_pitch(theta_g)
  // ==========================================================================
  // gimbal pitch quaternion (about camera/gimbal y-axis)
  IntermediateState pw = cos(0.5 * theta_g);
  IntermediateState py = sin(0.5 * theta_g);

  // body->camera quaternion  q_B_C = q_B_M (x) q_pitch
  IntermediateState bcw = q_B_M_w * pw - q_B_M_y * py;
  IntermediateState bcx = q_B_M_x * pw - q_B_M_z * py;
  IntermediateState bcy = q_B_M_w * py + q_B_M_y * pw;
  IntermediateState bcz = q_B_M_x * py + q_B_M_z * pw;

  // R_BC  (body -> camera-to-body rotation matrix, i.e. rotates camera vec to body)
  IntermediateState rb11 = 1 - 2 * (bcy * bcy + bcz * bcz);
  IntermediateState rb12 = 2 * (bcx * bcy - bcw * bcz);
  IntermediateState rb13 = 2 * (bcx * bcz + bcw * bcy);
  IntermediateState rb21 = 2 * (bcx * bcy + bcw * bcz);
  IntermediateState rb22 = 1 - 2 * (bcx * bcx + bcz * bcz);
  IntermediateState rb23 = 2 * (bcy * bcz - bcw * bcx);
  IntermediateState rb31 = 2 * (bcx * bcz - bcw * bcy);
  IntermediateState rb32 = 2 * (bcy * bcz + bcw * bcx);
  IntermediateState rb33 = 1 - 2 * (bcx * bcx + bcy * bcy);

  // level-body yaw
  IntermediateState cpsi = cos(psi);
  IntermediateState spsi = sin(psi);

  // R_WC = Rz(psi) * R_BC   (rotates camera-frame vector into world)
  IntermediateState r11 = cpsi * rb11 - spsi * rb21;
  IntermediateState r12 = cpsi * rb12 - spsi * rb22;
  IntermediateState r13 = cpsi * rb13 - spsi * rb23;
  IntermediateState r21 = spsi * rb11 + cpsi * rb21;
  IntermediateState r22 = spsi * rb12 + cpsi * rb22;
  IntermediateState r23 = spsi * rb13 + cpsi * rb23;
  IntermediateState r31 = rb31;
  IntermediateState r32 = rb32;
  IntermediateState r33 = rb33;

  // ==========================================================================
  //  Camera-frame velocities feeding the (unchanged) IBVS bearing dynamics
  // ==========================================================================
  // vc = R_WC^T * v_world
  IntermediateState vc_x = r11 * v_x + r21 * v_y + r31 * v_z;
  IntermediateState vc_y = r12 * v_x + r22 * v_y + r32 * v_z;
  IntermediateState vc_z = r13 * v_x + r23 * v_y + r33 * v_z;

  // wc = R_BC^T * [0,0,psi_rate]  +  [0, omega_g, 0]
  //   body yaw spins about world/body z; gimbal spins about camera y.
  IntermediateState wc_x = psi_rate * rb31;
  IntermediateState wc_y = psi_rate * rb32 + omega_g;
  IntermediateState wc_z = psi_rate * rb33;

  // ==========================================================================
  //  Bearing (f1) + range (r1) dynamics  --  IDENTICAL to the original model
  // ==========================================================================
  IntermediateState rdot1 =
      (vc_z * (2 * f1_x * f1_x + 2 * f1_y * f1_y - 1)) / 1.0 -
      (vc_x * (2 * f1_w * f1_y + 2 * f1_x * f1_z)) / 1.0 +
      (vc_y * (2 * f1_w * f1_x - 2 * f1_y * f1_z)) / 1.0;

  IntermediateState ttc1_inv = rdot1 / r1;   // inverse time-to-collision

  // ==========================================================================
  //  System dynamics
  // ==========================================================================
  f << dot(p_x) == v_x;
  f << dot(p_y) == v_y;
  f << dot(p_z) == v_z;
  f << dot(psi) == psi_rate;
  // first-order velocity tracking of the autopilot's velocity loop
  f << dot(v_x) == (vx_cmd - v_x) / tau_v;
  f << dot(v_y) == (vy_cmd - v_y) / tau_v;
  f << dot(v_z) == (vz_cmd - v_z) / tau_v;
  // bearing kinematics (unchanged from original -- uses new vc / wc)
  f << dot(f1_w) == 0.0;
  f << dot(f1_x) == 0.5 * (wc_x * ((2 * f1_w * f1_y + 2 * f1_x * f1_z) * (2 * f1_w * f1_y + 2 * f1_x * f1_z) - 1) - (vc_y * (2 * f1_x * f1_x + 2 * f1_y * f1_y - 1)) / (r1 + epsilon) + (vc_z * (2 * f1_w * f1_x - 2 * f1_y * f1_z)) / (r1 + epsilon) - wc_y * (2 * f1_w * f1_x - 2 * f1_y * f1_z) * (2 * f1_w * f1_y + 2 * f1_x * f1_z) - wc_z * (2 * f1_w * f1_y + 2 * f1_x * f1_z) * (2 * f1_x * f1_x + 2 * f1_y * f1_y - 1));
  f << dot(f1_y) == 0.5 * (wc_y * ((2 * f1_w * f1_x - 2 * f1_y * f1_z) * (2 * f1_w * f1_x - 2 * f1_y * f1_z) - 1) + (vc_x * (2 * f1_x * f1_x + 2 * f1_y * f1_y - 1)) / (r1 + epsilon) + (vc_z * (2 * f1_w * f1_y + 2 * f1_x * f1_z)) / (r1 + epsilon) - wc_x * (2 * f1_w * f1_y + 2 * f1_x * f1_z) * (2 * f1_w * f1_x - 2 * f1_y * f1_z) + wc_z * (2 * f1_w * f1_x - 2 * f1_y * f1_z) * (2 * f1_x * f1_x + 2 * f1_y * f1_y - 1));
  f << dot(f1_z) == 0.5 * (wc_z * ((2 * f1_x * f1_x + 2 * f1_y * f1_y - 1) * (2 * f1_x * f1_x + 2 * f1_y * f1_y - 1) - 1) - (vc_x * (2 * f1_w * f1_x - 2 * f1_y * f1_z)) / (r1 + epsilon) - (vc_y * (2 * f1_w * f1_y + 2 * f1_x * f1_z)) / (r1 + epsilon) - wc_x * (2 * f1_w * f1_y + 2 * f1_x * f1_z) * (2 * f1_x * f1_x + 2 * f1_y * f1_y - 1) + wc_y * (2 * f1_w * f1_x - 2 * f1_y * f1_z) * (2 * f1_x * f1_x + 2 * f1_y * f1_y - 1));
  f << dot(r1) == rdot1;
  f << dot(theta_g) == omega_g;
  f << dot(dy1) == sv1;
  f << dot(dy2) == sv2;

  // ==========================================================================
  //  Target -> drone reconstruction (progress / visual-servo cost)
  //    target = drone + R_WB * t_B_C + r1 * (R_WC * b_C)
  //  => drone = focus_p(target) - R_WB * t_B_C - r1 * (R_WC * b_C)
  // ==========================================================================
  // bearing (LOS) direction in camera frame = 3rd column of R(f1)
  IntermediateState vec1_x = 2 * f1_w * f1_y + 2 * f1_x * f1_z;
  IntermediateState vec1_y = 2 * f1_y * f1_z - 2 * f1_w * f1_x;
  IntermediateState vec1_z = -2 * f1_x * f1_x - 2 * f1_y * f1_y + 1;

  // LOS in world frame
  IntermediateState los_x = r11 * vec1_x + r12 * vec1_y + r13 * vec1_z;
  IntermediateState los_y = r21 * vec1_x + r22 * vec1_y + r23 * vec1_z;
  IntermediateState los_z = r31 * vec1_x + r32 * vec1_y + r33 * vec1_z;

  // reconstructed drone position (R_WB = Rz(psi))
  IntermediateState p_est_x = focus_p_x - (cpsi * t_B_C_x - spsi * t_B_C_y) - r1 * los_x;
  IntermediateState p_est_y = focus_p_y - (spsi * t_B_C_x + cpsi * t_B_C_y) - r1 * los_y;
  IntermediateState p_est_z = focus_p_z - t_B_C_z - r1 * los_z;

  // reference drone path, parameterized by covered distance (r_start - r1)
  IntermediateState p_poly_x = ((ax * (r_start - r1) + bx) * (r_start - r1) + cx) * (r_start - r1) + dx;
  IntermediateState p_poly_y = ((ay * (r_start - r1) + by) * (r_start - r1) + cy) * (r_start - r1) + dy;
  IntermediateState p_poly_z = ((az * (r_start - r1) + bz) * (r_start - r1) + cz) * (r_start - r1) + dz;

  // ==========================================================================
  //  Cost outputs
  // ==========================================================================
  h << p_x << p_y << p_z                     // position
    << psi                                   // heading
    << v_x << v_y << v_z                     // velocity
    << vec1_x / (vec1_z + epsilon)           // image u  (framing)
    << vec1_y / (vec1_z + epsilon)           // image v  (framing)
    << p_poly_x - p_est_x                    // visual position error x
    << p_poly_y - p_est_y                    // visual position error y
    << p_poly_z - p_est_z                    // visual position error z
    << rdot1                                 // closing rate
    << theta_g                               // gimbal angle (regularize)
    << vx_cmd << vy_cmd << vz_cmd            // velocity effort
    << psi_rate << omega_g                   // rate effort
    << sv1 << sv2;                           // slacks

  hN << p_x << p_y << p_z
     << psi
     << v_x << v_y << v_z
     << vec1_x / (vec1_z + epsilon)
     << vec1_y / (vec1_z + epsilon)
     << p_poly_x - p_est_x
     << p_poly_y - p_est_y
     << p_poly_z - p_est_z
     << rdot1
     << theta_g;

  // ==========================================================================
  //  Weights (defaults; overwritten at run time when using code generation)
  // ==========================================================================
  DMatrix Q(h.getDim(), h.getDim());
  Q.setIdentity();
  Q(0, 0) = 200;  Q(1, 1) = 200;  Q(2, 2) = 200;   // position
  Q(3, 3) = 50;                                     // yaw
  Q(4, 4) = 10;   Q(5, 5) = 10;   Q(6, 6) = 10;     // velocity
  Q(7, 7) = 1;    Q(8, 8) = 1;                      // image u,v
  Q(9, 9) = 1;    Q(10, 10) = 1;  Q(11, 11) = 1;    // visual error
  Q(12, 12) = 1;                                    // rdot1
  Q(13, 13) = 1;                                    // gimbal angle
  Q(14, 14) = 1;  Q(15, 15) = 1;  Q(16, 16) = 1;    // vel cmd
  Q(17, 17) = 1;  Q(18, 18) = 1;                    // yaw rate, gimbal rate
  Q(19, 19) = 1;  Q(20, 20) = 1;                    // slacks

  DMatrix QN(hN.getDim(), hN.getDim());
  QN.setIdentity();
  for (int i = 0; i < (int)hN.getDim(); ++i) QN(i, i) = Q(i, i);

  // ==========================================================================
  //  OCP
  // ==========================================================================
  OCP ocp(t_start, t_end, N);

#ifndef USE_CODE_GEN
  DVector r(h.getDim());   r.setZero();
  r(3)  = 0.0;   // yaw ref
  r(12) = -2.0;  // desired closing rate
  DVector rN(hN.getDim()); rN.setZero(); rN(12) = -2.0;
  ocp.minimizeLSQ(Q, h, r);
  ocp.minimizeLSQEndTerm(QN, hN, rN);
#else
  BMatrix Q_sparse(h.getDim(), h.getDim());   Q_sparse.setIdentity();
  BMatrix QN_sparse(hN.getDim(), hN.getDim()); QN_sparse.setIdentity();
  ocp.minimizeLSQ(Q_sparse, h);
  ocp.minimizeLSQEndTerm(QN_sparse, hN);
#endif

  // dynamics
  ocp.subjectTo(f);

  // input / state limits
  ocp.subjectTo(-v_max <= vx_cmd <= v_max);
  ocp.subjectTo(-v_max <= vy_cmd <= v_max);
  ocp.subjectTo(-v_max <= vz_cmd <= v_max);
  ocp.subjectTo(-psi_rate_max <= psi_rate <= psi_rate_max);
  ocp.subjectTo(-omega_g_max <= omega_g <= omega_g_max);
  ocp.subjectTo(theta_g_min <= theta_g <= theta_g_max);
  ocp.subjectTo(sv_min <= sv1 <= sv_max);
  ocp.subjectTo(sv_min <= sv2 <= sv_max);
  ocp.subjectTo(r1_min <= r1);

  // visibility constraint (softened by sv1)
  ocp.subjectTo(vec1_x / (vec1_z + epsilon) - sv1 <= img_u_max);
  ocp.subjectTo(img_u_min <= vec1_x / (vec1_z + epsilon) + sv1);
  ocp.subjectTo(vec1_y / (vec1_z + epsilon) - sv1 <= img_v_max);
  ocp.subjectTo(img_v_min <= vec1_y / (vec1_z + epsilon) + sv1);

  // time-to-collision constraint (softened by sv2)
  ocp.subjectTo(ttc_max_inv <= ttc1_inv + sv2);

  ocp.setNOD(23);

#ifndef USE_CODE_GEN
  OptimizationAlgorithm algorithm(ocp);
  algorithm.set(INTEGRATOR_TOLERANCE, 1e-6);
  algorithm.set(KKT_TOLERANCE, 1e-3);
  algorithm.solve();
#else
  OCPexport mpc(ocp);
  mpc.set(HESSIAN_APPROXIMATION,  GAUSS_NEWTON);
  mpc.set(DISCRETIZATION_TYPE,    MULTIPLE_SHOOTING);
  mpc.set(SPARSE_QP_SOLUTION,     FULL_CONDENSING_N2);
  mpc.set(INTEGRATOR_TYPE,        INT_IRK_GL4);
  mpc.set(NUM_INTEGRATOR_STEPS,   N);
  mpc.set(QP_SOLVER,              QP_QPOASES);
  mpc.set(HOTSTART_QP,            YES);
  mpc.set(CG_USE_OPENMP,                    YES);
  mpc.set(CG_HARDCODE_CONSTRAINT_VALUES,    NO);
  mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);
  mpc.set(USE_SINGLE_PRECISION,             YES);
  mpc.set(GENERATE_TEST_FILE,          NO);
  mpc.set(GENERATE_MAKE_FILE,          NO);
  mpc.set(GENERATE_MATLAB_INTERFACE,   NO);
  mpc.set(GENERATE_SIMULINK_INTERFACE, NO);

  if (mpc.exportCode("pose_track_mpc_gimbal") != SUCCESSFUL_RETURN)
    exit(EXIT_FAILURE);
  mpc.printDimensionsQP();
#endif

  return EXIT_SUCCESS;
}

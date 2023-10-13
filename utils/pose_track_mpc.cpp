#include <memory>
#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>

#define USE_CODE_GEN 1

int main(){
  // Use Acado
  USING_NAMESPACE_ACADO

  // r_startstem variables
  DifferentialState     p_x, p_y, p_z;
  DifferentialState     q_w, q_x, q_y, q_z;
  DifferentialState     v_x, v_y, v_z;
  DifferentialState     Th;
  DifferentialState     f1_w, f1_x, f1_y, f1_z;
  DifferentialState     r1;
  DifferentialState     dy1;  // dummy1 state
  DifferentialState     dy2;  // dummy2 state
  Control               dTh, w_x, w_y, w_z;
  Control               sv1; // slack variable
  Control               sv2; // slack variable
  DifferentialEquation  f;
  Function              h, hN;

  const double t_start = 0.0;     // Initial time [s]
  const double t_end = 1.0;       // Time horizon [s]
  const double dt = 0.05;          // Discretization time [s]
  const int N = round(t_end/dt);  // Number of nodes
  const double g_z = 9.81;      // Gravity is everywhere [m/s^2]
  const double w_max_yaw = 1;     // Maximal yaw rate [rad/s]
  const double w_max_xy = 3;      // Maximal pitch and roll rate [rad/s]
  const double min_thrust = 2;         // Minimal thrust [N]
  const double max_thrust = 20;        // Maximal thrust [N]
  const double max_thrust_dot = 6;        // Maximal thrust rate [N]

  const double sv_min = 0.0;    // slack variable for visibility constraint
  const double sv_max = 1000.0;    // slack variable for collision constraint
  const double img_u_min = -0.588836;
  const double img_u_max = 0.588836;
  const double img_v_min = -0.588836;
  const double img_v_max = 0.588836;
  const double r1_min = 0.1;
  const double r1_max = 100.0;
  const double epsilon = 1e-2;

  const double ttc_max = -4;
  const double ttc_max_inv = 1.0f / ttc_max;

#ifndef USE_CODE_GEN

#else
  // camera extrinsics: 7
  OnlineData            t_B_C_x, t_B_C_y, t_B_C_z;
  OnlineData            q_B_C_w, q_B_C_x, q_B_C_y, q_B_C_z;
  // focus: 7
  OnlineData            focus_p_x, focus_p_y, focus_p_z;
  OnlineData            focus_q_w, focus_q_x, focus_q_y, focus_q_z;
  // gate: 7 
  OnlineData            box_p_x, box_p_y, box_p_z;
  OnlineData            box_q_w, box_q_x, box_q_y, box_q_z;
  // distance parameterized trajectory: 13
  OnlineData            ax, bx, cx, dx;
  OnlineData            ay, by, cy, dy;
  OnlineData            az, bz, cz, dz;
  OnlineData            r_start;
  // gate size and quadrotor ball radius: 3
  OnlineData            box_width, box_height, quad_radius;
#endif

  IntermediateState vc_x = v_z*((2*q_B_C_w*q_B_C_y - 2*q_B_C_x*q_B_C_z)*(2*q_x*q_x + 2*q_y*q_y - 1) + (2*q_w*q_y - 2*q_x*q_z)*(2*q_B_C_y*q_B_C_y + 2*q_B_C_z*q_B_C_z - 1) + (2*q_B_C_w*q_B_C_z + 2*q_B_C_x*q_B_C_y)*(2*q_w*q_x + 2*q_y*q_z)) - v_x*((2*q_B_C_w*q_B_C_y - 2*q_B_C_x*q_B_C_z)*(2*q_w*q_y + 2*q_x*q_z) + (2*q_B_C_w*q_B_C_z + 2*q_B_C_x*q_B_C_y)*(2*q_w*q_z - 2*q_x*q_y) - (2*q_B_C_y*q_B_C_y + 2*q_B_C_z*q_B_C_z - 1)*(2*q_y*q_y + 2*q_z*q_z - 1)) - v_y*((2*q_B_C_w*q_B_C_z + 2*q_B_C_x*q_B_C_y)*(2*q_x*q_x + 2*q_z*q_z - 1) + (2*q_w*q_z + 2*q_x*q_y)*(2*q_B_C_y*q_B_C_y + 2*q_B_C_z*q_B_C_z - 1) - (2*q_B_C_w*q_B_C_y - 2*q_B_C_x*q_B_C_z)*(2*q_w*q_x - 2*q_y*q_z));
  IntermediateState vc_y = v_x*((2*q_B_C_w*q_B_C_z - 2*q_B_C_x*q_B_C_y)*(2*q_y*q_y + 2*q_z*q_z - 1) + (2*q_w*q_z - 2*q_x*q_y)*(2*q_B_C_x*q_B_C_x + 2*q_B_C_z*q_B_C_z - 1) + (2*q_B_C_w*q_B_C_x + 2*q_B_C_y*q_B_C_z)*(2*q_w*q_y + 2*q_x*q_z)) - v_y*((2*q_B_C_w*q_B_C_x + 2*q_B_C_y*q_B_C_z)*(2*q_w*q_x - 2*q_y*q_z) + (2*q_B_C_w*q_B_C_z - 2*q_B_C_x*q_B_C_y)*(2*q_w*q_z + 2*q_x*q_y) - (2*q_B_C_x*q_B_C_x + 2*q_B_C_z*q_B_C_z - 1)*(2*q_x*q_x + 2*q_z*q_z - 1)) - v_z*((2*q_B_C_w*q_B_C_x + 2*q_B_C_y*q_B_C_z)*(2*q_x*q_x + 2*q_y*q_y - 1) + (2*q_w*q_x + 2*q_y*q_z)*(2*q_B_C_x*q_B_C_x + 2*q_B_C_z*q_B_C_z - 1) - (2*q_B_C_w*q_B_C_z - 2*q_B_C_x*q_B_C_y)*(2*q_w*q_y - 2*q_x*q_z));
  IntermediateState vc_z = v_y*((2*q_B_C_w*q_B_C_x - 2*q_B_C_y*q_B_C_z)*(2*q_x*q_x + 2*q_z*q_z - 1) + (2*q_w*q_x - 2*q_y*q_z)*(2*q_B_C_x*q_B_C_x + 2*q_B_C_y*q_B_C_y - 1) + (2*q_B_C_w*q_B_C_y + 2*q_B_C_x*q_B_C_z)*(2*q_w*q_z + 2*q_x*q_y)) - v_x*((2*q_B_C_w*q_B_C_y + 2*q_B_C_x*q_B_C_z)*(2*q_y*q_y + 2*q_z*q_z - 1) + (2*q_w*q_y + 2*q_x*q_z)*(2*q_B_C_x*q_B_C_x + 2*q_B_C_y*q_B_C_y - 1) - (2*q_B_C_w*q_B_C_x - 2*q_B_C_y*q_B_C_z)*(2*q_w*q_z - 2*q_x*q_y)) - v_z*((2*q_B_C_w*q_B_C_x - 2*q_B_C_y*q_B_C_z)*(2*q_w*q_x + 2*q_y*q_z) + (2*q_B_C_w*q_B_C_y + 2*q_B_C_x*q_B_C_z)*(2*q_w*q_y - 2*q_x*q_z) - (2*q_B_C_x*q_B_C_x + 2*q_B_C_y*q_B_C_y - 1)*(2*q_x*q_x + 2*q_y*q_y - 1));

  IntermediateState wc_x = w_y*(2*q_B_C_w*q_B_C_z + 2*q_B_C_x*q_B_C_y) - w_z*(2*q_B_C_w*q_B_C_y - 2*q_B_C_x*q_B_C_z) - w_x*(2*q_B_C_y*q_B_C_y + 2*q_B_C_z*q_B_C_z - 1);
  IntermediateState wc_y = w_z*(2*q_B_C_w*q_B_C_x + 2*q_B_C_y*q_B_C_z) - w_x*(2*q_B_C_w*q_B_C_z - 2*q_B_C_x*q_B_C_y) - w_y*(2*q_B_C_x*q_B_C_x + 2*q_B_C_z*q_B_C_z - 1);
  IntermediateState wc_z = w_x*(2*q_B_C_w*q_B_C_y + 2*q_B_C_x*q_B_C_z) - w_y*(2*q_B_C_w*q_B_C_x - 2*q_B_C_y*q_B_C_z) - w_z*(2*q_B_C_x*q_B_C_x + 2*q_B_C_y*q_B_C_y - 1);
 
  IntermediateState rdot1 = (vc_z*(2*f1_x*f1_x + 2*f1_y*f1_y - 1))/1.0 - (vc_x*(2*f1_w*f1_y + 2*f1_x*f1_z))/1.0 + (vc_y*(2*f1_w*f1_x - 2*f1_y*f1_z))/1.0;
  
  // inverse of time to collision
  IntermediateState ttc1_inv = rdot1/r1;


  // System Dynamics
  f << dot(p_x) ==  v_x;
  f << dot(p_y) ==  v_y;
  f << dot(p_z) ==  v_z;
  f << dot(q_w) ==  0.5 * ( - w_x * q_x - w_y * q_y - w_z * q_z);
  f << dot(q_x) ==  0.5 * ( w_x * q_w + w_z * q_y - w_y * q_z);
  f << dot(q_y) ==  0.5 * ( w_y * q_w - w_z * q_x + w_x * q_z);
  f << dot(q_z) ==  0.5 * ( w_z * q_w + w_y * q_x - w_x * q_y);
  f << dot(v_x) ==  2 * ( q_w * q_y + q_x * q_z ) * Th;
  f << dot(v_y) ==  2 * ( q_y * q_z - q_w * q_x ) * Th;
  f << dot(v_z) ==  ( 1 - 2 * q_x * q_x - 2 * q_y * q_y ) * Th - g_z;
  f << dot(Th) == dTh;
  f << dot(f1_w) == 0.0;
  f << dot(f1_x) == 0.5 * (wc_x*((2*f1_w*f1_y + 2*f1_x*f1_z)*(2*f1_w*f1_y + 2*f1_x*f1_z) - 1) - (vc_y*(2*f1_x*f1_x + 2*f1_y*f1_y - 1))/(r1+epsilon) + (vc_z*(2*f1_w*f1_x - 2*f1_y*f1_z))/(r1+epsilon) - wc_y*(2*f1_w*f1_x - 2*f1_y*f1_z)*(2*f1_w*f1_y + 2*f1_x*f1_z) - wc_z*(2*f1_w*f1_y + 2*f1_x*f1_z)*(2*f1_x*f1_x + 2*f1_y*f1_y - 1));
  f << dot(f1_y) == 0.5 * (wc_y*((2*f1_w*f1_x - 2*f1_y*f1_z)*(2*f1_w*f1_x - 2*f1_y*f1_z) - 1) + (vc_x*(2*f1_x*f1_x + 2*f1_y*f1_y - 1))/(r1+epsilon) + (vc_z*(2*f1_w*f1_y + 2*f1_x*f1_z))/(r1+epsilon) - wc_x*(2*f1_w*f1_y + 2*f1_x*f1_z)*(2*f1_w*f1_x - 2*f1_y*f1_z) + wc_z*(2*f1_w*f1_x - 2*f1_y*f1_z)*(2*f1_x*f1_x + 2*f1_y*f1_y - 1));
  f << dot(f1_z) == 0.5 * (wc_z*((2*f1_x*f1_x + 2*f1_y*f1_y - 1)*(2*f1_x*f1_x + 2*f1_y*f1_y - 1) - 1) - (vc_x*(2*f1_w*f1_x - 2*f1_y*f1_z))/(r1+epsilon) - (vc_y*(2*f1_w*f1_y + 2*f1_x*f1_z))/(r1+epsilon) - wc_x*(2*f1_w*f1_y + 2*f1_x*f1_z)*(2*f1_x*f1_x + 2*f1_y*f1_y - 1) + wc_y*(2*f1_w*f1_x - 2*f1_y*f1_z)*(2*f1_x*f1_x + 2*f1_y*f1_y - 1));
  f << dot(r1) == rdot1;
  f << dot(dy1) == sv1;
  f << dot(dy2) == sv2;

  // referece point in the world frame
  IntermediateState p_est_x = focus_p_x + (2*q_y*q_y + 2*q_z*q_z - 1)*(t_B_C_x - r1*(2*f1_w*f1_y + 2*f1_x*f1_z)*(2*q_B_C_y*q_B_C_y + 2*q_B_C_z*q_B_C_z - 1) - r1*(2*q_B_C_w*q_B_C_y + 2*q_B_C_x*q_B_C_z)*(2*f1_x*f1_x + 2*f1_y*f1_y - 1) + r1*(2*f1_w*f1_x - 2*f1_y*f1_z)*(2*q_B_C_w*q_B_C_z - 2*q_B_C_x*q_B_C_y)) + (2*q_w*q_z - 2*q_x*q_y)*(t_B_C_y + r1*(2*f1_w*f1_x - 2*f1_y*f1_z)*(2*q_B_C_x*q_B_C_x + 2*q_B_C_z*q_B_C_z - 1) + r1*(2*q_B_C_w*q_B_C_x - 2*q_B_C_y*q_B_C_z)*(2*f1_x*f1_x + 2*f1_y*f1_y - 1) + r1*(2*f1_w*f1_y + 2*f1_x*f1_z)*(2*q_B_C_w*q_B_C_z + 2*q_B_C_x*q_B_C_y)) - (2*q_w*q_y + 2*q_x*q_z)*(t_B_C_z + r1*(2*f1_x*f1_x + 2*f1_y*f1_y - 1)*(2*q_B_C_x*q_B_C_x + 2*q_B_C_y*q_B_C_y - 1) - r1*(2*f1_w*f1_x - 2*f1_y*f1_z)*(2*q_B_C_w*q_B_C_x + 2*q_B_C_y*q_B_C_z) - r1*(2*f1_w*f1_y + 2*f1_x*f1_z)*(2*q_B_C_w*q_B_C_y - 2*q_B_C_x*q_B_C_z));
  IntermediateState p_est_y = focus_p_y + (2*q_x*q_x + 2*q_z*q_z - 1)*(t_B_C_y + r1*(2*f1_w*f1_x - 2*f1_y*f1_z)*(2*q_B_C_x*q_B_C_x + 2*q_B_C_z*q_B_C_z - 1) + r1*(2*q_B_C_w*q_B_C_x - 2*q_B_C_y*q_B_C_z)*(2*f1_x*f1_x + 2*f1_y*f1_y - 1) + r1*(2*f1_w*f1_y + 2*f1_x*f1_z)*(2*q_B_C_w*q_B_C_z + 2*q_B_C_x*q_B_C_y)) + (2*q_w*q_x - 2*q_y*q_z)*(t_B_C_z + r1*(2*f1_x*f1_x + 2*f1_y*f1_y - 1)*(2*q_B_C_x*q_B_C_x + 2*q_B_C_y*q_B_C_y - 1) - r1*(2*f1_w*f1_x - 2*f1_y*f1_z)*(2*q_B_C_w*q_B_C_x + 2*q_B_C_y*q_B_C_z) - r1*(2*f1_w*f1_y + 2*f1_x*f1_z)*(2*q_B_C_w*q_B_C_y - 2*q_B_C_x*q_B_C_z)) - (2*q_w*q_z + 2*q_x*q_y)*(t_B_C_x - r1*(2*f1_w*f1_y + 2*f1_x*f1_z)*(2*q_B_C_y*q_B_C_y + 2*q_B_C_z*q_B_C_z - 1) - r1*(2*q_B_C_w*q_B_C_y + 2*q_B_C_x*q_B_C_z)*(2*f1_x*f1_x + 2*f1_y*f1_y - 1) + r1*(2*f1_w*f1_x - 2*f1_y*f1_z)*(2*q_B_C_w*q_B_C_z - 2*q_B_C_x*q_B_C_y));
  IntermediateState p_est_z = focus_p_z + (2*q_x*q_x + 2*q_y*q_y - 1)*(t_B_C_z + r1*(2*f1_x*f1_x + 2*f1_y*f1_y - 1)*(2*q_B_C_x*q_B_C_x + 2*q_B_C_y*q_B_C_y - 1) - r1*(2*f1_w*f1_x - 2*f1_y*f1_z)*(2*q_B_C_w*q_B_C_x + 2*q_B_C_y*q_B_C_z) - r1*(2*f1_w*f1_y + 2*f1_x*f1_z)*(2*q_B_C_w*q_B_C_y - 2*q_B_C_x*q_B_C_z)) - (2*q_w*q_x + 2*q_y*q_z)*(t_B_C_y + r1*(2*f1_w*f1_x - 2*f1_y*f1_z)*(2*q_B_C_x*q_B_C_x + 2*q_B_C_z*q_B_C_z - 1) + r1*(2*q_B_C_w*q_B_C_x - 2*q_B_C_y*q_B_C_z)*(2*f1_x*f1_x + 2*f1_y*f1_y - 1) + r1*(2*f1_w*f1_y + 2*f1_x*f1_z)*(2*q_B_C_w*q_B_C_z + 2*q_B_C_x*q_B_C_y)) + (2*q_w*q_y - 2*q_x*q_z)*(t_B_C_x - r1*(2*f1_w*f1_y + 2*f1_x*f1_z)*(2*q_B_C_y*q_B_C_y + 2*q_B_C_z*q_B_C_z - 1) - r1*(2*q_B_C_w*q_B_C_y + 2*q_B_C_x*q_B_C_z)*(2*f1_x*f1_x + 2*f1_y*f1_y - 1) + r1*(2*f1_w*f1_x - 2*f1_y*f1_z)*(2*q_B_C_w*q_B_C_z - 2*q_B_C_x*q_B_C_y));
  
  // progress costs h = x - m_x[idx] = (d_max - r1) - (d_max - r_start) = r_start - r1
  IntermediateState p_poly_x = ((ax * (r_start - r1) + bx) * (r_start - r1) + cx) * (r_start - r1) + dx;
  IntermediateState p_poly_y = ((ay * (r_start - r1) + by) * (r_start - r1) + cy) * (r_start - r1) + dy;
  IntermediateState p_poly_z = ((az * (r_start - r1) + bz) * (r_start - r1) + cz) * (r_start - r1) + dz;

  // projection on the gate coordinate
  IntermediateState box_x = ((2*box_q_w*box_q_y - 2*box_q_x*box_q_z)*(box_p_z - p_est_z) - (2*box_q_w*box_q_z + 2*box_q_x*box_q_y)*(box_p_y - p_est_y) + (box_p_x - p_est_x)*(2*box_q_y*box_q_y + 2*box_q_z*box_q_z - 1)) / (0.5*box_width - quad_radius);
  IntermediateState box_y = ((2*box_q_w*box_q_z - 2*box_q_x*box_q_y)*(box_p_x - p_est_x) - (2*box_q_w*box_q_x + 2*box_q_y*box_q_z)*(box_p_z - p_est_z) + (box_p_y - p_est_y)*(2*box_q_x*box_q_x + 2*box_q_z*box_q_z - 1)) / (0.5*box_height - quad_radius);

  IntermediateState vec1_x = 2*f1_w*f1_y + 2*f1_x*f1_z; 
  IntermediateState vec1_y = 2*f1_y*f1_z - 2*f1_w*f1_x;
  IntermediateState vec1_z = - 2*f1_x*f1_x - 2*f1_y*f1_y + 1;

  h << p_x
    << p_y
    << p_z
    << q_w << q_x << q_y << q_z
    << v_x << v_y << v_z
    << Th
    << vec1_x/(vec1_z+epsilon)
    << vec1_y/(vec1_z+epsilon)
    << p_poly_x - p_est_x
    << p_poly_y - p_est_y
    << p_poly_z - p_est_z
    << rdot1
    << dTh << w_x << w_y << w_z
    << sv1 << sv2;

  hN << p_x
     << p_y
     << p_z
     << q_w << q_x << q_y << q_z
     << v_x << v_y << v_z
     << Th
     << vec1_x/(vec1_z+epsilon)
     << vec1_y/(vec1_z+epsilon)
     << p_poly_x - p_est_x
     << p_poly_y - p_est_y
     << p_poly_z - p_est_z
     << rdot1;

  // Running cost weight matrix
  DMatrix Q(h.getDim(), h.getDim());
  Q.setIdentity();
  Q(0,0) = 200;   // x
  Q(1,1) = 200;   // y
  Q(2,2) = 200;   // z
  Q(3,3) = 50;   // qw
  Q(4,4) = 50;   // qx
  Q(5,5) = 50;   // qy
  Q(6,6) = 50;   // qz
  Q(7,7) = 10;    // vx
  Q(8,8) = 10;    // vy
  Q(9,9) = 10;    // vz
  Q(10,10) = 1;   // Th
  Q(11,11) = 1;   // img_u
  Q(12,12) = 1;   // img_v
  Q(13,13) = 1;   // e_x
  Q(14,14) = 1;   // e_y
  Q(15,15) = 1;   // e_z
  Q(16,16) = 1;   // rdot1

  Q(17,17) = 1;   // dTh
  Q(18,18) = 1;   // wx
  Q(19,19) = 1;   // wy
  Q(20,20) = 1;   // wz
  Q(21,21) = 1;   // sv1
  Q(22,22) = 1;   // sv2

  // End cost weight matrix
  DMatrix QN(hN.getDim(), hN.getDim());
  QN.setIdentity();
  QN(0,0) = Q(0,0);   // x
  QN(1,1) = Q(1,1);   // y
  QN(2,2) = Q(2,2);   // z
  QN(3,3) = Q(3,3);   // qw
  QN(4,4) = Q(4,4);   // qx
  QN(5,5) = Q(5,5);   // qy
  QN(6,6) = Q(6,6);   // qz
  QN(7,7) = Q(7,7);   // vx
  QN(8,8) = Q(8,8);   // vy
  QN(9,9) = Q(9,9);   // vz
  QN(10,10) = Q(10,10);   // Th
  QN(11,11) = Q(11,11);   
  QN(12,12) = Q(12,12);   
  QN(13,13) = Q(13,13);   
  QN(14,14) = Q(14,14);   
  QN(15,15) = Q(15,15);  
  QN(16,16) = Q(16,16);

  // Reference is at x = 0.0m in hover (qw = 1).
  DVector r(h.getDim());    // Running cost reference
  r.setZero();
  r(0) = 0.5; // x
  r(1) = -0.5; // y
  r(2) = 0.0; // z
  r(3) = 1.0; // qw
  r(4) = 0.0; // qx
  r(5) = 0.0; // qy
  r(6) = 0.0; // qz
  r(7) = 0.0; // vx
  r(8) = 0.0; // vy
  r(9) = 0.0; // vz 
  r(10) = g_z; // Th
  r(11) = 0.0; // img_u
  r(12) = 0.0; // img_v
  r(13) = 0.0; // e_x
  r(14) = 0.0; // e_y
  r(15) = 0.0; // e_z
  r(16) = -2.0; // rdot1

  r(11) = 0.0; // dTh
  r(12) = 0.0; // wx
  r(13) = 0.0; // wy
  r(14) = 0.0; // wz 

  DVector rN(hN.getDim());   // End cost reference
  rN(0) = r(0);
  rN(1) = r(1);
  rN(2) = r(2);
  rN(3) = r(3);
  rN(4) = r(4);
  rN(5) = r(5);
  rN(6) = r(6);
  rN(7) = r(7);
  rN(8) = r(8);
  rN(9) = r(9);
  rN(10) = r(10);
  rN(11) = r(11);
  rN(12) = r(12);
  rN(13) = r(13);
  rN(14) = r(14);
  rN(15) = r(15);
  rN(16) = r(16);

  // DEFINE AN OPTIMAL CONTROL PROBLEM:
  // ----------------------------------
  OCP ocp( t_start, t_end, N );

#ifndef USE_CODE_GEN
  // For analysis, set references.
  ocp.minimizeLSQ( Q, h, r );
  ocp.minimizeLSQEndTerm( QN, hN, rN );
#else
  // For code generation, references are set during run time.
  BMatrix Q_sparse(h.getDim(), h.getDim());
  Q_sparse.setIdentity();
  BMatrix QN_sparse(hN.getDim(), hN.getDim());
  QN_sparse.setIdentity();
  ocp.minimizeLSQ( Q_sparse, h);
  ocp.minimizeLSQEndTerm( QN_sparse, hN );
#endif

  // Add system dynamics
  ocp.subjectTo( f );
  // Add constraints
  ocp.subjectTo(-w_max_xy <= w_x <= w_max_xy);
  ocp.subjectTo(-w_max_xy <= w_y <= w_max_xy);
  ocp.subjectTo(-w_max_yaw <= w_z <= w_max_yaw);
  ocp.subjectTo(-max_thrust_dot <= dTh <= max_thrust_dot);
  ocp.subjectTo(sv_min <= sv1 <= sv_max);
  ocp.subjectTo(sv_min <= sv2 <= sv_max);

  ocp.subjectTo(r1_min <= r1);
  ocp.subjectTo(min_thrust <= Th <= max_thrust);

  ocp.subjectTo(vec1_x/(vec1_z+epsilon) - sv1  <= img_u_max);
  ocp.subjectTo(img_u_min <= vec1_x/(vec1_z+epsilon) + sv1);
  ocp.subjectTo(vec1_y/(vec1_z+epsilon) - sv1  <= img_v_max);
  ocp.subjectTo(img_v_min <= vec1_y/(vec1_z+epsilon) + sv1);

  ocp.subjectTo(ttc_max_inv <= ttc1_inv + sv2);

  // ocp.subjectTo(box_x - sv2  <= 1.0);
  // ocp.subjectTo(-1.0 <= box_x + sv2);
  // ocp.subjectTo(box_y - sv2  <= 1.0);
  // ocp.subjectTo(-1.0 <= box_y + sv2);

  ocp.setNOD(37);

#ifndef USE_CODE_GEN
  // Set initial state
  ocp.subjectTo( AT_START, p_x ==  0.0 );
  ocp.subjectTo( AT_START, p_y ==  0.0 );
  ocp.subjectTo( AT_START, p_z ==  1.0 );
  ocp.subjectTo( AT_START, q_w ==  1.0 );
  ocp.subjectTo( AT_START, q_x ==  0.0 );
  ocp.subjectTo( AT_START, q_y ==  0.0 );
  ocp.subjectTo( AT_START, q_z ==  0.0 );
  ocp.subjectTo( AT_START, v_x ==  0.0 );
  ocp.subjectTo( AT_START, v_y ==  0.0 );
  ocp.subjectTo( AT_START, v_z ==  0.0 );
  ocp.subjectTo( AT_START, Th ==  g_z );
  ocp.subjectTo( AT_START, dTh ==  0.0 );
  ocp.subjectTo( AT_START, w_x ==  0.0 );
  ocp.subjectTo( AT_START, w_y ==  0.0 );
  ocp.subjectTo( AT_START, w_z ==  0.0 );

  // Setup some visualization
  GnuplotWindow window1( PLOT_AT_EACH_ITERATION );
  window1.addSubplot( p_x,"position x" );
  window1.addSubplot( p_y,"position y" );
  window1.addSubplot( p_z,"position z" );
  window1.addSubplot( v_x,"verlocity x" );
  window1.addSubplot( v_y,"verlocity y" );
  window1.addSubplot( Th,"thrust" );

  GnuplotWindow window2( PLOT_AT_EACH_ITERATION );
  window2.addSubplot( w_x,"rotation-acc x" );
  window2.addSubplot( w_y,"rotation-acc y" );
  window2.addSubplot( w_z,"rotation-acc z" ); 
  window2.addSubplot( dTh,"thrust dot" );

  // Define an algorithm to solve it.
  OptimizationAlgorithm algorithm(ocp);
  algorithm.set( INTEGRATOR_TOLERANCE, 1e-6 );
  algorithm.set( KKT_TOLERANCE, 1e-3 );
  algorithm << window1;
  algorithm << window2;
  algorithm.solve();
#else
  // For code generation, we can set some properties.
  // The main reason for a setting is given as comment.
  OCPexport mpc(ocp);

  mpc.set(HESSIAN_APPROXIMATION,  GAUSS_NEWTON);        // is robust, stable
  mpc.set(DISCRETIZATION_TYPE,    MULTIPLE_SHOOTING);   // good convergence
  mpc.set(SPARSE_QP_SOLUTION,     FULL_CONDENSING_N2);  // due to qpOASES
  mpc.set(INTEGRATOR_TYPE,        INT_IRK_GL4);         // accurate
  mpc.set(NUM_INTEGRATOR_STEPS,   N);
  mpc.set(QP_SOLVER,              QP_QPOASES);          // free, source code
  mpc.set(HOTSTART_QP,            YES);
  mpc.set(CG_USE_OPENMP,                    YES);       // paralellization
  mpc.set(CG_HARDCODE_CONSTRAINT_VALUES,    NO);        // set on runtime
  mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);       // time-varying costs
  mpc.set( USE_SINGLE_PRECISION,        YES);           // Single precision

  // Do not generate tests, makes or matlab-related interfaces.
  mpc.set( GENERATE_TEST_FILE,          NO);
  mpc.set( GENERATE_MAKE_FILE,          NO);
  mpc.set( GENERATE_MATLAB_INTERFACE,   NO);
  mpc.set( GENERATE_SIMULINK_INTERFACE, NO);

  // Finally, export everything.
  if(mpc.exportCode("pose_track_mpc") != SUCCESSFUL_RETURN)
    exit( EXIT_FAILURE );
  mpc.printDimensionsQP( );
#endif

  return EXIT_SUCCESS;

}
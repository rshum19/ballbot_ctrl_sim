//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: IO_PD_controller_c.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 28-Aug-2017 11:45:43
//

// Include Files
#include "rt_nonfinite.h"
#include "IO_PD_controller_c.h"

// Function Definitions

//
// IO_PD_controller  I/O feedback linearized controller with an outer loop PD
//  AUTHOR:   Roberto Shu
//  LAST EDIT: 8/26/2017
//     %% ----------------------------------------------------------
//    Unpack actual and desired state vectors
//  -----------------------------------------------------------
//  State variables
// Arguments    : const double x[4]
//                const double x_d[4]
//                double *u
//                struct0_T *uparts
// Return Type  : void
//
void IO_PD_controller_c(const double x[4], const double x_d[4], double *u,
  struct0_T *uparts)
{
  double err[4];
  double t6;
  int i0;
  double phi_des;
  static const double a[4] = { 0.01, 0.015, 0.0, 0.0 };

  double t2;
  double LgLfh;
  double u_ff;
  double b_x[2];

  //  Desired state
  //     %% ----------------------------------------------------------
  //    Outer loop control
  //  -----------------------------------------------------------
  //  Aim: get a desired lean angle (phi_des) to track desired ball
  //  position using a simple PD controller
  //  Error state vector
  err[0] = x[0] - x_d[0];
  err[1] = x[2] - x_d[2];
  err[2] = x[1] - x_d[1];
  err[3] = x[3] - x_d[3];

  //  Control gains
  //  Augmented desired lean angle
  t6 = 0.0;
  for (i0 = 0; i0 < 4; i0++) {
    t6 += a[i0] * err[i0];
  }

  phi_des = x_d[1] + t6;

  //     %% ----------------------------------------------------------
  //    Inner loop control
  //  -----------------------------------------------------------
  //  Aim: I/O feedback linearized controller with PD to track desired lean angles 
  //  to balance and navigate
  //  Output function
  //  Output function lie derative
  // AUTOFUN_LIEDERV_BALLBOT2D_IO_PD
  //     [LGLFH,LF2H] = AUTOFUN_LIEDERV_BALLBOT2D_IO_PD(THETA,PHI,DTHETA,DPHI,PHI_D,DPHI_D) 
  //     This function was generated by the Symbolic Math Toolbox version 7.2.
  //     26-Aug-2017 18:20:59
  t2 = std::cos(x[1]);
  t6 = 1.0 / (t2 * t2 * 14.23452879810162 - 23.183049380911449);
  LgLfh = -t6 * (t2 * 3.772867450375327 - 0.62341284452527279);

  //  Control gains
  //  Feedfoward term
  u_ff = -(1.0 / LgLfh) * (t6 * std::sin(x[1]) * (x[3] * x[3] * t2 *
    0.39931288480891952 - 6.1156800047929263) * 35.64755694);

  //  PD term
  b_x[0] = x[1] - phi_des;
  b_x[1] = x[3] - x_d[3];
  t6 = 0.0;
  for (i0 = 0; i0 < 2; i0++) {
    t6 += (-50.0 + 35.0 * (double)i0) * b_x[i0];
  }

  t2 = 1.0 / LgLfh * t6;

  //  Full I/O controller
  *u = u_ff + t2;

  //  Bound input
  if (std::abs(*u) > 20.0) {
    if (*u < 0.0) {
      *u = -1.0;
    } else if (*u > 0.0) {
      *u = 1.0;
    } else {
      if (*u == 0.0) {
        *u = 0.0;
      }
    }

    *u *= 20.0;
  }

  //     %% ----------------------------------------------------------
  //    Pack controller relevant info
  //  -----------------------------------------------------------
  uparts->u_ff = u_ff;
  uparts->u_fb = t2;
  uparts->mu = t6;
  uparts->phi_des = phi_des;
  t6 = 0.0;
  for (i0 = 0; i0 < 4; i0++) {
    t6 += a[i0] * err[i0];
  }

  uparts->kerr = t6;
  uparts->phi_d = x_d[1];
  uparts->V_eps = 0.0;
  uparts->d1 = 0.0;
  uparts->psi0 = 0.0;
  uparts->psi1 = 0.0;
}

//
// File trailer for IO_PD_controller_c.cpp
//
// [EOF]
//

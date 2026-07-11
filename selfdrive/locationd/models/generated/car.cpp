#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3577007229449845283) {
   out_3577007229449845283[0] = delta_x[0] + nom_x[0];
   out_3577007229449845283[1] = delta_x[1] + nom_x[1];
   out_3577007229449845283[2] = delta_x[2] + nom_x[2];
   out_3577007229449845283[3] = delta_x[3] + nom_x[3];
   out_3577007229449845283[4] = delta_x[4] + nom_x[4];
   out_3577007229449845283[5] = delta_x[5] + nom_x[5];
   out_3577007229449845283[6] = delta_x[6] + nom_x[6];
   out_3577007229449845283[7] = delta_x[7] + nom_x[7];
   out_3577007229449845283[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6399722403094813471) {
   out_6399722403094813471[0] = -nom_x[0] + true_x[0];
   out_6399722403094813471[1] = -nom_x[1] + true_x[1];
   out_6399722403094813471[2] = -nom_x[2] + true_x[2];
   out_6399722403094813471[3] = -nom_x[3] + true_x[3];
   out_6399722403094813471[4] = -nom_x[4] + true_x[4];
   out_6399722403094813471[5] = -nom_x[5] + true_x[5];
   out_6399722403094813471[6] = -nom_x[6] + true_x[6];
   out_6399722403094813471[7] = -nom_x[7] + true_x[7];
   out_6399722403094813471[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7904303228569973439) {
   out_7904303228569973439[0] = 1.0;
   out_7904303228569973439[1] = 0.0;
   out_7904303228569973439[2] = 0.0;
   out_7904303228569973439[3] = 0.0;
   out_7904303228569973439[4] = 0.0;
   out_7904303228569973439[5] = 0.0;
   out_7904303228569973439[6] = 0.0;
   out_7904303228569973439[7] = 0.0;
   out_7904303228569973439[8] = 0.0;
   out_7904303228569973439[9] = 0.0;
   out_7904303228569973439[10] = 1.0;
   out_7904303228569973439[11] = 0.0;
   out_7904303228569973439[12] = 0.0;
   out_7904303228569973439[13] = 0.0;
   out_7904303228569973439[14] = 0.0;
   out_7904303228569973439[15] = 0.0;
   out_7904303228569973439[16] = 0.0;
   out_7904303228569973439[17] = 0.0;
   out_7904303228569973439[18] = 0.0;
   out_7904303228569973439[19] = 0.0;
   out_7904303228569973439[20] = 1.0;
   out_7904303228569973439[21] = 0.0;
   out_7904303228569973439[22] = 0.0;
   out_7904303228569973439[23] = 0.0;
   out_7904303228569973439[24] = 0.0;
   out_7904303228569973439[25] = 0.0;
   out_7904303228569973439[26] = 0.0;
   out_7904303228569973439[27] = 0.0;
   out_7904303228569973439[28] = 0.0;
   out_7904303228569973439[29] = 0.0;
   out_7904303228569973439[30] = 1.0;
   out_7904303228569973439[31] = 0.0;
   out_7904303228569973439[32] = 0.0;
   out_7904303228569973439[33] = 0.0;
   out_7904303228569973439[34] = 0.0;
   out_7904303228569973439[35] = 0.0;
   out_7904303228569973439[36] = 0.0;
   out_7904303228569973439[37] = 0.0;
   out_7904303228569973439[38] = 0.0;
   out_7904303228569973439[39] = 0.0;
   out_7904303228569973439[40] = 1.0;
   out_7904303228569973439[41] = 0.0;
   out_7904303228569973439[42] = 0.0;
   out_7904303228569973439[43] = 0.0;
   out_7904303228569973439[44] = 0.0;
   out_7904303228569973439[45] = 0.0;
   out_7904303228569973439[46] = 0.0;
   out_7904303228569973439[47] = 0.0;
   out_7904303228569973439[48] = 0.0;
   out_7904303228569973439[49] = 0.0;
   out_7904303228569973439[50] = 1.0;
   out_7904303228569973439[51] = 0.0;
   out_7904303228569973439[52] = 0.0;
   out_7904303228569973439[53] = 0.0;
   out_7904303228569973439[54] = 0.0;
   out_7904303228569973439[55] = 0.0;
   out_7904303228569973439[56] = 0.0;
   out_7904303228569973439[57] = 0.0;
   out_7904303228569973439[58] = 0.0;
   out_7904303228569973439[59] = 0.0;
   out_7904303228569973439[60] = 1.0;
   out_7904303228569973439[61] = 0.0;
   out_7904303228569973439[62] = 0.0;
   out_7904303228569973439[63] = 0.0;
   out_7904303228569973439[64] = 0.0;
   out_7904303228569973439[65] = 0.0;
   out_7904303228569973439[66] = 0.0;
   out_7904303228569973439[67] = 0.0;
   out_7904303228569973439[68] = 0.0;
   out_7904303228569973439[69] = 0.0;
   out_7904303228569973439[70] = 1.0;
   out_7904303228569973439[71] = 0.0;
   out_7904303228569973439[72] = 0.0;
   out_7904303228569973439[73] = 0.0;
   out_7904303228569973439[74] = 0.0;
   out_7904303228569973439[75] = 0.0;
   out_7904303228569973439[76] = 0.0;
   out_7904303228569973439[77] = 0.0;
   out_7904303228569973439[78] = 0.0;
   out_7904303228569973439[79] = 0.0;
   out_7904303228569973439[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2229936979988599717) {
   out_2229936979988599717[0] = state[0];
   out_2229936979988599717[1] = state[1];
   out_2229936979988599717[2] = state[2];
   out_2229936979988599717[3] = state[3];
   out_2229936979988599717[4] = state[4];
   out_2229936979988599717[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2229936979988599717[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2229936979988599717[7] = state[7];
   out_2229936979988599717[8] = state[8];
}
void F_fun(double *state, double dt, double *out_265299588973257661) {
   out_265299588973257661[0] = 1;
   out_265299588973257661[1] = 0;
   out_265299588973257661[2] = 0;
   out_265299588973257661[3] = 0;
   out_265299588973257661[4] = 0;
   out_265299588973257661[5] = 0;
   out_265299588973257661[6] = 0;
   out_265299588973257661[7] = 0;
   out_265299588973257661[8] = 0;
   out_265299588973257661[9] = 0;
   out_265299588973257661[10] = 1;
   out_265299588973257661[11] = 0;
   out_265299588973257661[12] = 0;
   out_265299588973257661[13] = 0;
   out_265299588973257661[14] = 0;
   out_265299588973257661[15] = 0;
   out_265299588973257661[16] = 0;
   out_265299588973257661[17] = 0;
   out_265299588973257661[18] = 0;
   out_265299588973257661[19] = 0;
   out_265299588973257661[20] = 1;
   out_265299588973257661[21] = 0;
   out_265299588973257661[22] = 0;
   out_265299588973257661[23] = 0;
   out_265299588973257661[24] = 0;
   out_265299588973257661[25] = 0;
   out_265299588973257661[26] = 0;
   out_265299588973257661[27] = 0;
   out_265299588973257661[28] = 0;
   out_265299588973257661[29] = 0;
   out_265299588973257661[30] = 1;
   out_265299588973257661[31] = 0;
   out_265299588973257661[32] = 0;
   out_265299588973257661[33] = 0;
   out_265299588973257661[34] = 0;
   out_265299588973257661[35] = 0;
   out_265299588973257661[36] = 0;
   out_265299588973257661[37] = 0;
   out_265299588973257661[38] = 0;
   out_265299588973257661[39] = 0;
   out_265299588973257661[40] = 1;
   out_265299588973257661[41] = 0;
   out_265299588973257661[42] = 0;
   out_265299588973257661[43] = 0;
   out_265299588973257661[44] = 0;
   out_265299588973257661[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_265299588973257661[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_265299588973257661[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_265299588973257661[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_265299588973257661[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_265299588973257661[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_265299588973257661[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_265299588973257661[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_265299588973257661[53] = -9.8100000000000005*dt;
   out_265299588973257661[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_265299588973257661[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_265299588973257661[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_265299588973257661[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_265299588973257661[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_265299588973257661[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_265299588973257661[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_265299588973257661[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_265299588973257661[62] = 0;
   out_265299588973257661[63] = 0;
   out_265299588973257661[64] = 0;
   out_265299588973257661[65] = 0;
   out_265299588973257661[66] = 0;
   out_265299588973257661[67] = 0;
   out_265299588973257661[68] = 0;
   out_265299588973257661[69] = 0;
   out_265299588973257661[70] = 1;
   out_265299588973257661[71] = 0;
   out_265299588973257661[72] = 0;
   out_265299588973257661[73] = 0;
   out_265299588973257661[74] = 0;
   out_265299588973257661[75] = 0;
   out_265299588973257661[76] = 0;
   out_265299588973257661[77] = 0;
   out_265299588973257661[78] = 0;
   out_265299588973257661[79] = 0;
   out_265299588973257661[80] = 1;
}
void h_25(double *state, double *unused, double *out_6366176548286495673) {
   out_6366176548286495673[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2729963794422514103) {
   out_2729963794422514103[0] = 0;
   out_2729963794422514103[1] = 0;
   out_2729963794422514103[2] = 0;
   out_2729963794422514103[3] = 0;
   out_2729963794422514103[4] = 0;
   out_2729963794422514103[5] = 0;
   out_2729963794422514103[6] = 1;
   out_2729963794422514103[7] = 0;
   out_2729963794422514103[8] = 0;
}
void h_24(double *state, double *unused, double *out_3288800728819921756) {
   out_3288800728819921756[0] = state[4];
   out_3288800728819921756[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3471068885428647323) {
   out_3471068885428647323[0] = 0;
   out_3471068885428647323[1] = 0;
   out_3471068885428647323[2] = 0;
   out_3471068885428647323[3] = 0;
   out_3471068885428647323[4] = 1;
   out_3471068885428647323[5] = 0;
   out_3471068885428647323[6] = 0;
   out_3471068885428647323[7] = 0;
   out_3471068885428647323[8] = 0;
   out_3471068885428647323[9] = 0;
   out_3471068885428647323[10] = 0;
   out_3471068885428647323[11] = 0;
   out_3471068885428647323[12] = 0;
   out_3471068885428647323[13] = 0;
   out_3471068885428647323[14] = 1;
   out_3471068885428647323[15] = 0;
   out_3471068885428647323[16] = 0;
   out_3471068885428647323[17] = 0;
}
void h_30(double *state, double *unused, double *out_4915545155657612506) {
   out_4915545155657612506[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5248296752929762730) {
   out_5248296752929762730[0] = 0;
   out_5248296752929762730[1] = 0;
   out_5248296752929762730[2] = 0;
   out_5248296752929762730[3] = 0;
   out_5248296752929762730[4] = 1;
   out_5248296752929762730[5] = 0;
   out_5248296752929762730[6] = 0;
   out_5248296752929762730[7] = 0;
   out_5248296752929762730[8] = 0;
}
void h_26(double *state, double *unused, double *out_4312229270981134989) {
   out_4312229270981134989[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1011539524451542121) {
   out_1011539524451542121[0] = 0;
   out_1011539524451542121[1] = 0;
   out_1011539524451542121[2] = 0;
   out_1011539524451542121[3] = 0;
   out_1011539524451542121[4] = 0;
   out_1011539524451542121[5] = 0;
   out_1011539524451542121[6] = 0;
   out_1011539524451542121[7] = 1;
   out_1011539524451542121[8] = 0;
}
void h_27(double *state, double *unused, double *out_2035945772564408966) {
   out_2035945772564408966[0] = state[3];
}
void H_27(double *state, double *unused, double *out_425861535478849122) {
   out_425861535478849122[0] = 0;
   out_425861535478849122[1] = 0;
   out_425861535478849122[2] = 0;
   out_425861535478849122[3] = 1;
   out_425861535478849122[4] = 0;
   out_425861535478849122[5] = 0;
   out_425861535478849122[6] = 0;
   out_425861535478849122[7] = 0;
   out_425861535478849122[8] = 0;
}
void h_29(double *state, double *unused, double *out_8529525650391093465) {
   out_8529525650391093465[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1287501191390701911) {
   out_1287501191390701911[0] = 0;
   out_1287501191390701911[1] = 1;
   out_1287501191390701911[2] = 0;
   out_1287501191390701911[3] = 0;
   out_1287501191390701911[4] = 0;
   out_1287501191390701911[5] = 0;
   out_1287501191390701911[6] = 0;
   out_1287501191390701911[7] = 0;
   out_1287501191390701911[8] = 0;
}
void h_28(double *state, double *unused, double *out_9014834141134194238) {
   out_9014834141134194238[0] = state[0];
}
void H_28(double *state, double *unused, double *out_676129080174624340) {
   out_676129080174624340[0] = 1;
   out_676129080174624340[1] = 0;
   out_676129080174624340[2] = 0;
   out_676129080174624340[3] = 0;
   out_676129080174624340[4] = 0;
   out_676129080174624340[5] = 0;
   out_676129080174624340[6] = 0;
   out_676129080174624340[7] = 0;
   out_676129080174624340[8] = 0;
}
void h_31(double *state, double *unused, double *out_1515096102467553512) {
   out_1515096102467553512[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2760609756299474531) {
   out_2760609756299474531[0] = 0;
   out_2760609756299474531[1] = 0;
   out_2760609756299474531[2] = 0;
   out_2760609756299474531[3] = 0;
   out_2760609756299474531[4] = 0;
   out_2760609756299474531[5] = 0;
   out_2760609756299474531[6] = 0;
   out_2760609756299474531[7] = 0;
   out_2760609756299474531[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_3577007229449845283) {
  err_fun(nom_x, delta_x, out_3577007229449845283);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6399722403094813471) {
  inv_err_fun(nom_x, true_x, out_6399722403094813471);
}
void car_H_mod_fun(double *state, double *out_7904303228569973439) {
  H_mod_fun(state, out_7904303228569973439);
}
void car_f_fun(double *state, double dt, double *out_2229936979988599717) {
  f_fun(state,  dt, out_2229936979988599717);
}
void car_F_fun(double *state, double dt, double *out_265299588973257661) {
  F_fun(state,  dt, out_265299588973257661);
}
void car_h_25(double *state, double *unused, double *out_6366176548286495673) {
  h_25(state, unused, out_6366176548286495673);
}
void car_H_25(double *state, double *unused, double *out_2729963794422514103) {
  H_25(state, unused, out_2729963794422514103);
}
void car_h_24(double *state, double *unused, double *out_3288800728819921756) {
  h_24(state, unused, out_3288800728819921756);
}
void car_H_24(double *state, double *unused, double *out_3471068885428647323) {
  H_24(state, unused, out_3471068885428647323);
}
void car_h_30(double *state, double *unused, double *out_4915545155657612506) {
  h_30(state, unused, out_4915545155657612506);
}
void car_H_30(double *state, double *unused, double *out_5248296752929762730) {
  H_30(state, unused, out_5248296752929762730);
}
void car_h_26(double *state, double *unused, double *out_4312229270981134989) {
  h_26(state, unused, out_4312229270981134989);
}
void car_H_26(double *state, double *unused, double *out_1011539524451542121) {
  H_26(state, unused, out_1011539524451542121);
}
void car_h_27(double *state, double *unused, double *out_2035945772564408966) {
  h_27(state, unused, out_2035945772564408966);
}
void car_H_27(double *state, double *unused, double *out_425861535478849122) {
  H_27(state, unused, out_425861535478849122);
}
void car_h_29(double *state, double *unused, double *out_8529525650391093465) {
  h_29(state, unused, out_8529525650391093465);
}
void car_H_29(double *state, double *unused, double *out_1287501191390701911) {
  H_29(state, unused, out_1287501191390701911);
}
void car_h_28(double *state, double *unused, double *out_9014834141134194238) {
  h_28(state, unused, out_9014834141134194238);
}
void car_H_28(double *state, double *unused, double *out_676129080174624340) {
  H_28(state, unused, out_676129080174624340);
}
void car_h_31(double *state, double *unused, double *out_1515096102467553512) {
  h_31(state, unused, out_1515096102467553512);
}
void car_H_31(double *state, double *unused, double *out_2760609756299474531) {
  H_31(state, unused, out_2760609756299474531);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)

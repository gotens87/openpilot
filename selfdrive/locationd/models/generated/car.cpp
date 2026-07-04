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
void err_fun(double *nom_x, double *delta_x, double *out_4786623230309029832) {
   out_4786623230309029832[0] = delta_x[0] + nom_x[0];
   out_4786623230309029832[1] = delta_x[1] + nom_x[1];
   out_4786623230309029832[2] = delta_x[2] + nom_x[2];
   out_4786623230309029832[3] = delta_x[3] + nom_x[3];
   out_4786623230309029832[4] = delta_x[4] + nom_x[4];
   out_4786623230309029832[5] = delta_x[5] + nom_x[5];
   out_4786623230309029832[6] = delta_x[6] + nom_x[6];
   out_4786623230309029832[7] = delta_x[7] + nom_x[7];
   out_4786623230309029832[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1853746942225963803) {
   out_1853746942225963803[0] = -nom_x[0] + true_x[0];
   out_1853746942225963803[1] = -nom_x[1] + true_x[1];
   out_1853746942225963803[2] = -nom_x[2] + true_x[2];
   out_1853746942225963803[3] = -nom_x[3] + true_x[3];
   out_1853746942225963803[4] = -nom_x[4] + true_x[4];
   out_1853746942225963803[5] = -nom_x[5] + true_x[5];
   out_1853746942225963803[6] = -nom_x[6] + true_x[6];
   out_1853746942225963803[7] = -nom_x[7] + true_x[7];
   out_1853746942225963803[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8577930728410900676) {
   out_8577930728410900676[0] = 1.0;
   out_8577930728410900676[1] = 0.0;
   out_8577930728410900676[2] = 0.0;
   out_8577930728410900676[3] = 0.0;
   out_8577930728410900676[4] = 0.0;
   out_8577930728410900676[5] = 0.0;
   out_8577930728410900676[6] = 0.0;
   out_8577930728410900676[7] = 0.0;
   out_8577930728410900676[8] = 0.0;
   out_8577930728410900676[9] = 0.0;
   out_8577930728410900676[10] = 1.0;
   out_8577930728410900676[11] = 0.0;
   out_8577930728410900676[12] = 0.0;
   out_8577930728410900676[13] = 0.0;
   out_8577930728410900676[14] = 0.0;
   out_8577930728410900676[15] = 0.0;
   out_8577930728410900676[16] = 0.0;
   out_8577930728410900676[17] = 0.0;
   out_8577930728410900676[18] = 0.0;
   out_8577930728410900676[19] = 0.0;
   out_8577930728410900676[20] = 1.0;
   out_8577930728410900676[21] = 0.0;
   out_8577930728410900676[22] = 0.0;
   out_8577930728410900676[23] = 0.0;
   out_8577930728410900676[24] = 0.0;
   out_8577930728410900676[25] = 0.0;
   out_8577930728410900676[26] = 0.0;
   out_8577930728410900676[27] = 0.0;
   out_8577930728410900676[28] = 0.0;
   out_8577930728410900676[29] = 0.0;
   out_8577930728410900676[30] = 1.0;
   out_8577930728410900676[31] = 0.0;
   out_8577930728410900676[32] = 0.0;
   out_8577930728410900676[33] = 0.0;
   out_8577930728410900676[34] = 0.0;
   out_8577930728410900676[35] = 0.0;
   out_8577930728410900676[36] = 0.0;
   out_8577930728410900676[37] = 0.0;
   out_8577930728410900676[38] = 0.0;
   out_8577930728410900676[39] = 0.0;
   out_8577930728410900676[40] = 1.0;
   out_8577930728410900676[41] = 0.0;
   out_8577930728410900676[42] = 0.0;
   out_8577930728410900676[43] = 0.0;
   out_8577930728410900676[44] = 0.0;
   out_8577930728410900676[45] = 0.0;
   out_8577930728410900676[46] = 0.0;
   out_8577930728410900676[47] = 0.0;
   out_8577930728410900676[48] = 0.0;
   out_8577930728410900676[49] = 0.0;
   out_8577930728410900676[50] = 1.0;
   out_8577930728410900676[51] = 0.0;
   out_8577930728410900676[52] = 0.0;
   out_8577930728410900676[53] = 0.0;
   out_8577930728410900676[54] = 0.0;
   out_8577930728410900676[55] = 0.0;
   out_8577930728410900676[56] = 0.0;
   out_8577930728410900676[57] = 0.0;
   out_8577930728410900676[58] = 0.0;
   out_8577930728410900676[59] = 0.0;
   out_8577930728410900676[60] = 1.0;
   out_8577930728410900676[61] = 0.0;
   out_8577930728410900676[62] = 0.0;
   out_8577930728410900676[63] = 0.0;
   out_8577930728410900676[64] = 0.0;
   out_8577930728410900676[65] = 0.0;
   out_8577930728410900676[66] = 0.0;
   out_8577930728410900676[67] = 0.0;
   out_8577930728410900676[68] = 0.0;
   out_8577930728410900676[69] = 0.0;
   out_8577930728410900676[70] = 1.0;
   out_8577930728410900676[71] = 0.0;
   out_8577930728410900676[72] = 0.0;
   out_8577930728410900676[73] = 0.0;
   out_8577930728410900676[74] = 0.0;
   out_8577930728410900676[75] = 0.0;
   out_8577930728410900676[76] = 0.0;
   out_8577930728410900676[77] = 0.0;
   out_8577930728410900676[78] = 0.0;
   out_8577930728410900676[79] = 0.0;
   out_8577930728410900676[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4086600449481598187) {
   out_4086600449481598187[0] = state[0];
   out_4086600449481598187[1] = state[1];
   out_4086600449481598187[2] = state[2];
   out_4086600449481598187[3] = state[3];
   out_4086600449481598187[4] = state[4];
   out_4086600449481598187[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4086600449481598187[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4086600449481598187[7] = state[7];
   out_4086600449481598187[8] = state[8];
}
void F_fun(double *state, double dt, double *out_4206537046303623167) {
   out_4206537046303623167[0] = 1;
   out_4206537046303623167[1] = 0;
   out_4206537046303623167[2] = 0;
   out_4206537046303623167[3] = 0;
   out_4206537046303623167[4] = 0;
   out_4206537046303623167[5] = 0;
   out_4206537046303623167[6] = 0;
   out_4206537046303623167[7] = 0;
   out_4206537046303623167[8] = 0;
   out_4206537046303623167[9] = 0;
   out_4206537046303623167[10] = 1;
   out_4206537046303623167[11] = 0;
   out_4206537046303623167[12] = 0;
   out_4206537046303623167[13] = 0;
   out_4206537046303623167[14] = 0;
   out_4206537046303623167[15] = 0;
   out_4206537046303623167[16] = 0;
   out_4206537046303623167[17] = 0;
   out_4206537046303623167[18] = 0;
   out_4206537046303623167[19] = 0;
   out_4206537046303623167[20] = 1;
   out_4206537046303623167[21] = 0;
   out_4206537046303623167[22] = 0;
   out_4206537046303623167[23] = 0;
   out_4206537046303623167[24] = 0;
   out_4206537046303623167[25] = 0;
   out_4206537046303623167[26] = 0;
   out_4206537046303623167[27] = 0;
   out_4206537046303623167[28] = 0;
   out_4206537046303623167[29] = 0;
   out_4206537046303623167[30] = 1;
   out_4206537046303623167[31] = 0;
   out_4206537046303623167[32] = 0;
   out_4206537046303623167[33] = 0;
   out_4206537046303623167[34] = 0;
   out_4206537046303623167[35] = 0;
   out_4206537046303623167[36] = 0;
   out_4206537046303623167[37] = 0;
   out_4206537046303623167[38] = 0;
   out_4206537046303623167[39] = 0;
   out_4206537046303623167[40] = 1;
   out_4206537046303623167[41] = 0;
   out_4206537046303623167[42] = 0;
   out_4206537046303623167[43] = 0;
   out_4206537046303623167[44] = 0;
   out_4206537046303623167[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4206537046303623167[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4206537046303623167[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4206537046303623167[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4206537046303623167[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4206537046303623167[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4206537046303623167[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4206537046303623167[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4206537046303623167[53] = -9.8100000000000005*dt;
   out_4206537046303623167[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4206537046303623167[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4206537046303623167[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4206537046303623167[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4206537046303623167[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4206537046303623167[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4206537046303623167[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4206537046303623167[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4206537046303623167[62] = 0;
   out_4206537046303623167[63] = 0;
   out_4206537046303623167[64] = 0;
   out_4206537046303623167[65] = 0;
   out_4206537046303623167[66] = 0;
   out_4206537046303623167[67] = 0;
   out_4206537046303623167[68] = 0;
   out_4206537046303623167[69] = 0;
   out_4206537046303623167[70] = 1;
   out_4206537046303623167[71] = 0;
   out_4206537046303623167[72] = 0;
   out_4206537046303623167[73] = 0;
   out_4206537046303623167[74] = 0;
   out_4206537046303623167[75] = 0;
   out_4206537046303623167[76] = 0;
   out_4206537046303623167[77] = 0;
   out_4206537046303623167[78] = 0;
   out_4206537046303623167[79] = 0;
   out_4206537046303623167[80] = 1;
}
void h_25(double *state, double *unused, double *out_1336693333340347965) {
   out_1336693333340347965[0] = state[6];
}
void H_25(double *state, double *unused, double *out_675625119386413703) {
   out_675625119386413703[0] = 0;
   out_675625119386413703[1] = 0;
   out_675625119386413703[2] = 0;
   out_675625119386413703[3] = 0;
   out_675625119386413703[4] = 0;
   out_675625119386413703[5] = 0;
   out_675625119386413703[6] = 1;
   out_675625119386413703[7] = 0;
   out_675625119386413703[8] = 0;
}
void h_24(double *state, double *unused, double *out_14163058279738692) {
   out_14163058279738692[0] = state[4];
   out_14163058279738692[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5949119402299426297) {
   out_5949119402299426297[0] = 0;
   out_5949119402299426297[1] = 0;
   out_5949119402299426297[2] = 0;
   out_5949119402299426297[3] = 0;
   out_5949119402299426297[4] = 1;
   out_5949119402299426297[5] = 0;
   out_5949119402299426297[6] = 0;
   out_5949119402299426297[7] = 0;
   out_5949119402299426297[8] = 0;
   out_5949119402299426297[9] = 0;
   out_5949119402299426297[10] = 0;
   out_5949119402299426297[11] = 0;
   out_5949119402299426297[12] = 0;
   out_5949119402299426297[13] = 0;
   out_5949119402299426297[14] = 1;
   out_5949119402299426297[15] = 0;
   out_5949119402299426297[16] = 0;
   out_5949119402299426297[17] = 0;
}
void h_30(double *state, double *unused, double *out_65271124810109003) {
   out_65271124810109003[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3193958077893662330) {
   out_3193958077893662330[0] = 0;
   out_3193958077893662330[1] = 0;
   out_3193958077893662330[2] = 0;
   out_3193958077893662330[3] = 0;
   out_3193958077893662330[4] = 1;
   out_3193958077893662330[5] = 0;
   out_3193958077893662330[6] = 0;
   out_3193958077893662330[7] = 0;
   out_3193958077893662330[8] = 0;
}
void h_26(double *state, double *unused, double *out_8783108044678801329) {
   out_8783108044678801329[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3065878199487642521) {
   out_3065878199487642521[0] = 0;
   out_3065878199487642521[1] = 0;
   out_3065878199487642521[2] = 0;
   out_3065878199487642521[3] = 0;
   out_3065878199487642521[4] = 0;
   out_3065878199487642521[5] = 0;
   out_3065878199487642521[6] = 0;
   out_3065878199487642521[7] = 1;
   out_3065878199487642521[8] = 0;
}
void h_27(double *state, double *unused, double *out_6338366601001193610) {
   out_6338366601001193610[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5417552149077605547) {
   out_5417552149077605547[0] = 0;
   out_5417552149077605547[1] = 0;
   out_5417552149077605547[2] = 0;
   out_5417552149077605547[3] = 1;
   out_5417552149077605547[4] = 0;
   out_5417552149077605547[5] = 0;
   out_5417552149077605547[6] = 0;
   out_5417552149077605547[7] = 0;
   out_5417552149077605547[8] = 0;
}
void h_29(double *state, double *unused, double *out_8470927944454335656) {
   out_8470927944454335656[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3704189422208054514) {
   out_3704189422208054514[0] = 0;
   out_3704189422208054514[1] = 1;
   out_3704189422208054514[2] = 0;
   out_3704189422208054514[3] = 0;
   out_3704189422208054514[4] = 0;
   out_3704189422208054514[5] = 0;
   out_3704189422208054514[6] = 0;
   out_3704189422208054514[7] = 0;
   out_3704189422208054514[8] = 0;
}
void h_28(double *state, double *unused, double *out_3717698227664568318) {
   out_3717698227664568318[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1378209594861476060) {
   out_1378209594861476060[0] = 1;
   out_1378209594861476060[1] = 0;
   out_1378209594861476060[2] = 0;
   out_1378209594861476060[3] = 0;
   out_1378209594861476060[4] = 0;
   out_1378209594861476060[5] = 0;
   out_1378209594861476060[6] = 0;
   out_1378209594861476060[7] = 0;
   out_1378209594861476060[8] = 0;
}
void h_31(double *state, double *unused, double *out_3977914715348637242) {
   out_3977914715348637242[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3692086301720993997) {
   out_3692086301720993997[0] = 0;
   out_3692086301720993997[1] = 0;
   out_3692086301720993997[2] = 0;
   out_3692086301720993997[3] = 0;
   out_3692086301720993997[4] = 0;
   out_3692086301720993997[5] = 0;
   out_3692086301720993997[6] = 0;
   out_3692086301720993997[7] = 0;
   out_3692086301720993997[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4786623230309029832) {
  err_fun(nom_x, delta_x, out_4786623230309029832);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1853746942225963803) {
  inv_err_fun(nom_x, true_x, out_1853746942225963803);
}
void car_H_mod_fun(double *state, double *out_8577930728410900676) {
  H_mod_fun(state, out_8577930728410900676);
}
void car_f_fun(double *state, double dt, double *out_4086600449481598187) {
  f_fun(state,  dt, out_4086600449481598187);
}
void car_F_fun(double *state, double dt, double *out_4206537046303623167) {
  F_fun(state,  dt, out_4206537046303623167);
}
void car_h_25(double *state, double *unused, double *out_1336693333340347965) {
  h_25(state, unused, out_1336693333340347965);
}
void car_H_25(double *state, double *unused, double *out_675625119386413703) {
  H_25(state, unused, out_675625119386413703);
}
void car_h_24(double *state, double *unused, double *out_14163058279738692) {
  h_24(state, unused, out_14163058279738692);
}
void car_H_24(double *state, double *unused, double *out_5949119402299426297) {
  H_24(state, unused, out_5949119402299426297);
}
void car_h_30(double *state, double *unused, double *out_65271124810109003) {
  h_30(state, unused, out_65271124810109003);
}
void car_H_30(double *state, double *unused, double *out_3193958077893662330) {
  H_30(state, unused, out_3193958077893662330);
}
void car_h_26(double *state, double *unused, double *out_8783108044678801329) {
  h_26(state, unused, out_8783108044678801329);
}
void car_H_26(double *state, double *unused, double *out_3065878199487642521) {
  H_26(state, unused, out_3065878199487642521);
}
void car_h_27(double *state, double *unused, double *out_6338366601001193610) {
  h_27(state, unused, out_6338366601001193610);
}
void car_H_27(double *state, double *unused, double *out_5417552149077605547) {
  H_27(state, unused, out_5417552149077605547);
}
void car_h_29(double *state, double *unused, double *out_8470927944454335656) {
  h_29(state, unused, out_8470927944454335656);
}
void car_H_29(double *state, double *unused, double *out_3704189422208054514) {
  H_29(state, unused, out_3704189422208054514);
}
void car_h_28(double *state, double *unused, double *out_3717698227664568318) {
  h_28(state, unused, out_3717698227664568318);
}
void car_H_28(double *state, double *unused, double *out_1378209594861476060) {
  H_28(state, unused, out_1378209594861476060);
}
void car_h_31(double *state, double *unused, double *out_3977914715348637242) {
  h_31(state, unused, out_3977914715348637242);
}
void car_H_31(double *state, double *unused, double *out_3692086301720993997) {
  H_31(state, unused, out_3692086301720993997);
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

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
void err_fun(double *nom_x, double *delta_x, double *out_5469727734415202139) {
   out_5469727734415202139[0] = delta_x[0] + nom_x[0];
   out_5469727734415202139[1] = delta_x[1] + nom_x[1];
   out_5469727734415202139[2] = delta_x[2] + nom_x[2];
   out_5469727734415202139[3] = delta_x[3] + nom_x[3];
   out_5469727734415202139[4] = delta_x[4] + nom_x[4];
   out_5469727734415202139[5] = delta_x[5] + nom_x[5];
   out_5469727734415202139[6] = delta_x[6] + nom_x[6];
   out_5469727734415202139[7] = delta_x[7] + nom_x[7];
   out_5469727734415202139[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_9043415007849521510) {
   out_9043415007849521510[0] = -nom_x[0] + true_x[0];
   out_9043415007849521510[1] = -nom_x[1] + true_x[1];
   out_9043415007849521510[2] = -nom_x[2] + true_x[2];
   out_9043415007849521510[3] = -nom_x[3] + true_x[3];
   out_9043415007849521510[4] = -nom_x[4] + true_x[4];
   out_9043415007849521510[5] = -nom_x[5] + true_x[5];
   out_9043415007849521510[6] = -nom_x[6] + true_x[6];
   out_9043415007849521510[7] = -nom_x[7] + true_x[7];
   out_9043415007849521510[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3975286947566256014) {
   out_3975286947566256014[0] = 1.0;
   out_3975286947566256014[1] = 0.0;
   out_3975286947566256014[2] = 0.0;
   out_3975286947566256014[3] = 0.0;
   out_3975286947566256014[4] = 0.0;
   out_3975286947566256014[5] = 0.0;
   out_3975286947566256014[6] = 0.0;
   out_3975286947566256014[7] = 0.0;
   out_3975286947566256014[8] = 0.0;
   out_3975286947566256014[9] = 0.0;
   out_3975286947566256014[10] = 1.0;
   out_3975286947566256014[11] = 0.0;
   out_3975286947566256014[12] = 0.0;
   out_3975286947566256014[13] = 0.0;
   out_3975286947566256014[14] = 0.0;
   out_3975286947566256014[15] = 0.0;
   out_3975286947566256014[16] = 0.0;
   out_3975286947566256014[17] = 0.0;
   out_3975286947566256014[18] = 0.0;
   out_3975286947566256014[19] = 0.0;
   out_3975286947566256014[20] = 1.0;
   out_3975286947566256014[21] = 0.0;
   out_3975286947566256014[22] = 0.0;
   out_3975286947566256014[23] = 0.0;
   out_3975286947566256014[24] = 0.0;
   out_3975286947566256014[25] = 0.0;
   out_3975286947566256014[26] = 0.0;
   out_3975286947566256014[27] = 0.0;
   out_3975286947566256014[28] = 0.0;
   out_3975286947566256014[29] = 0.0;
   out_3975286947566256014[30] = 1.0;
   out_3975286947566256014[31] = 0.0;
   out_3975286947566256014[32] = 0.0;
   out_3975286947566256014[33] = 0.0;
   out_3975286947566256014[34] = 0.0;
   out_3975286947566256014[35] = 0.0;
   out_3975286947566256014[36] = 0.0;
   out_3975286947566256014[37] = 0.0;
   out_3975286947566256014[38] = 0.0;
   out_3975286947566256014[39] = 0.0;
   out_3975286947566256014[40] = 1.0;
   out_3975286947566256014[41] = 0.0;
   out_3975286947566256014[42] = 0.0;
   out_3975286947566256014[43] = 0.0;
   out_3975286947566256014[44] = 0.0;
   out_3975286947566256014[45] = 0.0;
   out_3975286947566256014[46] = 0.0;
   out_3975286947566256014[47] = 0.0;
   out_3975286947566256014[48] = 0.0;
   out_3975286947566256014[49] = 0.0;
   out_3975286947566256014[50] = 1.0;
   out_3975286947566256014[51] = 0.0;
   out_3975286947566256014[52] = 0.0;
   out_3975286947566256014[53] = 0.0;
   out_3975286947566256014[54] = 0.0;
   out_3975286947566256014[55] = 0.0;
   out_3975286947566256014[56] = 0.0;
   out_3975286947566256014[57] = 0.0;
   out_3975286947566256014[58] = 0.0;
   out_3975286947566256014[59] = 0.0;
   out_3975286947566256014[60] = 1.0;
   out_3975286947566256014[61] = 0.0;
   out_3975286947566256014[62] = 0.0;
   out_3975286947566256014[63] = 0.0;
   out_3975286947566256014[64] = 0.0;
   out_3975286947566256014[65] = 0.0;
   out_3975286947566256014[66] = 0.0;
   out_3975286947566256014[67] = 0.0;
   out_3975286947566256014[68] = 0.0;
   out_3975286947566256014[69] = 0.0;
   out_3975286947566256014[70] = 1.0;
   out_3975286947566256014[71] = 0.0;
   out_3975286947566256014[72] = 0.0;
   out_3975286947566256014[73] = 0.0;
   out_3975286947566256014[74] = 0.0;
   out_3975286947566256014[75] = 0.0;
   out_3975286947566256014[76] = 0.0;
   out_3975286947566256014[77] = 0.0;
   out_3975286947566256014[78] = 0.0;
   out_3975286947566256014[79] = 0.0;
   out_3975286947566256014[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1399233721380390552) {
   out_1399233721380390552[0] = state[0];
   out_1399233721380390552[1] = state[1];
   out_1399233721380390552[2] = state[2];
   out_1399233721380390552[3] = state[3];
   out_1399233721380390552[4] = state[4];
   out_1399233721380390552[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1399233721380390552[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1399233721380390552[7] = state[7];
   out_1399233721380390552[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1547855628850571031) {
   out_1547855628850571031[0] = 1;
   out_1547855628850571031[1] = 0;
   out_1547855628850571031[2] = 0;
   out_1547855628850571031[3] = 0;
   out_1547855628850571031[4] = 0;
   out_1547855628850571031[5] = 0;
   out_1547855628850571031[6] = 0;
   out_1547855628850571031[7] = 0;
   out_1547855628850571031[8] = 0;
   out_1547855628850571031[9] = 0;
   out_1547855628850571031[10] = 1;
   out_1547855628850571031[11] = 0;
   out_1547855628850571031[12] = 0;
   out_1547855628850571031[13] = 0;
   out_1547855628850571031[14] = 0;
   out_1547855628850571031[15] = 0;
   out_1547855628850571031[16] = 0;
   out_1547855628850571031[17] = 0;
   out_1547855628850571031[18] = 0;
   out_1547855628850571031[19] = 0;
   out_1547855628850571031[20] = 1;
   out_1547855628850571031[21] = 0;
   out_1547855628850571031[22] = 0;
   out_1547855628850571031[23] = 0;
   out_1547855628850571031[24] = 0;
   out_1547855628850571031[25] = 0;
   out_1547855628850571031[26] = 0;
   out_1547855628850571031[27] = 0;
   out_1547855628850571031[28] = 0;
   out_1547855628850571031[29] = 0;
   out_1547855628850571031[30] = 1;
   out_1547855628850571031[31] = 0;
   out_1547855628850571031[32] = 0;
   out_1547855628850571031[33] = 0;
   out_1547855628850571031[34] = 0;
   out_1547855628850571031[35] = 0;
   out_1547855628850571031[36] = 0;
   out_1547855628850571031[37] = 0;
   out_1547855628850571031[38] = 0;
   out_1547855628850571031[39] = 0;
   out_1547855628850571031[40] = 1;
   out_1547855628850571031[41] = 0;
   out_1547855628850571031[42] = 0;
   out_1547855628850571031[43] = 0;
   out_1547855628850571031[44] = 0;
   out_1547855628850571031[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1547855628850571031[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1547855628850571031[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1547855628850571031[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1547855628850571031[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1547855628850571031[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1547855628850571031[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1547855628850571031[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1547855628850571031[53] = -9.8100000000000005*dt;
   out_1547855628850571031[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1547855628850571031[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1547855628850571031[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1547855628850571031[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1547855628850571031[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1547855628850571031[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1547855628850571031[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1547855628850571031[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1547855628850571031[62] = 0;
   out_1547855628850571031[63] = 0;
   out_1547855628850571031[64] = 0;
   out_1547855628850571031[65] = 0;
   out_1547855628850571031[66] = 0;
   out_1547855628850571031[67] = 0;
   out_1547855628850571031[68] = 0;
   out_1547855628850571031[69] = 0;
   out_1547855628850571031[70] = 1;
   out_1547855628850571031[71] = 0;
   out_1547855628850571031[72] = 0;
   out_1547855628850571031[73] = 0;
   out_1547855628850571031[74] = 0;
   out_1547855628850571031[75] = 0;
   out_1547855628850571031[76] = 0;
   out_1547855628850571031[77] = 0;
   out_1547855628850571031[78] = 0;
   out_1547855628850571031[79] = 0;
   out_1547855628850571031[80] = 1;
}
void h_25(double *state, double *unused, double *out_4999223901660539793) {
   out_4999223901660539793[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5217901278345981223) {
   out_5217901278345981223[0] = 0;
   out_5217901278345981223[1] = 0;
   out_5217901278345981223[2] = 0;
   out_5217901278345981223[3] = 0;
   out_5217901278345981223[4] = 0;
   out_5217901278345981223[5] = 0;
   out_5217901278345981223[6] = 1;
   out_5217901278345981223[7] = 0;
   out_5217901278345981223[8] = 0;
}
void h_24(double *state, double *unused, double *out_6882293632162622150) {
   out_6882293632162622150[0] = state[4];
   out_6882293632162622150[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1436304179762204534) {
   out_1436304179762204534[0] = 0;
   out_1436304179762204534[1] = 0;
   out_1436304179762204534[2] = 0;
   out_1436304179762204534[3] = 0;
   out_1436304179762204534[4] = 1;
   out_1436304179762204534[5] = 0;
   out_1436304179762204534[6] = 0;
   out_1436304179762204534[7] = 0;
   out_1436304179762204534[8] = 0;
   out_1436304179762204534[9] = 0;
   out_1436304179762204534[10] = 0;
   out_1436304179762204534[11] = 0;
   out_1436304179762204534[12] = 0;
   out_1436304179762204534[13] = 0;
   out_1436304179762204534[14] = 1;
   out_1436304179762204534[15] = 0;
   out_1436304179762204534[16] = 0;
   out_1436304179762204534[17] = 0;
}
void h_30(double *state, double *unused, double *out_4724029839376033904) {
   out_4724029839376033904[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8701146465235962195) {
   out_8701146465235962195[0] = 0;
   out_8701146465235962195[1] = 0;
   out_8701146465235962195[2] = 0;
   out_8701146465235962195[3] = 0;
   out_8701146465235962195[4] = 1;
   out_8701146465235962195[5] = 0;
   out_8701146465235962195[6] = 0;
   out_8701146465235962195[7] = 0;
   out_8701146465235962195[8] = 0;
}
void h_26(double *state, double *unused, double *out_3556286038299892167) {
   out_3556286038299892167[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8959404597220037447) {
   out_8959404597220037447[0] = 0;
   out_8959404597220037447[1] = 0;
   out_8959404597220037447[2] = 0;
   out_8959404597220037447[3] = 0;
   out_8959404597220037447[4] = 0;
   out_8959404597220037447[5] = 0;
   out_8959404597220037447[6] = 0;
   out_8959404597220037447[7] = 1;
   out_8959404597220037447[8] = 0;
}
void h_27(double *state, double *unused, double *out_356357552238636072) {
   out_356357552238636072[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7522003537289646204) {
   out_7522003537289646204[0] = 0;
   out_7522003537289646204[1] = 0;
   out_7522003537289646204[2] = 0;
   out_7522003537289646204[3] = 1;
   out_7522003537289646204[4] = 0;
   out_7522003537289646204[5] = 0;
   out_7522003537289646204[6] = 0;
   out_7522003537289646204[7] = 0;
   out_7522003537289646204[8] = 0;
}
void h_29(double *state, double *unused, double *out_277643428284811741) {
   out_277643428284811741[0] = state[1];
}
void H_29(double *state, double *unused, double *out_9211377809550354379) {
   out_9211377809550354379[0] = 0;
   out_9211377809550354379[1] = 1;
   out_9211377809550354379[2] = 0;
   out_9211377809550354379[3] = 0;
   out_9211377809550354379[4] = 0;
   out_9211377809550354379[5] = 0;
   out_9211377809550354379[6] = 0;
   out_9211377809550354379[7] = 0;
   out_9211377809550354379[8] = 0;
}
void h_28(double *state, double *unused, double *out_5358030819899481717) {
   out_5358030819899481717[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4128978792480823805) {
   out_4128978792480823805[0] = 1;
   out_4128978792480823805[1] = 0;
   out_4128978792480823805[2] = 0;
   out_4128978792480823805[3] = 0;
   out_4128978792480823805[4] = 0;
   out_4128978792480823805[5] = 0;
   out_4128978792480823805[6] = 0;
   out_4128978792480823805[7] = 0;
   out_4128978792480823805[8] = 0;
}
void h_31(double *state, double *unused, double *out_2893715593223983566) {
   out_2893715593223983566[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8861131374256162693) {
   out_8861131374256162693[0] = 0;
   out_8861131374256162693[1] = 0;
   out_8861131374256162693[2] = 0;
   out_8861131374256162693[3] = 0;
   out_8861131374256162693[4] = 0;
   out_8861131374256162693[5] = 0;
   out_8861131374256162693[6] = 0;
   out_8861131374256162693[7] = 0;
   out_8861131374256162693[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5469727734415202139) {
  err_fun(nom_x, delta_x, out_5469727734415202139);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9043415007849521510) {
  inv_err_fun(nom_x, true_x, out_9043415007849521510);
}
void car_H_mod_fun(double *state, double *out_3975286947566256014) {
  H_mod_fun(state, out_3975286947566256014);
}
void car_f_fun(double *state, double dt, double *out_1399233721380390552) {
  f_fun(state,  dt, out_1399233721380390552);
}
void car_F_fun(double *state, double dt, double *out_1547855628850571031) {
  F_fun(state,  dt, out_1547855628850571031);
}
void car_h_25(double *state, double *unused, double *out_4999223901660539793) {
  h_25(state, unused, out_4999223901660539793);
}
void car_H_25(double *state, double *unused, double *out_5217901278345981223) {
  H_25(state, unused, out_5217901278345981223);
}
void car_h_24(double *state, double *unused, double *out_6882293632162622150) {
  h_24(state, unused, out_6882293632162622150);
}
void car_H_24(double *state, double *unused, double *out_1436304179762204534) {
  H_24(state, unused, out_1436304179762204534);
}
void car_h_30(double *state, double *unused, double *out_4724029839376033904) {
  h_30(state, unused, out_4724029839376033904);
}
void car_H_30(double *state, double *unused, double *out_8701146465235962195) {
  H_30(state, unused, out_8701146465235962195);
}
void car_h_26(double *state, double *unused, double *out_3556286038299892167) {
  h_26(state, unused, out_3556286038299892167);
}
void car_H_26(double *state, double *unused, double *out_8959404597220037447) {
  H_26(state, unused, out_8959404597220037447);
}
void car_h_27(double *state, double *unused, double *out_356357552238636072) {
  h_27(state, unused, out_356357552238636072);
}
void car_H_27(double *state, double *unused, double *out_7522003537289646204) {
  H_27(state, unused, out_7522003537289646204);
}
void car_h_29(double *state, double *unused, double *out_277643428284811741) {
  h_29(state, unused, out_277643428284811741);
}
void car_H_29(double *state, double *unused, double *out_9211377809550354379) {
  H_29(state, unused, out_9211377809550354379);
}
void car_h_28(double *state, double *unused, double *out_5358030819899481717) {
  h_28(state, unused, out_5358030819899481717);
}
void car_H_28(double *state, double *unused, double *out_4128978792480823805) {
  H_28(state, unused, out_4128978792480823805);
}
void car_h_31(double *state, double *unused, double *out_2893715593223983566) {
  h_31(state, unused, out_2893715593223983566);
}
void car_H_31(double *state, double *unused, double *out_8861131374256162693) {
  H_31(state, unused, out_8861131374256162693);
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

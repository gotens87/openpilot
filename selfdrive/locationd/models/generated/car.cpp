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
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7312882027617302654) {
   out_7312882027617302654[0] = delta_x[0] + nom_x[0];
   out_7312882027617302654[1] = delta_x[1] + nom_x[1];
   out_7312882027617302654[2] = delta_x[2] + nom_x[2];
   out_7312882027617302654[3] = delta_x[3] + nom_x[3];
   out_7312882027617302654[4] = delta_x[4] + nom_x[4];
   out_7312882027617302654[5] = delta_x[5] + nom_x[5];
   out_7312882027617302654[6] = delta_x[6] + nom_x[6];
   out_7312882027617302654[7] = delta_x[7] + nom_x[7];
   out_7312882027617302654[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7611434335802104229) {
   out_7611434335802104229[0] = -nom_x[0] + true_x[0];
   out_7611434335802104229[1] = -nom_x[1] + true_x[1];
   out_7611434335802104229[2] = -nom_x[2] + true_x[2];
   out_7611434335802104229[3] = -nom_x[3] + true_x[3];
   out_7611434335802104229[4] = -nom_x[4] + true_x[4];
   out_7611434335802104229[5] = -nom_x[5] + true_x[5];
   out_7611434335802104229[6] = -nom_x[6] + true_x[6];
   out_7611434335802104229[7] = -nom_x[7] + true_x[7];
   out_7611434335802104229[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1755367647345397347) {
   out_1755367647345397347[0] = 1.0;
   out_1755367647345397347[1] = 0.0;
   out_1755367647345397347[2] = 0.0;
   out_1755367647345397347[3] = 0.0;
   out_1755367647345397347[4] = 0.0;
   out_1755367647345397347[5] = 0.0;
   out_1755367647345397347[6] = 0.0;
   out_1755367647345397347[7] = 0.0;
   out_1755367647345397347[8] = 0.0;
   out_1755367647345397347[9] = 0.0;
   out_1755367647345397347[10] = 1.0;
   out_1755367647345397347[11] = 0.0;
   out_1755367647345397347[12] = 0.0;
   out_1755367647345397347[13] = 0.0;
   out_1755367647345397347[14] = 0.0;
   out_1755367647345397347[15] = 0.0;
   out_1755367647345397347[16] = 0.0;
   out_1755367647345397347[17] = 0.0;
   out_1755367647345397347[18] = 0.0;
   out_1755367647345397347[19] = 0.0;
   out_1755367647345397347[20] = 1.0;
   out_1755367647345397347[21] = 0.0;
   out_1755367647345397347[22] = 0.0;
   out_1755367647345397347[23] = 0.0;
   out_1755367647345397347[24] = 0.0;
   out_1755367647345397347[25] = 0.0;
   out_1755367647345397347[26] = 0.0;
   out_1755367647345397347[27] = 0.0;
   out_1755367647345397347[28] = 0.0;
   out_1755367647345397347[29] = 0.0;
   out_1755367647345397347[30] = 1.0;
   out_1755367647345397347[31] = 0.0;
   out_1755367647345397347[32] = 0.0;
   out_1755367647345397347[33] = 0.0;
   out_1755367647345397347[34] = 0.0;
   out_1755367647345397347[35] = 0.0;
   out_1755367647345397347[36] = 0.0;
   out_1755367647345397347[37] = 0.0;
   out_1755367647345397347[38] = 0.0;
   out_1755367647345397347[39] = 0.0;
   out_1755367647345397347[40] = 1.0;
   out_1755367647345397347[41] = 0.0;
   out_1755367647345397347[42] = 0.0;
   out_1755367647345397347[43] = 0.0;
   out_1755367647345397347[44] = 0.0;
   out_1755367647345397347[45] = 0.0;
   out_1755367647345397347[46] = 0.0;
   out_1755367647345397347[47] = 0.0;
   out_1755367647345397347[48] = 0.0;
   out_1755367647345397347[49] = 0.0;
   out_1755367647345397347[50] = 1.0;
   out_1755367647345397347[51] = 0.0;
   out_1755367647345397347[52] = 0.0;
   out_1755367647345397347[53] = 0.0;
   out_1755367647345397347[54] = 0.0;
   out_1755367647345397347[55] = 0.0;
   out_1755367647345397347[56] = 0.0;
   out_1755367647345397347[57] = 0.0;
   out_1755367647345397347[58] = 0.0;
   out_1755367647345397347[59] = 0.0;
   out_1755367647345397347[60] = 1.0;
   out_1755367647345397347[61] = 0.0;
   out_1755367647345397347[62] = 0.0;
   out_1755367647345397347[63] = 0.0;
   out_1755367647345397347[64] = 0.0;
   out_1755367647345397347[65] = 0.0;
   out_1755367647345397347[66] = 0.0;
   out_1755367647345397347[67] = 0.0;
   out_1755367647345397347[68] = 0.0;
   out_1755367647345397347[69] = 0.0;
   out_1755367647345397347[70] = 1.0;
   out_1755367647345397347[71] = 0.0;
   out_1755367647345397347[72] = 0.0;
   out_1755367647345397347[73] = 0.0;
   out_1755367647345397347[74] = 0.0;
   out_1755367647345397347[75] = 0.0;
   out_1755367647345397347[76] = 0.0;
   out_1755367647345397347[77] = 0.0;
   out_1755367647345397347[78] = 0.0;
   out_1755367647345397347[79] = 0.0;
   out_1755367647345397347[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3297305622544450583) {
   out_3297305622544450583[0] = state[0];
   out_3297305622544450583[1] = state[1];
   out_3297305622544450583[2] = state[2];
   out_3297305622544450583[3] = state[3];
   out_3297305622544450583[4] = state[4];
   out_3297305622544450583[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3297305622544450583[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3297305622544450583[7] = state[7];
   out_3297305622544450583[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3243986764759865323) {
   out_3243986764759865323[0] = 1;
   out_3243986764759865323[1] = 0;
   out_3243986764759865323[2] = 0;
   out_3243986764759865323[3] = 0;
   out_3243986764759865323[4] = 0;
   out_3243986764759865323[5] = 0;
   out_3243986764759865323[6] = 0;
   out_3243986764759865323[7] = 0;
   out_3243986764759865323[8] = 0;
   out_3243986764759865323[9] = 0;
   out_3243986764759865323[10] = 1;
   out_3243986764759865323[11] = 0;
   out_3243986764759865323[12] = 0;
   out_3243986764759865323[13] = 0;
   out_3243986764759865323[14] = 0;
   out_3243986764759865323[15] = 0;
   out_3243986764759865323[16] = 0;
   out_3243986764759865323[17] = 0;
   out_3243986764759865323[18] = 0;
   out_3243986764759865323[19] = 0;
   out_3243986764759865323[20] = 1;
   out_3243986764759865323[21] = 0;
   out_3243986764759865323[22] = 0;
   out_3243986764759865323[23] = 0;
   out_3243986764759865323[24] = 0;
   out_3243986764759865323[25] = 0;
   out_3243986764759865323[26] = 0;
   out_3243986764759865323[27] = 0;
   out_3243986764759865323[28] = 0;
   out_3243986764759865323[29] = 0;
   out_3243986764759865323[30] = 1;
   out_3243986764759865323[31] = 0;
   out_3243986764759865323[32] = 0;
   out_3243986764759865323[33] = 0;
   out_3243986764759865323[34] = 0;
   out_3243986764759865323[35] = 0;
   out_3243986764759865323[36] = 0;
   out_3243986764759865323[37] = 0;
   out_3243986764759865323[38] = 0;
   out_3243986764759865323[39] = 0;
   out_3243986764759865323[40] = 1;
   out_3243986764759865323[41] = 0;
   out_3243986764759865323[42] = 0;
   out_3243986764759865323[43] = 0;
   out_3243986764759865323[44] = 0;
   out_3243986764759865323[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3243986764759865323[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3243986764759865323[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3243986764759865323[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3243986764759865323[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3243986764759865323[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3243986764759865323[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3243986764759865323[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3243986764759865323[53] = -9.8000000000000007*dt;
   out_3243986764759865323[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3243986764759865323[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3243986764759865323[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3243986764759865323[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3243986764759865323[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3243986764759865323[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3243986764759865323[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3243986764759865323[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3243986764759865323[62] = 0;
   out_3243986764759865323[63] = 0;
   out_3243986764759865323[64] = 0;
   out_3243986764759865323[65] = 0;
   out_3243986764759865323[66] = 0;
   out_3243986764759865323[67] = 0;
   out_3243986764759865323[68] = 0;
   out_3243986764759865323[69] = 0;
   out_3243986764759865323[70] = 1;
   out_3243986764759865323[71] = 0;
   out_3243986764759865323[72] = 0;
   out_3243986764759865323[73] = 0;
   out_3243986764759865323[74] = 0;
   out_3243986764759865323[75] = 0;
   out_3243986764759865323[76] = 0;
   out_3243986764759865323[77] = 0;
   out_3243986764759865323[78] = 0;
   out_3243986764759865323[79] = 0;
   out_3243986764759865323[80] = 1;
}
void h_25(double *state, double *unused, double *out_5049182664059894771) {
   out_5049182664059894771[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3148324177839267493) {
   out_3148324177839267493[0] = 0;
   out_3148324177839267493[1] = 0;
   out_3148324177839267493[2] = 0;
   out_3148324177839267493[3] = 0;
   out_3148324177839267493[4] = 0;
   out_3148324177839267493[5] = 0;
   out_3148324177839267493[6] = 1;
   out_3148324177839267493[7] = 0;
   out_3148324177839267493[8] = 0;
}
void h_24(double *state, double *unused, double *out_7961352067590225679) {
   out_7961352067590225679[0] = state[4];
   out_7961352067590225679[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8722848089278766022) {
   out_8722848089278766022[0] = 0;
   out_8722848089278766022[1] = 0;
   out_8722848089278766022[2] = 0;
   out_8722848089278766022[3] = 0;
   out_8722848089278766022[4] = 1;
   out_8722848089278766022[5] = 0;
   out_8722848089278766022[6] = 0;
   out_8722848089278766022[7] = 0;
   out_8722848089278766022[8] = 0;
   out_8722848089278766022[9] = 0;
   out_8722848089278766022[10] = 0;
   out_8722848089278766022[11] = 0;
   out_8722848089278766022[12] = 0;
   out_8722848089278766022[13] = 0;
   out_8722848089278766022[14] = 1;
   out_8722848089278766022[15] = 0;
   out_8722848089278766022[16] = 0;
   out_8722848089278766022[17] = 0;
}
void h_30(double *state, double *unused, double *out_6226913174035872437) {
   out_6226913174035872437[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5666657136346516120) {
   out_5666657136346516120[0] = 0;
   out_5666657136346516120[1] = 0;
   out_5666657136346516120[2] = 0;
   out_5666657136346516120[3] = 0;
   out_5666657136346516120[4] = 1;
   out_5666657136346516120[5] = 0;
   out_5666657136346516120[6] = 0;
   out_5666657136346516120[7] = 0;
   out_5666657136346516120[8] = 0;
}
void h_26(double *state, double *unused, double *out_7794469984375447646) {
   out_7794469984375447646[0] = state[7];
}
void H_26(double *state, double *unused, double *out_593179141034788731) {
   out_593179141034788731[0] = 0;
   out_593179141034788731[1] = 0;
   out_593179141034788731[2] = 0;
   out_593179141034788731[3] = 0;
   out_593179141034788731[4] = 0;
   out_593179141034788731[5] = 0;
   out_593179141034788731[6] = 0;
   out_593179141034788731[7] = 1;
   out_593179141034788731[8] = 0;
}
void h_27(double *state, double *unused, double *out_7912477378308824390) {
   out_7912477378308824390[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7890251207530459337) {
   out_7890251207530459337[0] = 0;
   out_7890251207530459337[1] = 0;
   out_7890251207530459337[2] = 0;
   out_7890251207530459337[3] = 1;
   out_7890251207530459337[4] = 0;
   out_7890251207530459337[5] = 0;
   out_7890251207530459337[6] = 0;
   out_7890251207530459337[7] = 0;
   out_7890251207530459337[8] = 0;
}
void h_29(double *state, double *unused, double *out_584183302954915602) {
   out_584183302954915602[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6176888480660908304) {
   out_6176888480660908304[0] = 0;
   out_6176888480660908304[1] = 1;
   out_6176888480660908304[2] = 0;
   out_6176888480660908304[3] = 0;
   out_6176888480660908304[4] = 0;
   out_6176888480660908304[5] = 0;
   out_6176888480660908304[6] = 0;
   out_6176888480660908304[7] = 0;
   out_6176888480660908304[8] = 0;
}
void h_28(double *state, double *unused, double *out_3463782686048119142) {
   out_3463782686048119142[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1094489463591377730) {
   out_1094489463591377730[0] = 1;
   out_1094489463591377730[1] = 0;
   out_1094489463591377730[2] = 0;
   out_1094489463591377730[3] = 0;
   out_1094489463591377730[4] = 0;
   out_1094489463591377730[5] = 0;
   out_1094489463591377730[6] = 0;
   out_1094489463591377730[7] = 0;
   out_1094489463591377730[8] = 0;
}
void h_31(double *state, double *unused, double *out_8686632192313917121) {
   out_8686632192313917121[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3178970139716227921) {
   out_3178970139716227921[0] = 0;
   out_3178970139716227921[1] = 0;
   out_3178970139716227921[2] = 0;
   out_3178970139716227921[3] = 0;
   out_3178970139716227921[4] = 0;
   out_3178970139716227921[5] = 0;
   out_3178970139716227921[6] = 0;
   out_3178970139716227921[7] = 0;
   out_3178970139716227921[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7312882027617302654) {
  err_fun(nom_x, delta_x, out_7312882027617302654);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7611434335802104229) {
  inv_err_fun(nom_x, true_x, out_7611434335802104229);
}
void car_H_mod_fun(double *state, double *out_1755367647345397347) {
  H_mod_fun(state, out_1755367647345397347);
}
void car_f_fun(double *state, double dt, double *out_3297305622544450583) {
  f_fun(state,  dt, out_3297305622544450583);
}
void car_F_fun(double *state, double dt, double *out_3243986764759865323) {
  F_fun(state,  dt, out_3243986764759865323);
}
void car_h_25(double *state, double *unused, double *out_5049182664059894771) {
  h_25(state, unused, out_5049182664059894771);
}
void car_H_25(double *state, double *unused, double *out_3148324177839267493) {
  H_25(state, unused, out_3148324177839267493);
}
void car_h_24(double *state, double *unused, double *out_7961352067590225679) {
  h_24(state, unused, out_7961352067590225679);
}
void car_H_24(double *state, double *unused, double *out_8722848089278766022) {
  H_24(state, unused, out_8722848089278766022);
}
void car_h_30(double *state, double *unused, double *out_6226913174035872437) {
  h_30(state, unused, out_6226913174035872437);
}
void car_H_30(double *state, double *unused, double *out_5666657136346516120) {
  H_30(state, unused, out_5666657136346516120);
}
void car_h_26(double *state, double *unused, double *out_7794469984375447646) {
  h_26(state, unused, out_7794469984375447646);
}
void car_H_26(double *state, double *unused, double *out_593179141034788731) {
  H_26(state, unused, out_593179141034788731);
}
void car_h_27(double *state, double *unused, double *out_7912477378308824390) {
  h_27(state, unused, out_7912477378308824390);
}
void car_H_27(double *state, double *unused, double *out_7890251207530459337) {
  H_27(state, unused, out_7890251207530459337);
}
void car_h_29(double *state, double *unused, double *out_584183302954915602) {
  h_29(state, unused, out_584183302954915602);
}
void car_H_29(double *state, double *unused, double *out_6176888480660908304) {
  H_29(state, unused, out_6176888480660908304);
}
void car_h_28(double *state, double *unused, double *out_3463782686048119142) {
  h_28(state, unused, out_3463782686048119142);
}
void car_H_28(double *state, double *unused, double *out_1094489463591377730) {
  H_28(state, unused, out_1094489463591377730);
}
void car_h_31(double *state, double *unused, double *out_8686632192313917121) {
  h_31(state, unused, out_8686632192313917121);
}
void car_H_31(double *state, double *unused, double *out_3178970139716227921) {
  H_31(state, unused, out_3178970139716227921);
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

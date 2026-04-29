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
void err_fun(double *nom_x, double *delta_x, double *out_1836217441218094185) {
   out_1836217441218094185[0] = delta_x[0] + nom_x[0];
   out_1836217441218094185[1] = delta_x[1] + nom_x[1];
   out_1836217441218094185[2] = delta_x[2] + nom_x[2];
   out_1836217441218094185[3] = delta_x[3] + nom_x[3];
   out_1836217441218094185[4] = delta_x[4] + nom_x[4];
   out_1836217441218094185[5] = delta_x[5] + nom_x[5];
   out_1836217441218094185[6] = delta_x[6] + nom_x[6];
   out_1836217441218094185[7] = delta_x[7] + nom_x[7];
   out_1836217441218094185[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3664144515047539542) {
   out_3664144515047539542[0] = -nom_x[0] + true_x[0];
   out_3664144515047539542[1] = -nom_x[1] + true_x[1];
   out_3664144515047539542[2] = -nom_x[2] + true_x[2];
   out_3664144515047539542[3] = -nom_x[3] + true_x[3];
   out_3664144515047539542[4] = -nom_x[4] + true_x[4];
   out_3664144515047539542[5] = -nom_x[5] + true_x[5];
   out_3664144515047539542[6] = -nom_x[6] + true_x[6];
   out_3664144515047539542[7] = -nom_x[7] + true_x[7];
   out_3664144515047539542[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6584291372032100099) {
   out_6584291372032100099[0] = 1.0;
   out_6584291372032100099[1] = 0.0;
   out_6584291372032100099[2] = 0.0;
   out_6584291372032100099[3] = 0.0;
   out_6584291372032100099[4] = 0.0;
   out_6584291372032100099[5] = 0.0;
   out_6584291372032100099[6] = 0.0;
   out_6584291372032100099[7] = 0.0;
   out_6584291372032100099[8] = 0.0;
   out_6584291372032100099[9] = 0.0;
   out_6584291372032100099[10] = 1.0;
   out_6584291372032100099[11] = 0.0;
   out_6584291372032100099[12] = 0.0;
   out_6584291372032100099[13] = 0.0;
   out_6584291372032100099[14] = 0.0;
   out_6584291372032100099[15] = 0.0;
   out_6584291372032100099[16] = 0.0;
   out_6584291372032100099[17] = 0.0;
   out_6584291372032100099[18] = 0.0;
   out_6584291372032100099[19] = 0.0;
   out_6584291372032100099[20] = 1.0;
   out_6584291372032100099[21] = 0.0;
   out_6584291372032100099[22] = 0.0;
   out_6584291372032100099[23] = 0.0;
   out_6584291372032100099[24] = 0.0;
   out_6584291372032100099[25] = 0.0;
   out_6584291372032100099[26] = 0.0;
   out_6584291372032100099[27] = 0.0;
   out_6584291372032100099[28] = 0.0;
   out_6584291372032100099[29] = 0.0;
   out_6584291372032100099[30] = 1.0;
   out_6584291372032100099[31] = 0.0;
   out_6584291372032100099[32] = 0.0;
   out_6584291372032100099[33] = 0.0;
   out_6584291372032100099[34] = 0.0;
   out_6584291372032100099[35] = 0.0;
   out_6584291372032100099[36] = 0.0;
   out_6584291372032100099[37] = 0.0;
   out_6584291372032100099[38] = 0.0;
   out_6584291372032100099[39] = 0.0;
   out_6584291372032100099[40] = 1.0;
   out_6584291372032100099[41] = 0.0;
   out_6584291372032100099[42] = 0.0;
   out_6584291372032100099[43] = 0.0;
   out_6584291372032100099[44] = 0.0;
   out_6584291372032100099[45] = 0.0;
   out_6584291372032100099[46] = 0.0;
   out_6584291372032100099[47] = 0.0;
   out_6584291372032100099[48] = 0.0;
   out_6584291372032100099[49] = 0.0;
   out_6584291372032100099[50] = 1.0;
   out_6584291372032100099[51] = 0.0;
   out_6584291372032100099[52] = 0.0;
   out_6584291372032100099[53] = 0.0;
   out_6584291372032100099[54] = 0.0;
   out_6584291372032100099[55] = 0.0;
   out_6584291372032100099[56] = 0.0;
   out_6584291372032100099[57] = 0.0;
   out_6584291372032100099[58] = 0.0;
   out_6584291372032100099[59] = 0.0;
   out_6584291372032100099[60] = 1.0;
   out_6584291372032100099[61] = 0.0;
   out_6584291372032100099[62] = 0.0;
   out_6584291372032100099[63] = 0.0;
   out_6584291372032100099[64] = 0.0;
   out_6584291372032100099[65] = 0.0;
   out_6584291372032100099[66] = 0.0;
   out_6584291372032100099[67] = 0.0;
   out_6584291372032100099[68] = 0.0;
   out_6584291372032100099[69] = 0.0;
   out_6584291372032100099[70] = 1.0;
   out_6584291372032100099[71] = 0.0;
   out_6584291372032100099[72] = 0.0;
   out_6584291372032100099[73] = 0.0;
   out_6584291372032100099[74] = 0.0;
   out_6584291372032100099[75] = 0.0;
   out_6584291372032100099[76] = 0.0;
   out_6584291372032100099[77] = 0.0;
   out_6584291372032100099[78] = 0.0;
   out_6584291372032100099[79] = 0.0;
   out_6584291372032100099[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_775091494716029244) {
   out_775091494716029244[0] = state[0];
   out_775091494716029244[1] = state[1];
   out_775091494716029244[2] = state[2];
   out_775091494716029244[3] = state[3];
   out_775091494716029244[4] = state[4];
   out_775091494716029244[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_775091494716029244[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_775091494716029244[7] = state[7];
   out_775091494716029244[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1373692616022552200) {
   out_1373692616022552200[0] = 1;
   out_1373692616022552200[1] = 0;
   out_1373692616022552200[2] = 0;
   out_1373692616022552200[3] = 0;
   out_1373692616022552200[4] = 0;
   out_1373692616022552200[5] = 0;
   out_1373692616022552200[6] = 0;
   out_1373692616022552200[7] = 0;
   out_1373692616022552200[8] = 0;
   out_1373692616022552200[9] = 0;
   out_1373692616022552200[10] = 1;
   out_1373692616022552200[11] = 0;
   out_1373692616022552200[12] = 0;
   out_1373692616022552200[13] = 0;
   out_1373692616022552200[14] = 0;
   out_1373692616022552200[15] = 0;
   out_1373692616022552200[16] = 0;
   out_1373692616022552200[17] = 0;
   out_1373692616022552200[18] = 0;
   out_1373692616022552200[19] = 0;
   out_1373692616022552200[20] = 1;
   out_1373692616022552200[21] = 0;
   out_1373692616022552200[22] = 0;
   out_1373692616022552200[23] = 0;
   out_1373692616022552200[24] = 0;
   out_1373692616022552200[25] = 0;
   out_1373692616022552200[26] = 0;
   out_1373692616022552200[27] = 0;
   out_1373692616022552200[28] = 0;
   out_1373692616022552200[29] = 0;
   out_1373692616022552200[30] = 1;
   out_1373692616022552200[31] = 0;
   out_1373692616022552200[32] = 0;
   out_1373692616022552200[33] = 0;
   out_1373692616022552200[34] = 0;
   out_1373692616022552200[35] = 0;
   out_1373692616022552200[36] = 0;
   out_1373692616022552200[37] = 0;
   out_1373692616022552200[38] = 0;
   out_1373692616022552200[39] = 0;
   out_1373692616022552200[40] = 1;
   out_1373692616022552200[41] = 0;
   out_1373692616022552200[42] = 0;
   out_1373692616022552200[43] = 0;
   out_1373692616022552200[44] = 0;
   out_1373692616022552200[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1373692616022552200[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1373692616022552200[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1373692616022552200[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1373692616022552200[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1373692616022552200[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1373692616022552200[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1373692616022552200[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1373692616022552200[53] = -9.8100000000000005*dt;
   out_1373692616022552200[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1373692616022552200[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1373692616022552200[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1373692616022552200[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1373692616022552200[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1373692616022552200[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1373692616022552200[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1373692616022552200[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1373692616022552200[62] = 0;
   out_1373692616022552200[63] = 0;
   out_1373692616022552200[64] = 0;
   out_1373692616022552200[65] = 0;
   out_1373692616022552200[66] = 0;
   out_1373692616022552200[67] = 0;
   out_1373692616022552200[68] = 0;
   out_1373692616022552200[69] = 0;
   out_1373692616022552200[70] = 1;
   out_1373692616022552200[71] = 0;
   out_1373692616022552200[72] = 0;
   out_1373692616022552200[73] = 0;
   out_1373692616022552200[74] = 0;
   out_1373692616022552200[75] = 0;
   out_1373692616022552200[76] = 0;
   out_1373692616022552200[77] = 0;
   out_1373692616022552200[78] = 0;
   out_1373692616022552200[79] = 0;
   out_1373692616022552200[80] = 1;
}
void h_25(double *state, double *unused, double *out_1913811324090937931) {
   out_1913811324090937931[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3061310722042608422) {
   out_3061310722042608422[0] = 0;
   out_3061310722042608422[1] = 0;
   out_3061310722042608422[2] = 0;
   out_3061310722042608422[3] = 0;
   out_3061310722042608422[4] = 0;
   out_3061310722042608422[5] = 0;
   out_3061310722042608422[6] = 1;
   out_3061310722042608422[7] = 0;
   out_3061310722042608422[8] = 0;
}
void h_24(double *state, double *unused, double *out_8648690320250273547) {
   out_8648690320250273547[0] = state[4];
   out_8648690320250273547[1] = state[5];
}
void H_24(double *state, double *unused, double *out_496614876759714714) {
   out_496614876759714714[0] = 0;
   out_496614876759714714[1] = 0;
   out_496614876759714714[2] = 0;
   out_496614876759714714[3] = 0;
   out_496614876759714714[4] = 1;
   out_496614876759714714[5] = 0;
   out_496614876759714714[6] = 0;
   out_496614876759714714[7] = 0;
   out_496614876759714714[8] = 0;
   out_496614876759714714[9] = 0;
   out_496614876759714714[10] = 0;
   out_496614876759714714[11] = 0;
   out_496614876759714714[12] = 0;
   out_496614876759714714[13] = 0;
   out_496614876759714714[14] = 1;
   out_496614876759714714[15] = 0;
   out_496614876759714714[16] = 0;
   out_496614876759714714[17] = 0;
}
void h_30(double *state, double *unused, double *out_537139063672349691) {
   out_537139063672349691[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3190649669185848492) {
   out_3190649669185848492[0] = 0;
   out_3190649669185848492[1] = 0;
   out_3190649669185848492[2] = 0;
   out_3190649669185848492[3] = 0;
   out_3190649669185848492[4] = 1;
   out_3190649669185848492[5] = 0;
   out_3190649669185848492[6] = 0;
   out_3190649669185848492[7] = 0;
   out_3190649669185848492[8] = 0;
}
void h_26(double *state, double *unused, double *out_8588046903789292966) {
   out_8588046903789292966[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6802814040916664646) {
   out_6802814040916664646[0] = 0;
   out_6802814040916664646[1] = 0;
   out_6802814040916664646[2] = 0;
   out_6802814040916664646[3] = 0;
   out_6802814040916664646[4] = 0;
   out_6802814040916664646[5] = 0;
   out_6802814040916664646[6] = 0;
   out_6802814040916664646[7] = 1;
   out_6802814040916664646[8] = 0;
}
void h_27(double *state, double *unused, double *out_5342332393002513778) {
   out_5342332393002513778[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5365412980986273403) {
   out_5365412980986273403[0] = 0;
   out_5365412980986273403[1] = 0;
   out_5365412980986273403[2] = 0;
   out_5365412980986273403[3] = 1;
   out_5365412980986273403[4] = 0;
   out_5365412980986273403[5] = 0;
   out_5365412980986273403[6] = 0;
   out_5365412980986273403[7] = 0;
   out_5365412980986273403[8] = 0;
}
void h_29(double *state, double *unused, double *out_4607750999443297846) {
   out_4607750999443297846[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2680418324871456308) {
   out_2680418324871456308[0] = 0;
   out_2680418324871456308[1] = 1;
   out_2680418324871456308[2] = 0;
   out_2680418324871456308[3] = 0;
   out_2680418324871456308[4] = 0;
   out_2680418324871456308[5] = 0;
   out_2680418324871456308[6] = 0;
   out_2680418324871456308[7] = 0;
   out_2680418324871456308[8] = 0;
}
void h_28(double *state, double *unused, double *out_1728151616350094306) {
   out_1728151616350094306[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6285569348784196606) {
   out_6285569348784196606[0] = 1;
   out_6285569348784196606[1] = 0;
   out_6285569348784196606[2] = 0;
   out_6285569348784196606[3] = 0;
   out_6285569348784196606[4] = 0;
   out_6285569348784196606[5] = 0;
   out_6285569348784196606[6] = 0;
   out_6285569348784196606[7] = 0;
   out_6285569348784196606[8] = 0;
}
void h_31(double *state, double *unused, double *out_7827544350761686304) {
   out_7827544350761686304[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3030664760165647994) {
   out_3030664760165647994[0] = 0;
   out_3030664760165647994[1] = 0;
   out_3030664760165647994[2] = 0;
   out_3030664760165647994[3] = 0;
   out_3030664760165647994[4] = 0;
   out_3030664760165647994[5] = 0;
   out_3030664760165647994[6] = 0;
   out_3030664760165647994[7] = 0;
   out_3030664760165647994[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_1836217441218094185) {
  err_fun(nom_x, delta_x, out_1836217441218094185);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3664144515047539542) {
  inv_err_fun(nom_x, true_x, out_3664144515047539542);
}
void car_H_mod_fun(double *state, double *out_6584291372032100099) {
  H_mod_fun(state, out_6584291372032100099);
}
void car_f_fun(double *state, double dt, double *out_775091494716029244) {
  f_fun(state,  dt, out_775091494716029244);
}
void car_F_fun(double *state, double dt, double *out_1373692616022552200) {
  F_fun(state,  dt, out_1373692616022552200);
}
void car_h_25(double *state, double *unused, double *out_1913811324090937931) {
  h_25(state, unused, out_1913811324090937931);
}
void car_H_25(double *state, double *unused, double *out_3061310722042608422) {
  H_25(state, unused, out_3061310722042608422);
}
void car_h_24(double *state, double *unused, double *out_8648690320250273547) {
  h_24(state, unused, out_8648690320250273547);
}
void car_H_24(double *state, double *unused, double *out_496614876759714714) {
  H_24(state, unused, out_496614876759714714);
}
void car_h_30(double *state, double *unused, double *out_537139063672349691) {
  h_30(state, unused, out_537139063672349691);
}
void car_H_30(double *state, double *unused, double *out_3190649669185848492) {
  H_30(state, unused, out_3190649669185848492);
}
void car_h_26(double *state, double *unused, double *out_8588046903789292966) {
  h_26(state, unused, out_8588046903789292966);
}
void car_H_26(double *state, double *unused, double *out_6802814040916664646) {
  H_26(state, unused, out_6802814040916664646);
}
void car_h_27(double *state, double *unused, double *out_5342332393002513778) {
  h_27(state, unused, out_5342332393002513778);
}
void car_H_27(double *state, double *unused, double *out_5365412980986273403) {
  H_27(state, unused, out_5365412980986273403);
}
void car_h_29(double *state, double *unused, double *out_4607750999443297846) {
  h_29(state, unused, out_4607750999443297846);
}
void car_H_29(double *state, double *unused, double *out_2680418324871456308) {
  H_29(state, unused, out_2680418324871456308);
}
void car_h_28(double *state, double *unused, double *out_1728151616350094306) {
  h_28(state, unused, out_1728151616350094306);
}
void car_H_28(double *state, double *unused, double *out_6285569348784196606) {
  H_28(state, unused, out_6285569348784196606);
}
void car_h_31(double *state, double *unused, double *out_7827544350761686304) {
  h_31(state, unused, out_7827544350761686304);
}
void car_H_31(double *state, double *unused, double *out_3030664760165647994) {
  H_31(state, unused, out_3030664760165647994);
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

#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_3037457306540495367);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5504742843249496146);
void pose_H_mod_fun(double *state, double *out_4825348341631687231);
void pose_f_fun(double *state, double dt, double *out_6515554172087681296);
void pose_F_fun(double *state, double dt, double *out_6200830679267345445);
void pose_h_4(double *state, double *unused, double *out_2281311047032518481);
void pose_H_4(double *state, double *unused, double *out_1022733465120271490);
void pose_h_10(double *state, double *unused, double *out_7298821485634386065);
void pose_H_10(double *state, double *unused, double *out_4580916266697147042);
void pose_h_13(double *state, double *unused, double *out_1884508169225221663);
void pose_H_13(double *state, double *unused, double *out_2189540360212061311);
void pose_h_14(double *state, double *unused, double *out_1873182816391455999);
void pose_H_14(double *state, double *unused, double *out_2940507391219213039);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
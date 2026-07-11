#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_6464539647254934838);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6486962015996802201);
void pose_H_mod_fun(double *state, double *out_6072805508798864108);
void pose_f_fun(double *state, double dt, double *out_5954902381465487783);
void pose_F_fun(double *state, double dt, double *out_3606916503598787696);
void pose_h_4(double *state, double *unused, double *out_5987237266806342619);
void pose_H_4(double *state, double *unused, double *out_2798583430007914855);
void pose_h_10(double *state, double *unused, double *out_3025597584677695987);
void pose_H_10(double *state, double *unused, double *out_1745173860280422879);
void pose_h_13(double *state, double *unused, double *out_8599707273883010977);
void pose_H_13(double *state, double *unused, double *out_8037529435384935832);
void pose_h_14(double *state, double *unused, double *out_3899158584107184147);
void pose_H_14(double *state, double *unused, double *out_6761824286347399384);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
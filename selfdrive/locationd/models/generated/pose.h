#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_3842342068078672063);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_8486186134848101365);
void pose_H_mod_fun(double *state, double *out_6473267640003251484);
void pose_f_fun(double *state, double dt, double *out_6565120308118865312);
void pose_F_fun(double *state, double dt, double *out_5609468243508396124);
void pose_h_4(double *state, double *unused, double *out_4855683315993454061);
void pose_H_4(double *state, double *unused, double *out_8201669223862392560);
void pose_h_10(double *state, double *unused, double *out_851235722336489475);
void pose_H_10(double *state, double *unused, double *out_3556931133651902201);
void pose_h_13(double *state, double *unused, double *out_4193103824640271278);
void pose_H_13(double *state, double *unused, double *out_591038015545691631);
void pose_h_14(double *state, double *unused, double *out_379962720619032263);
void pose_H_14(double *state, double *unused, double *out_7162286417551786760);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_8879515811318903877);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5299248816140732451);
void pose_H_mod_fun(double *state, double *out_4677544152144015683);
void pose_f_fun(double *state, double dt, double *out_7013494318329414190);
void pose_F_fun(double *state, double dt, double *out_1354267763168931680);
void pose_h_4(double *state, double *unused, double *out_4996940222577203074);
void pose_H_4(double *state, double *unused, double *out_1403322073353066430);
void pose_h_10(double *state, double *unused, double *out_4736471881593411299);
void pose_H_10(double *state, double *unused, double *out_6592862647789865695);
void pose_h_13(double *state, double *unused, double *out_4300313025089579851);
void pose_H_13(double *state, double *unused, double *out_4615595898685399231);
void pose_h_14(double *state, double *unused, double *out_4861809331317933851);
void pose_H_14(double *state, double *unused, double *out_5366562929692550959);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
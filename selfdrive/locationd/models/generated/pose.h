#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_8885455317417923135);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7818671957054169652);
void pose_H_mod_fun(double *state, double *out_6824361727259051753);
void pose_f_fun(double *state, double dt, double *out_1755259086508631527);
void pose_F_fun(double *state, double dt, double *out_4099616348303124892);
void pose_h_4(double *state, double *unused, double *out_2755321730925237079);
void pose_H_4(double *state, double *unused, double *out_3052554517415144181);
void pose_h_10(double *state, double *unused, double *out_5342072843391651998);
void pose_H_10(double *state, double *unused, double *out_7744556233743894871);
void pose_h_13(double *state, double *unused, double *out_2174293281027337494);
void pose_H_13(double *state, double *unused, double *out_6886309980717668205);
void pose_h_14(double *state, double *unused, double *out_7883954814799996171);
void pose_H_14(double *state, double *unused, double *out_6135342949710516477);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_1527400177482295711);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7620805951199866590);
void pose_H_mod_fun(double *state, double *out_2041474721312435071);
void pose_f_fun(double *state, double dt, double *out_3319995548118437592);
void pose_F_fun(double *state, double dt, double *out_1297884238983223646);
void pose_h_4(double *state, double *unused, double *out_8728651807226544018);
void pose_H_4(double *state, double *unused, double *out_5315696800103384324);
void pose_h_10(double *state, double *unused, double *out_3679201747496490736);
void pose_H_10(double *state, double *unused, double *out_2420796569664890088);
void pose_h_13(double *state, double *unused, double *out_4772104686591378688);
void pose_H_13(double *state, double *unused, double *out_2103422974771051523);
void pose_h_14(double *state, double *unused, double *out_3663326241180753804);
void pose_H_14(double *state, double *unused, double *out_1352455943763899795);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
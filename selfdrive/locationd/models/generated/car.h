#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_7312882027617302654);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7611434335802104229);
void car_H_mod_fun(double *state, double *out_1755367647345397347);
void car_f_fun(double *state, double dt, double *out_3297305622544450583);
void car_F_fun(double *state, double dt, double *out_3243986764759865323);
void car_h_25(double *state, double *unused, double *out_5049182664059894771);
void car_H_25(double *state, double *unused, double *out_3148324177839267493);
void car_h_24(double *state, double *unused, double *out_7961352067590225679);
void car_H_24(double *state, double *unused, double *out_8722848089278766022);
void car_h_30(double *state, double *unused, double *out_6226913174035872437);
void car_H_30(double *state, double *unused, double *out_5666657136346516120);
void car_h_26(double *state, double *unused, double *out_7794469984375447646);
void car_H_26(double *state, double *unused, double *out_593179141034788731);
void car_h_27(double *state, double *unused, double *out_7912477378308824390);
void car_H_27(double *state, double *unused, double *out_7890251207530459337);
void car_h_29(double *state, double *unused, double *out_584183302954915602);
void car_H_29(double *state, double *unused, double *out_6176888480660908304);
void car_h_28(double *state, double *unused, double *out_3463782686048119142);
void car_H_28(double *state, double *unused, double *out_1094489463591377730);
void car_h_31(double *state, double *unused, double *out_8686632192313917121);
void car_H_31(double *state, double *unused, double *out_3178970139716227921);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
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
void car_err_fun(double *nom_x, double *delta_x, double *out_3577007229449845283);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6399722403094813471);
void car_H_mod_fun(double *state, double *out_7904303228569973439);
void car_f_fun(double *state, double dt, double *out_2229936979988599717);
void car_F_fun(double *state, double dt, double *out_265299588973257661);
void car_h_25(double *state, double *unused, double *out_6366176548286495673);
void car_H_25(double *state, double *unused, double *out_2729963794422514103);
void car_h_24(double *state, double *unused, double *out_3288800728819921756);
void car_H_24(double *state, double *unused, double *out_3471068885428647323);
void car_h_30(double *state, double *unused, double *out_4915545155657612506);
void car_H_30(double *state, double *unused, double *out_5248296752929762730);
void car_h_26(double *state, double *unused, double *out_4312229270981134989);
void car_H_26(double *state, double *unused, double *out_1011539524451542121);
void car_h_27(double *state, double *unused, double *out_2035945772564408966);
void car_H_27(double *state, double *unused, double *out_425861535478849122);
void car_h_29(double *state, double *unused, double *out_8529525650391093465);
void car_H_29(double *state, double *unused, double *out_1287501191390701911);
void car_h_28(double *state, double *unused, double *out_9014834141134194238);
void car_H_28(double *state, double *unused, double *out_676129080174624340);
void car_h_31(double *state, double *unused, double *out_1515096102467553512);
void car_H_31(double *state, double *unused, double *out_2760609756299474531);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
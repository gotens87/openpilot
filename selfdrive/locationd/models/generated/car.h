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
void car_err_fun(double *nom_x, double *delta_x, double *out_4786623230309029832);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1853746942225963803);
void car_H_mod_fun(double *state, double *out_8577930728410900676);
void car_f_fun(double *state, double dt, double *out_4086600449481598187);
void car_F_fun(double *state, double dt, double *out_4206537046303623167);
void car_h_25(double *state, double *unused, double *out_1336693333340347965);
void car_H_25(double *state, double *unused, double *out_675625119386413703);
void car_h_24(double *state, double *unused, double *out_14163058279738692);
void car_H_24(double *state, double *unused, double *out_5949119402299426297);
void car_h_30(double *state, double *unused, double *out_65271124810109003);
void car_H_30(double *state, double *unused, double *out_3193958077893662330);
void car_h_26(double *state, double *unused, double *out_8783108044678801329);
void car_H_26(double *state, double *unused, double *out_3065878199487642521);
void car_h_27(double *state, double *unused, double *out_6338366601001193610);
void car_H_27(double *state, double *unused, double *out_5417552149077605547);
void car_h_29(double *state, double *unused, double *out_8470927944454335656);
void car_H_29(double *state, double *unused, double *out_3704189422208054514);
void car_h_28(double *state, double *unused, double *out_3717698227664568318);
void car_H_28(double *state, double *unused, double *out_1378209594861476060);
void car_h_31(double *state, double *unused, double *out_3977914715348637242);
void car_H_31(double *state, double *unused, double *out_3692086301720993997);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
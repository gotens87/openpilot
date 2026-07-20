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
void car_err_fun(double *nom_x, double *delta_x, double *out_6456809504724999118);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5644557425305384061);
void car_H_mod_fun(double *state, double *out_3623313066644464969);
void car_f_fun(double *state, double dt, double *out_2530420459597428980);
void car_F_fun(double *state, double dt, double *out_7641637713499099284);
void car_h_25(double *state, double *unused, double *out_9149101986531247315);
void car_H_25(double *state, double *unused, double *out_8527004891829129809);
void car_h_24(double *state, double *unused, double *out_3166828984508355697);
void car_H_24(double *state, double *unused, double *out_2072385171006399745);
void car_h_30(double *state, double *unused, double *out_1385836729710574874);
void car_H_30(double *state, double *unused, double *out_7401406223373173180);
void car_h_26(double *state, double *unused, double *out_7332000190158117452);
void car_H_26(double *state, double *unused, double *out_4785501572955073585);
void car_h_27(double *state, double *unused, double *out_3419356599619589213);
void car_H_27(double *state, double *unused, double *out_8870574538535953525);
void car_h_29(double *state, double *unused, double *out_442326876808912439);
void car_H_29(double *state, double *unused, double *out_6891174879058780996);
void car_h_28(double *state, double *unused, double *out_6231908702889442596);
void car_H_28(double *state, double *unused, double *out_6473170177581240046);
void car_h_31(double *state, double *unused, double *out_657764610459762417);
void car_H_31(double *state, double *unused, double *out_4159293470721722109);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
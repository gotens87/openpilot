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
void car_err_fun(double *nom_x, double *delta_x, double *out_7550976477549507356);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3781853638498010);
void car_H_mod_fun(double *state, double *out_6037661802737362721);
void car_f_fun(double *state, double dt, double *out_8701591355184024691);
void car_F_fun(double *state, double dt, double *out_4180724957458131151);
void car_h_25(double *state, double *unused, double *out_76571637109634956);
void car_H_25(double *state, double *unused, double *out_7505390445787524055);
void car_h_24(double *state, double *unused, double *out_6249911379334495626);
void car_H_24(double *state, double *unused, double *out_2231896162874511461);
void car_h_30(double *state, double *unused, double *out_198622425174870933);
void car_H_30(double *state, double *unused, double *out_6413657297794419363);
void car_h_26(double *state, double *unused, double *out_5679663062383844155);
void car_H_26(double *state, double *unused, double *out_7199850309047971337);
void car_h_27(double *state, double *unused, double *out_4925101630551210689);
void car_H_27(double *state, double *unused, double *out_8637251368978362580);
void car_h_29(double *state, double *unused, double *out_5200295692835716578);
void car_H_29(double *state, double *unused, double *out_6923888642108811547);
void car_h_28(double *state, double *unused, double *out_8899456413702232613);
void car_H_28(double *state, double *unused, double *out_8887518913674137798);
void car_h_31(double *state, double *unused, double *out_3714021165363657306);
void car_H_31(double *state, double *unused, double *out_7474744483910563627);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
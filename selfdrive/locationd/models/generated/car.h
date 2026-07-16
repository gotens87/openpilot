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
void car_err_fun(double *nom_x, double *delta_x, double *out_5469727734415202139);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9043415007849521510);
void car_H_mod_fun(double *state, double *out_3975286947566256014);
void car_f_fun(double *state, double dt, double *out_1399233721380390552);
void car_F_fun(double *state, double dt, double *out_1547855628850571031);
void car_h_25(double *state, double *unused, double *out_4999223901660539793);
void car_H_25(double *state, double *unused, double *out_5217901278345981223);
void car_h_24(double *state, double *unused, double *out_6882293632162622150);
void car_H_24(double *state, double *unused, double *out_1436304179762204534);
void car_h_30(double *state, double *unused, double *out_4724029839376033904);
void car_H_30(double *state, double *unused, double *out_8701146465235962195);
void car_h_26(double *state, double *unused, double *out_3556286038299892167);
void car_H_26(double *state, double *unused, double *out_8959404597220037447);
void car_h_27(double *state, double *unused, double *out_356357552238636072);
void car_H_27(double *state, double *unused, double *out_7522003537289646204);
void car_h_29(double *state, double *unused, double *out_277643428284811741);
void car_H_29(double *state, double *unused, double *out_9211377809550354379);
void car_h_28(double *state, double *unused, double *out_5358030819899481717);
void car_H_28(double *state, double *unused, double *out_4128978792480823805);
void car_h_31(double *state, double *unused, double *out_2893715593223983566);
void car_H_31(double *state, double *unused, double *out_8861131374256162693);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
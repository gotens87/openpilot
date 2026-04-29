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
void car_err_fun(double *nom_x, double *delta_x, double *out_1836217441218094185);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3664144515047539542);
void car_H_mod_fun(double *state, double *out_6584291372032100099);
void car_f_fun(double *state, double dt, double *out_775091494716029244);
void car_F_fun(double *state, double dt, double *out_1373692616022552200);
void car_h_25(double *state, double *unused, double *out_1913811324090937931);
void car_H_25(double *state, double *unused, double *out_3061310722042608422);
void car_h_24(double *state, double *unused, double *out_8648690320250273547);
void car_H_24(double *state, double *unused, double *out_496614876759714714);
void car_h_30(double *state, double *unused, double *out_537139063672349691);
void car_H_30(double *state, double *unused, double *out_3190649669185848492);
void car_h_26(double *state, double *unused, double *out_8588046903789292966);
void car_H_26(double *state, double *unused, double *out_6802814040916664646);
void car_h_27(double *state, double *unused, double *out_5342332393002513778);
void car_H_27(double *state, double *unused, double *out_5365412980986273403);
void car_h_29(double *state, double *unused, double *out_4607750999443297846);
void car_H_29(double *state, double *unused, double *out_2680418324871456308);
void car_h_28(double *state, double *unused, double *out_1728151616350094306);
void car_H_28(double *state, double *unused, double *out_6285569348784196606);
void car_h_31(double *state, double *unused, double *out_7827544350761686304);
void car_H_31(double *state, double *unused, double *out_3030664760165647994);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
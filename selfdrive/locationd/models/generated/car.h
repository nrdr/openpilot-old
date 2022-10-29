#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_7738871226124743511);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1257810621133578395);
void car_H_mod_fun(double *state, double *out_1082627477705123836);
void car_f_fun(double *state, double dt, double *out_8060049046834174365);
void car_F_fun(double *state, double dt, double *out_5342944527585928037);
void car_h_25(double *state, double *unused, double *out_2560243035918826463);
void car_H_25(double *state, double *unused, double *out_2236407327837830560);
void car_h_24(double *state, double *unused, double *out_5994894285391379092);
void car_H_24(double *state, double *unused, double *out_1395975543655785568);
void car_h_30(double *state, double *unused, double *out_3385895141706669319);
void car_H_30(double *state, double *unused, double *out_2291289002289777638);
void car_h_26(double *state, double *unused, double *out_7783847222988298331);
void car_H_26(double *state, double *unused, double *out_1505095991036225664);
void car_h_27(double *state, double *unused, double *out_2441430231742019182);
void car_H_27(double *state, double *unused, double *out_4466052314090202549);
void car_h_29(double *state, double *unused, double *out_2716624294026525071);
void car_H_29(double *state, double *unused, double *out_1781057657975385454);
void car_h_28(double *state, double *unused, double *out_7443103499402864827);
void car_H_28(double *state, double *unused, double *out_6863456675044916028);
void car_h_31(double *state, double *unused, double *out_5203022220901845978);
void car_H_31(double *state, double *unused, double *out_2131304093269577140);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
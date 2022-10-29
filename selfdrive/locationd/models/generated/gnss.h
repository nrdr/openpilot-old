#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9152697212499809699);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_6641208110102973198);
void gnss_H_mod_fun(double *state, double *out_4882180096896426461);
void gnss_f_fun(double *state, double dt, double *out_8139212335340482105);
void gnss_F_fun(double *state, double dt, double *out_2110048490497678022);
void gnss_h_6(double *state, double *sat_pos, double *out_2787231368960978811);
void gnss_H_6(double *state, double *sat_pos, double *out_1075816899521368544);
void gnss_h_20(double *state, double *sat_pos, double *out_8259322119346965405);
void gnss_H_20(double *state, double *sat_pos, double *out_3930923713889965456);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_5032550934911842458);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_1106145761210530165);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_5032550934911842458);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_1106145761210530165);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
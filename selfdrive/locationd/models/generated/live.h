#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_791848971611107440);
void live_err_fun(double *nom_x, double *delta_x, double *out_7263316136879107766);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4539108427845857732);
void live_H_mod_fun(double *state, double *out_5247633436136850661);
void live_f_fun(double *state, double dt, double *out_1873705910207205425);
void live_F_fun(double *state, double dt, double *out_4839540147203828691);
void live_h_4(double *state, double *unused, double *out_7209362868161800058);
void live_H_4(double *state, double *unused, double *out_5567472697196142778);
void live_h_9(double *state, double *unused, double *out_6932725484037252388);
void live_H_9(double *state, double *unused, double *out_5326283050566552133);
void live_h_10(double *state, double *unused, double *out_7251092503996911185);
void live_H_10(double *state, double *unused, double *out_189169393835983707);
void live_h_12(double *state, double *unused, double *out_8137161948703201078);
void live_H_12(double *state, double *unused, double *out_548016289164180983);
void live_h_35(double *state, double *unused, double *out_2589679212712571072);
void live_H_35(double *state, double *unused, double *out_2197546743160832726);
void live_h_32(double *state, double *unused, double *out_8653012006770198125);
void live_H_32(double *state, double *unused, double *out_3845650149534435004);
void live_h_13(double *state, double *unused, double *out_6403640437141376254);
void live_H_13(double *state, double *unused, double *out_3957022054025267238);
void live_h_14(double *state, double *unused, double *out_6932725484037252388);
void live_H_14(double *state, double *unused, double *out_5326283050566552133);
void live_h_33(double *state, double *unused, double *out_4501678879098190893);
void live_H_33(double *state, double *unused, double *out_5348103747799690330);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
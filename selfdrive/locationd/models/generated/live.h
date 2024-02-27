#pragma once
#include "rednose/helpers/ekf.h"
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
void live_H(double *in_vec, double *out_3066001621377614732);
void live_err_fun(double *nom_x, double *delta_x, double *out_6244428468773599989);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_461346095982138172);
void live_H_mod_fun(double *state, double *out_3446661359688455921);
void live_f_fun(double *state, double dt, double *out_888451463957461958);
void live_F_fun(double *state, double dt, double *out_163486744962538436);
void live_h_4(double *state, double *unused, double *out_6169915102617564504);
void live_H_4(double *state, double *unused, double *out_1090270891370715802);
void live_h_9(double *state, double *unused, double *out_6089758512190470679);
void live_H_9(double *state, double *unused, double *out_6196948043893731668);
void live_h_10(double *state, double *unused, double *out_560336391360125341);
void live_H_10(double *state, double *unused, double *out_7708814297568797326);
void live_h_12(double *state, double *unused, double *out_9147556785756373163);
void live_H_12(double *state, double *unused, double *out_7471529268413448798);
void live_h_35(double *state, double *unused, double *out_6223864458774672270);
void live_H_35(double *state, double *unused, double *out_2276391166001891574);
void live_h_32(double *state, double *unused, double *out_5740266977884607688);
void live_H_32(double *state, double *unused, double *out_2407555950942632043);
void live_h_13(double *state, double *unused, double *out_3623273138008475126);
void live_H_13(double *state, double *unused, double *out_3570360156763088178);
void live_h_14(double *state, double *unused, double *out_6089758512190470679);
void live_H_14(double *state, double *unused, double *out_6196948043893731668);
void live_h_33(double *state, double *unused, double *out_4340102630937086223);
void live_H_33(double *state, double *unused, double *out_5973766614433945613);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
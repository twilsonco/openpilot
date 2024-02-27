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
void car_err_fun(double *nom_x, double *delta_x, double *out_4328417858719399296);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7622011457833195352);
void car_H_mod_fun(double *state, double *out_7531422148562293488);
void car_f_fun(double *state, double dt, double *out_3545004875602579137);
void car_F_fun(double *state, double dt, double *out_2926959116897174320);
void car_h_25(double *state, double *unused, double *out_4206336983505903873);
void car_H_25(double *state, double *unused, double *out_2027678294181840875);
void car_h_24(double *state, double *unused, double *out_6602550213615218626);
void car_H_24(double *state, double *unused, double *out_228169780941976754);
void car_h_30(double *state, double *unused, double *out_7843786511759926223);
void car_H_30(double *state, double *unused, double *out_6555374624309449073);
void car_h_26(double *state, double *unused, double *out_6962450324126076075);
void car_H_26(double *state, double *unused, double *out_5769181613055897099);
void car_h_27(double *state, double *unused, double *out_3049806733587547836);
void car_H_27(double *state, double *unused, double *out_8730137936109873984);
void car_h_29(double *state, double *unused, double *out_3105640462253112293);
void car_H_29(double *state, double *unused, double *out_6045143279995056889);
void car_h_28(double *state, double *unused, double *out_226041079159908753);
void car_H_28(double *state, double *unused, double *out_4081513008429730638);
void car_h_31(double *state, double *unused, double *out_4807755141119267427);
void car_H_31(double *state, double *unused, double *out_6395389715289248575);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
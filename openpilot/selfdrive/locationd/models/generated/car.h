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
void car_err_fun(double *nom_x, double *delta_x, double *out_3010729618614624127);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3985552318855446440);
void car_H_mod_fun(double *state, double *out_4306310304618992202);
void car_f_fun(double *state, double dt, double *out_841039768515563673);
void car_F_fun(double *state, double dt, double *out_3325406224632462615);
void car_h_25(double *state, double *unused, double *out_8501492422434664386);
void car_H_25(double *state, double *unused, double *out_7217655417465113750);
void car_h_24(double *state, double *unused, double *out_2528371558444461081);
void car_H_24(double *state, double *unused, double *out_7688112899508452474);
void car_h_30(double *state, double *unused, double *out_8776686484719170275);
void car_H_30(double *state, double *unused, double *out_6701392326116829668);
void car_h_26(double *state, double *unused, double *out_8173370600042692758);
void car_H_26(double *state, double *unused, double *out_7487585337370381642);
void car_h_27(double *state, double *unused, double *out_4943578383614041585);
void car_H_27(double *state, double *unused, double *out_8924986397300772885);
void car_h_29(double *state, double *unused, double *out_4668384321329535696);
void car_H_29(double *state, double *unused, double *out_7211623670431221852);
void car_h_28(double *state, double *unused, double *out_6247021973002058322);
void car_H_28(double *state, double *unused, double *out_9175253941996548103);
void car_h_31(double *state, double *unused, double *out_1448590970011147238);
void car_H_31(double *state, double *unused, double *out_7187009455588153322);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
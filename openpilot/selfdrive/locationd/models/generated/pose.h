#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_342973137176612241);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4414887224430184744);
void pose_H_mod_fun(double *state, double *out_824354775938542426);
void pose_f_fun(double *state, double dt, double *out_432270031952541382);
void pose_F_fun(double *state, double dt, double *out_6844361959189325516);
void pose_h_4(double *state, double *unused, double *out_948512472116913193);
void pose_H_4(double *state, double *unused, double *out_4098576854729491679);
void pose_h_10(double *state, double *unused, double *out_8780443367168833320);
void pose_H_10(double *state, double *unused, double *out_7022073385523406729);
void pose_h_13(double *state, double *unused, double *out_3854432803415109726);
void pose_H_13(double *state, double *unused, double *out_3512054353587209250);
void pose_h_14(double *state, double *unused, double *out_3383842427869284419);
void pose_H_14(double *state, double *unused, double *out_135335998390007150);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
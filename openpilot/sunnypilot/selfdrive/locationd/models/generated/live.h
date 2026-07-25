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
void live_H(double *in_vec, double *out_6352154163192849580);
void live_err_fun(double *nom_x, double *delta_x, double *out_4071252314637571743);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_7579279316636236983);
void live_H_mod_fun(double *state, double *out_2183030463631189817);
void live_f_fun(double *state, double dt, double *out_5889449294370966489);
void live_F_fun(double *state, double dt, double *out_4590604826483372195);
void live_h_4(double *state, double *unused, double *out_8188582832631334816);
void live_H_4(double *state, double *unused, double *out_7947231586507101220);
void live_h_9(double *state, double *unused, double *out_3642341249638579952);
void live_H_9(double *state, double *unused, double *out_3212293551938002926);
void live_h_10(double *state, double *unused, double *out_2827509996430913912);
void live_H_10(double *state, double *unused, double *out_5851187932200043394);
void live_h_12(double *state, double *unused, double *out_8787305242966137389);
void live_H_12(double *state, double *unused, double *out_5480056079170488601);
void live_h_35(double *state, double *unused, double *out_5904086788787176377);
void live_H_35(double *state, double *unused, double *out_7132850429829843020);
void live_h_32(double *state, double *unused, double *out_4350922156484078074);
void live_H_32(double *state, double *unused, double *out_7095194406207799526);
void live_h_13(double *state, double *unused, double *out_3453680179466203195);
void live_H_13(double *state, double *unused, double *out_2170845890615723155);
void live_h_14(double *state, double *unused, double *out_3642341249638579952);
void live_H_14(double *state, double *unused, double *out_3212293551938002926);
void live_h_33(double *state, double *unused, double *out_8251223619274971083);
void live_H_33(double *state, double *unused, double *out_3982293425190985416);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
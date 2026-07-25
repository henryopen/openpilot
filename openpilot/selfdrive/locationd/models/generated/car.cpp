#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3010729618614624127) {
   out_3010729618614624127[0] = delta_x[0] + nom_x[0];
   out_3010729618614624127[1] = delta_x[1] + nom_x[1];
   out_3010729618614624127[2] = delta_x[2] + nom_x[2];
   out_3010729618614624127[3] = delta_x[3] + nom_x[3];
   out_3010729618614624127[4] = delta_x[4] + nom_x[4];
   out_3010729618614624127[5] = delta_x[5] + nom_x[5];
   out_3010729618614624127[6] = delta_x[6] + nom_x[6];
   out_3010729618614624127[7] = delta_x[7] + nom_x[7];
   out_3010729618614624127[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3985552318855446440) {
   out_3985552318855446440[0] = -nom_x[0] + true_x[0];
   out_3985552318855446440[1] = -nom_x[1] + true_x[1];
   out_3985552318855446440[2] = -nom_x[2] + true_x[2];
   out_3985552318855446440[3] = -nom_x[3] + true_x[3];
   out_3985552318855446440[4] = -nom_x[4] + true_x[4];
   out_3985552318855446440[5] = -nom_x[5] + true_x[5];
   out_3985552318855446440[6] = -nom_x[6] + true_x[6];
   out_3985552318855446440[7] = -nom_x[7] + true_x[7];
   out_3985552318855446440[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4306310304618992202) {
   out_4306310304618992202[0] = 1.0;
   out_4306310304618992202[1] = 0.0;
   out_4306310304618992202[2] = 0.0;
   out_4306310304618992202[3] = 0.0;
   out_4306310304618992202[4] = 0.0;
   out_4306310304618992202[5] = 0.0;
   out_4306310304618992202[6] = 0.0;
   out_4306310304618992202[7] = 0.0;
   out_4306310304618992202[8] = 0.0;
   out_4306310304618992202[9] = 0.0;
   out_4306310304618992202[10] = 1.0;
   out_4306310304618992202[11] = 0.0;
   out_4306310304618992202[12] = 0.0;
   out_4306310304618992202[13] = 0.0;
   out_4306310304618992202[14] = 0.0;
   out_4306310304618992202[15] = 0.0;
   out_4306310304618992202[16] = 0.0;
   out_4306310304618992202[17] = 0.0;
   out_4306310304618992202[18] = 0.0;
   out_4306310304618992202[19] = 0.0;
   out_4306310304618992202[20] = 1.0;
   out_4306310304618992202[21] = 0.0;
   out_4306310304618992202[22] = 0.0;
   out_4306310304618992202[23] = 0.0;
   out_4306310304618992202[24] = 0.0;
   out_4306310304618992202[25] = 0.0;
   out_4306310304618992202[26] = 0.0;
   out_4306310304618992202[27] = 0.0;
   out_4306310304618992202[28] = 0.0;
   out_4306310304618992202[29] = 0.0;
   out_4306310304618992202[30] = 1.0;
   out_4306310304618992202[31] = 0.0;
   out_4306310304618992202[32] = 0.0;
   out_4306310304618992202[33] = 0.0;
   out_4306310304618992202[34] = 0.0;
   out_4306310304618992202[35] = 0.0;
   out_4306310304618992202[36] = 0.0;
   out_4306310304618992202[37] = 0.0;
   out_4306310304618992202[38] = 0.0;
   out_4306310304618992202[39] = 0.0;
   out_4306310304618992202[40] = 1.0;
   out_4306310304618992202[41] = 0.0;
   out_4306310304618992202[42] = 0.0;
   out_4306310304618992202[43] = 0.0;
   out_4306310304618992202[44] = 0.0;
   out_4306310304618992202[45] = 0.0;
   out_4306310304618992202[46] = 0.0;
   out_4306310304618992202[47] = 0.0;
   out_4306310304618992202[48] = 0.0;
   out_4306310304618992202[49] = 0.0;
   out_4306310304618992202[50] = 1.0;
   out_4306310304618992202[51] = 0.0;
   out_4306310304618992202[52] = 0.0;
   out_4306310304618992202[53] = 0.0;
   out_4306310304618992202[54] = 0.0;
   out_4306310304618992202[55] = 0.0;
   out_4306310304618992202[56] = 0.0;
   out_4306310304618992202[57] = 0.0;
   out_4306310304618992202[58] = 0.0;
   out_4306310304618992202[59] = 0.0;
   out_4306310304618992202[60] = 1.0;
   out_4306310304618992202[61] = 0.0;
   out_4306310304618992202[62] = 0.0;
   out_4306310304618992202[63] = 0.0;
   out_4306310304618992202[64] = 0.0;
   out_4306310304618992202[65] = 0.0;
   out_4306310304618992202[66] = 0.0;
   out_4306310304618992202[67] = 0.0;
   out_4306310304618992202[68] = 0.0;
   out_4306310304618992202[69] = 0.0;
   out_4306310304618992202[70] = 1.0;
   out_4306310304618992202[71] = 0.0;
   out_4306310304618992202[72] = 0.0;
   out_4306310304618992202[73] = 0.0;
   out_4306310304618992202[74] = 0.0;
   out_4306310304618992202[75] = 0.0;
   out_4306310304618992202[76] = 0.0;
   out_4306310304618992202[77] = 0.0;
   out_4306310304618992202[78] = 0.0;
   out_4306310304618992202[79] = 0.0;
   out_4306310304618992202[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_841039768515563673) {
   out_841039768515563673[0] = state[0];
   out_841039768515563673[1] = state[1];
   out_841039768515563673[2] = state[2];
   out_841039768515563673[3] = state[3];
   out_841039768515563673[4] = state[4];
   out_841039768515563673[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_841039768515563673[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_841039768515563673[7] = state[7];
   out_841039768515563673[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3325406224632462615) {
   out_3325406224632462615[0] = 1;
   out_3325406224632462615[1] = 0;
   out_3325406224632462615[2] = 0;
   out_3325406224632462615[3] = 0;
   out_3325406224632462615[4] = 0;
   out_3325406224632462615[5] = 0;
   out_3325406224632462615[6] = 0;
   out_3325406224632462615[7] = 0;
   out_3325406224632462615[8] = 0;
   out_3325406224632462615[9] = 0;
   out_3325406224632462615[10] = 1;
   out_3325406224632462615[11] = 0;
   out_3325406224632462615[12] = 0;
   out_3325406224632462615[13] = 0;
   out_3325406224632462615[14] = 0;
   out_3325406224632462615[15] = 0;
   out_3325406224632462615[16] = 0;
   out_3325406224632462615[17] = 0;
   out_3325406224632462615[18] = 0;
   out_3325406224632462615[19] = 0;
   out_3325406224632462615[20] = 1;
   out_3325406224632462615[21] = 0;
   out_3325406224632462615[22] = 0;
   out_3325406224632462615[23] = 0;
   out_3325406224632462615[24] = 0;
   out_3325406224632462615[25] = 0;
   out_3325406224632462615[26] = 0;
   out_3325406224632462615[27] = 0;
   out_3325406224632462615[28] = 0;
   out_3325406224632462615[29] = 0;
   out_3325406224632462615[30] = 1;
   out_3325406224632462615[31] = 0;
   out_3325406224632462615[32] = 0;
   out_3325406224632462615[33] = 0;
   out_3325406224632462615[34] = 0;
   out_3325406224632462615[35] = 0;
   out_3325406224632462615[36] = 0;
   out_3325406224632462615[37] = 0;
   out_3325406224632462615[38] = 0;
   out_3325406224632462615[39] = 0;
   out_3325406224632462615[40] = 1;
   out_3325406224632462615[41] = 0;
   out_3325406224632462615[42] = 0;
   out_3325406224632462615[43] = 0;
   out_3325406224632462615[44] = 0;
   out_3325406224632462615[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3325406224632462615[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3325406224632462615[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3325406224632462615[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3325406224632462615[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3325406224632462615[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3325406224632462615[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3325406224632462615[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3325406224632462615[53] = -9.8100000000000005*dt;
   out_3325406224632462615[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3325406224632462615[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3325406224632462615[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3325406224632462615[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3325406224632462615[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3325406224632462615[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3325406224632462615[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3325406224632462615[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3325406224632462615[62] = 0;
   out_3325406224632462615[63] = 0;
   out_3325406224632462615[64] = 0;
   out_3325406224632462615[65] = 0;
   out_3325406224632462615[66] = 0;
   out_3325406224632462615[67] = 0;
   out_3325406224632462615[68] = 0;
   out_3325406224632462615[69] = 0;
   out_3325406224632462615[70] = 1;
   out_3325406224632462615[71] = 0;
   out_3325406224632462615[72] = 0;
   out_3325406224632462615[73] = 0;
   out_3325406224632462615[74] = 0;
   out_3325406224632462615[75] = 0;
   out_3325406224632462615[76] = 0;
   out_3325406224632462615[77] = 0;
   out_3325406224632462615[78] = 0;
   out_3325406224632462615[79] = 0;
   out_3325406224632462615[80] = 1;
}
void h_25(double *state, double *unused, double *out_8501492422434664386) {
   out_8501492422434664386[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7217655417465113750) {
   out_7217655417465113750[0] = 0;
   out_7217655417465113750[1] = 0;
   out_7217655417465113750[2] = 0;
   out_7217655417465113750[3] = 0;
   out_7217655417465113750[4] = 0;
   out_7217655417465113750[5] = 0;
   out_7217655417465113750[6] = 1;
   out_7217655417465113750[7] = 0;
   out_7217655417465113750[8] = 0;
}
void h_24(double *state, double *unused, double *out_2528371558444461081) {
   out_2528371558444461081[0] = state[4];
   out_2528371558444461081[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7688112899508452474) {
   out_7688112899508452474[0] = 0;
   out_7688112899508452474[1] = 0;
   out_7688112899508452474[2] = 0;
   out_7688112899508452474[3] = 0;
   out_7688112899508452474[4] = 1;
   out_7688112899508452474[5] = 0;
   out_7688112899508452474[6] = 0;
   out_7688112899508452474[7] = 0;
   out_7688112899508452474[8] = 0;
   out_7688112899508452474[9] = 0;
   out_7688112899508452474[10] = 0;
   out_7688112899508452474[11] = 0;
   out_7688112899508452474[12] = 0;
   out_7688112899508452474[13] = 0;
   out_7688112899508452474[14] = 1;
   out_7688112899508452474[15] = 0;
   out_7688112899508452474[16] = 0;
   out_7688112899508452474[17] = 0;
}
void h_30(double *state, double *unused, double *out_8776686484719170275) {
   out_8776686484719170275[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6701392326116829668) {
   out_6701392326116829668[0] = 0;
   out_6701392326116829668[1] = 0;
   out_6701392326116829668[2] = 0;
   out_6701392326116829668[3] = 0;
   out_6701392326116829668[4] = 1;
   out_6701392326116829668[5] = 0;
   out_6701392326116829668[6] = 0;
   out_6701392326116829668[7] = 0;
   out_6701392326116829668[8] = 0;
}
void h_26(double *state, double *unused, double *out_8173370600042692758) {
   out_8173370600042692758[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7487585337370381642) {
   out_7487585337370381642[0] = 0;
   out_7487585337370381642[1] = 0;
   out_7487585337370381642[2] = 0;
   out_7487585337370381642[3] = 0;
   out_7487585337370381642[4] = 0;
   out_7487585337370381642[5] = 0;
   out_7487585337370381642[6] = 0;
   out_7487585337370381642[7] = 1;
   out_7487585337370381642[8] = 0;
}
void h_27(double *state, double *unused, double *out_4943578383614041585) {
   out_4943578383614041585[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8924986397300772885) {
   out_8924986397300772885[0] = 0;
   out_8924986397300772885[1] = 0;
   out_8924986397300772885[2] = 0;
   out_8924986397300772885[3] = 1;
   out_8924986397300772885[4] = 0;
   out_8924986397300772885[5] = 0;
   out_8924986397300772885[6] = 0;
   out_8924986397300772885[7] = 0;
   out_8924986397300772885[8] = 0;
}
void h_29(double *state, double *unused, double *out_4668384321329535696) {
   out_4668384321329535696[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7211623670431221852) {
   out_7211623670431221852[0] = 0;
   out_7211623670431221852[1] = 1;
   out_7211623670431221852[2] = 0;
   out_7211623670431221852[3] = 0;
   out_7211623670431221852[4] = 0;
   out_7211623670431221852[5] = 0;
   out_7211623670431221852[6] = 0;
   out_7211623670431221852[7] = 0;
   out_7211623670431221852[8] = 0;
}
void h_28(double *state, double *unused, double *out_6247021973002058322) {
   out_6247021973002058322[0] = state[0];
}
void H_28(double *state, double *unused, double *out_9175253941996548103) {
   out_9175253941996548103[0] = 1;
   out_9175253941996548103[1] = 0;
   out_9175253941996548103[2] = 0;
   out_9175253941996548103[3] = 0;
   out_9175253941996548103[4] = 0;
   out_9175253941996548103[5] = 0;
   out_9175253941996548103[6] = 0;
   out_9175253941996548103[7] = 0;
   out_9175253941996548103[8] = 0;
}
void h_31(double *state, double *unused, double *out_1448590970011147238) {
   out_1448590970011147238[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7187009455588153322) {
   out_7187009455588153322[0] = 0;
   out_7187009455588153322[1] = 0;
   out_7187009455588153322[2] = 0;
   out_7187009455588153322[3] = 0;
   out_7187009455588153322[4] = 0;
   out_7187009455588153322[5] = 0;
   out_7187009455588153322[6] = 0;
   out_7187009455588153322[7] = 0;
   out_7187009455588153322[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_3010729618614624127) {
  err_fun(nom_x, delta_x, out_3010729618614624127);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3985552318855446440) {
  inv_err_fun(nom_x, true_x, out_3985552318855446440);
}
void car_H_mod_fun(double *state, double *out_4306310304618992202) {
  H_mod_fun(state, out_4306310304618992202);
}
void car_f_fun(double *state, double dt, double *out_841039768515563673) {
  f_fun(state,  dt, out_841039768515563673);
}
void car_F_fun(double *state, double dt, double *out_3325406224632462615) {
  F_fun(state,  dt, out_3325406224632462615);
}
void car_h_25(double *state, double *unused, double *out_8501492422434664386) {
  h_25(state, unused, out_8501492422434664386);
}
void car_H_25(double *state, double *unused, double *out_7217655417465113750) {
  H_25(state, unused, out_7217655417465113750);
}
void car_h_24(double *state, double *unused, double *out_2528371558444461081) {
  h_24(state, unused, out_2528371558444461081);
}
void car_H_24(double *state, double *unused, double *out_7688112899508452474) {
  H_24(state, unused, out_7688112899508452474);
}
void car_h_30(double *state, double *unused, double *out_8776686484719170275) {
  h_30(state, unused, out_8776686484719170275);
}
void car_H_30(double *state, double *unused, double *out_6701392326116829668) {
  H_30(state, unused, out_6701392326116829668);
}
void car_h_26(double *state, double *unused, double *out_8173370600042692758) {
  h_26(state, unused, out_8173370600042692758);
}
void car_H_26(double *state, double *unused, double *out_7487585337370381642) {
  H_26(state, unused, out_7487585337370381642);
}
void car_h_27(double *state, double *unused, double *out_4943578383614041585) {
  h_27(state, unused, out_4943578383614041585);
}
void car_H_27(double *state, double *unused, double *out_8924986397300772885) {
  H_27(state, unused, out_8924986397300772885);
}
void car_h_29(double *state, double *unused, double *out_4668384321329535696) {
  h_29(state, unused, out_4668384321329535696);
}
void car_H_29(double *state, double *unused, double *out_7211623670431221852) {
  H_29(state, unused, out_7211623670431221852);
}
void car_h_28(double *state, double *unused, double *out_6247021973002058322) {
  h_28(state, unused, out_6247021973002058322);
}
void car_H_28(double *state, double *unused, double *out_9175253941996548103) {
  H_28(state, unused, out_9175253941996548103);
}
void car_h_31(double *state, double *unused, double *out_1448590970011147238) {
  h_31(state, unused, out_1448590970011147238);
}
void car_H_31(double *state, double *unused, double *out_7187009455588153322) {
  H_31(state, unused, out_7187009455588153322);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)

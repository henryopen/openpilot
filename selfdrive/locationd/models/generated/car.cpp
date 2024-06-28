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
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5636437553953314028) {
   out_5636437553953314028[0] = delta_x[0] + nom_x[0];
   out_5636437553953314028[1] = delta_x[1] + nom_x[1];
   out_5636437553953314028[2] = delta_x[2] + nom_x[2];
   out_5636437553953314028[3] = delta_x[3] + nom_x[3];
   out_5636437553953314028[4] = delta_x[4] + nom_x[4];
   out_5636437553953314028[5] = delta_x[5] + nom_x[5];
   out_5636437553953314028[6] = delta_x[6] + nom_x[6];
   out_5636437553953314028[7] = delta_x[7] + nom_x[7];
   out_5636437553953314028[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_127342766874432092) {
   out_127342766874432092[0] = -nom_x[0] + true_x[0];
   out_127342766874432092[1] = -nom_x[1] + true_x[1];
   out_127342766874432092[2] = -nom_x[2] + true_x[2];
   out_127342766874432092[3] = -nom_x[3] + true_x[3];
   out_127342766874432092[4] = -nom_x[4] + true_x[4];
   out_127342766874432092[5] = -nom_x[5] + true_x[5];
   out_127342766874432092[6] = -nom_x[6] + true_x[6];
   out_127342766874432092[7] = -nom_x[7] + true_x[7];
   out_127342766874432092[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7881350123582689415) {
   out_7881350123582689415[0] = 1.0;
   out_7881350123582689415[1] = 0;
   out_7881350123582689415[2] = 0;
   out_7881350123582689415[3] = 0;
   out_7881350123582689415[4] = 0;
   out_7881350123582689415[5] = 0;
   out_7881350123582689415[6] = 0;
   out_7881350123582689415[7] = 0;
   out_7881350123582689415[8] = 0;
   out_7881350123582689415[9] = 0;
   out_7881350123582689415[10] = 1.0;
   out_7881350123582689415[11] = 0;
   out_7881350123582689415[12] = 0;
   out_7881350123582689415[13] = 0;
   out_7881350123582689415[14] = 0;
   out_7881350123582689415[15] = 0;
   out_7881350123582689415[16] = 0;
   out_7881350123582689415[17] = 0;
   out_7881350123582689415[18] = 0;
   out_7881350123582689415[19] = 0;
   out_7881350123582689415[20] = 1.0;
   out_7881350123582689415[21] = 0;
   out_7881350123582689415[22] = 0;
   out_7881350123582689415[23] = 0;
   out_7881350123582689415[24] = 0;
   out_7881350123582689415[25] = 0;
   out_7881350123582689415[26] = 0;
   out_7881350123582689415[27] = 0;
   out_7881350123582689415[28] = 0;
   out_7881350123582689415[29] = 0;
   out_7881350123582689415[30] = 1.0;
   out_7881350123582689415[31] = 0;
   out_7881350123582689415[32] = 0;
   out_7881350123582689415[33] = 0;
   out_7881350123582689415[34] = 0;
   out_7881350123582689415[35] = 0;
   out_7881350123582689415[36] = 0;
   out_7881350123582689415[37] = 0;
   out_7881350123582689415[38] = 0;
   out_7881350123582689415[39] = 0;
   out_7881350123582689415[40] = 1.0;
   out_7881350123582689415[41] = 0;
   out_7881350123582689415[42] = 0;
   out_7881350123582689415[43] = 0;
   out_7881350123582689415[44] = 0;
   out_7881350123582689415[45] = 0;
   out_7881350123582689415[46] = 0;
   out_7881350123582689415[47] = 0;
   out_7881350123582689415[48] = 0;
   out_7881350123582689415[49] = 0;
   out_7881350123582689415[50] = 1.0;
   out_7881350123582689415[51] = 0;
   out_7881350123582689415[52] = 0;
   out_7881350123582689415[53] = 0;
   out_7881350123582689415[54] = 0;
   out_7881350123582689415[55] = 0;
   out_7881350123582689415[56] = 0;
   out_7881350123582689415[57] = 0;
   out_7881350123582689415[58] = 0;
   out_7881350123582689415[59] = 0;
   out_7881350123582689415[60] = 1.0;
   out_7881350123582689415[61] = 0;
   out_7881350123582689415[62] = 0;
   out_7881350123582689415[63] = 0;
   out_7881350123582689415[64] = 0;
   out_7881350123582689415[65] = 0;
   out_7881350123582689415[66] = 0;
   out_7881350123582689415[67] = 0;
   out_7881350123582689415[68] = 0;
   out_7881350123582689415[69] = 0;
   out_7881350123582689415[70] = 1.0;
   out_7881350123582689415[71] = 0;
   out_7881350123582689415[72] = 0;
   out_7881350123582689415[73] = 0;
   out_7881350123582689415[74] = 0;
   out_7881350123582689415[75] = 0;
   out_7881350123582689415[76] = 0;
   out_7881350123582689415[77] = 0;
   out_7881350123582689415[78] = 0;
   out_7881350123582689415[79] = 0;
   out_7881350123582689415[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_7862709455467625440) {
   out_7862709455467625440[0] = state[0];
   out_7862709455467625440[1] = state[1];
   out_7862709455467625440[2] = state[2];
   out_7862709455467625440[3] = state[3];
   out_7862709455467625440[4] = state[4];
   out_7862709455467625440[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7862709455467625440[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7862709455467625440[7] = state[7];
   out_7862709455467625440[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8810424645333457135) {
   out_8810424645333457135[0] = 1;
   out_8810424645333457135[1] = 0;
   out_8810424645333457135[2] = 0;
   out_8810424645333457135[3] = 0;
   out_8810424645333457135[4] = 0;
   out_8810424645333457135[5] = 0;
   out_8810424645333457135[6] = 0;
   out_8810424645333457135[7] = 0;
   out_8810424645333457135[8] = 0;
   out_8810424645333457135[9] = 0;
   out_8810424645333457135[10] = 1;
   out_8810424645333457135[11] = 0;
   out_8810424645333457135[12] = 0;
   out_8810424645333457135[13] = 0;
   out_8810424645333457135[14] = 0;
   out_8810424645333457135[15] = 0;
   out_8810424645333457135[16] = 0;
   out_8810424645333457135[17] = 0;
   out_8810424645333457135[18] = 0;
   out_8810424645333457135[19] = 0;
   out_8810424645333457135[20] = 1;
   out_8810424645333457135[21] = 0;
   out_8810424645333457135[22] = 0;
   out_8810424645333457135[23] = 0;
   out_8810424645333457135[24] = 0;
   out_8810424645333457135[25] = 0;
   out_8810424645333457135[26] = 0;
   out_8810424645333457135[27] = 0;
   out_8810424645333457135[28] = 0;
   out_8810424645333457135[29] = 0;
   out_8810424645333457135[30] = 1;
   out_8810424645333457135[31] = 0;
   out_8810424645333457135[32] = 0;
   out_8810424645333457135[33] = 0;
   out_8810424645333457135[34] = 0;
   out_8810424645333457135[35] = 0;
   out_8810424645333457135[36] = 0;
   out_8810424645333457135[37] = 0;
   out_8810424645333457135[38] = 0;
   out_8810424645333457135[39] = 0;
   out_8810424645333457135[40] = 1;
   out_8810424645333457135[41] = 0;
   out_8810424645333457135[42] = 0;
   out_8810424645333457135[43] = 0;
   out_8810424645333457135[44] = 0;
   out_8810424645333457135[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8810424645333457135[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8810424645333457135[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8810424645333457135[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8810424645333457135[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8810424645333457135[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8810424645333457135[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8810424645333457135[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8810424645333457135[53] = -9.8000000000000007*dt;
   out_8810424645333457135[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8810424645333457135[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8810424645333457135[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8810424645333457135[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8810424645333457135[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8810424645333457135[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8810424645333457135[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8810424645333457135[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8810424645333457135[62] = 0;
   out_8810424645333457135[63] = 0;
   out_8810424645333457135[64] = 0;
   out_8810424645333457135[65] = 0;
   out_8810424645333457135[66] = 0;
   out_8810424645333457135[67] = 0;
   out_8810424645333457135[68] = 0;
   out_8810424645333457135[69] = 0;
   out_8810424645333457135[70] = 1;
   out_8810424645333457135[71] = 0;
   out_8810424645333457135[72] = 0;
   out_8810424645333457135[73] = 0;
   out_8810424645333457135[74] = 0;
   out_8810424645333457135[75] = 0;
   out_8810424645333457135[76] = 0;
   out_8810424645333457135[77] = 0;
   out_8810424645333457135[78] = 0;
   out_8810424645333457135[79] = 0;
   out_8810424645333457135[80] = 1;
}
void h_25(double *state, double *unused, double *out_3942377437605520571) {
   out_3942377437605520571[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7114611326874360129) {
   out_7114611326874360129[0] = 0;
   out_7114611326874360129[1] = 0;
   out_7114611326874360129[2] = 0;
   out_7114611326874360129[3] = 0;
   out_7114611326874360129[4] = 0;
   out_7114611326874360129[5] = 0;
   out_7114611326874360129[6] = 1;
   out_7114611326874360129[7] = 0;
   out_7114611326874360129[8] = 0;
}
void h_24(double *state, double *unused, double *out_916649066520717673) {
   out_916649066520717673[0] = state[4];
   out_916649066520717673[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7349809595437423589) {
   out_7349809595437423589[0] = 0;
   out_7349809595437423589[1] = 0;
   out_7349809595437423589[2] = 0;
   out_7349809595437423589[3] = 0;
   out_7349809595437423589[4] = 1;
   out_7349809595437423589[5] = 0;
   out_7349809595437423589[6] = 0;
   out_7349809595437423589[7] = 0;
   out_7349809595437423589[8] = 0;
   out_7349809595437423589[9] = 0;
   out_7349809595437423589[10] = 0;
   out_7349809595437423589[11] = 0;
   out_7349809595437423589[12] = 0;
   out_7349809595437423589[13] = 0;
   out_7349809595437423589[14] = 1;
   out_7349809595437423589[15] = 0;
   out_7349809595437423589[16] = 0;
   out_7349809595437423589[17] = 0;
}
void h_30(double *state, double *unused, double *out_2645450488870774592) {
   out_2645450488870774592[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4596278368367111502) {
   out_4596278368367111502[0] = 0;
   out_4596278368367111502[1] = 0;
   out_4596278368367111502[2] = 0;
   out_4596278368367111502[3] = 0;
   out_4596278368367111502[4] = 1;
   out_4596278368367111502[5] = 0;
   out_4596278368367111502[6] = 0;
   out_4596278368367111502[7] = 0;
   out_4596278368367111502[8] = 0;
}
void h_26(double *state, double *unused, double *out_8901275210829821846) {
   out_8901275210829821846[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7590629427961135263) {
   out_7590629427961135263[0] = 0;
   out_7590629427961135263[1] = 0;
   out_7590629427961135263[2] = 0;
   out_7590629427961135263[3] = 0;
   out_7590629427961135263[4] = 0;
   out_7590629427961135263[5] = 0;
   out_7590629427961135263[6] = 0;
   out_7590629427961135263[7] = 1;
   out_7590629427961135263[8] = 0;
}
void h_27(double *state, double *unused, double *out_1059295830055325074) {
   out_1059295830055325074[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2372684297183168285) {
   out_2372684297183168285[0] = 0;
   out_2372684297183168285[1] = 0;
   out_2372684297183168285[2] = 0;
   out_2372684297183168285[3] = 1;
   out_2372684297183168285[4] = 0;
   out_2372684297183168285[5] = 0;
   out_2372684297183168285[6] = 0;
   out_2372684297183168285[7] = 0;
   out_2372684297183168285[8] = 0;
}
void h_29(double *state, double *unused, double *out_7647123756531620237) {
   out_7647123756531620237[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4086047024052719318) {
   out_4086047024052719318[0] = 0;
   out_4086047024052719318[1] = 1;
   out_4086047024052719318[2] = 0;
   out_4086047024052719318[3] = 0;
   out_4086047024052719318[4] = 0;
   out_4086047024052719318[5] = 0;
   out_4086047024052719318[6] = 0;
   out_4086047024052719318[7] = 0;
   out_4086047024052719318[8] = 0;
}
void h_28(double *state, double *unused, double *out_5681481859511433388) {
   out_5681481859511433388[0] = state[0];
}
void H_28(double *state, double *unused, double *out_9168446041122249892) {
   out_9168446041122249892[0] = 1;
   out_9168446041122249892[1] = 0;
   out_9168446041122249892[2] = 0;
   out_9168446041122249892[3] = 0;
   out_9168446041122249892[4] = 0;
   out_9168446041122249892[5] = 0;
   out_9168446041122249892[6] = 0;
   out_9168446041122249892[7] = 0;
   out_9168446041122249892[8] = 0;
}
void h_31(double *state, double *unused, double *out_3820887819215151870) {
   out_3820887819215151870[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6964421325727783787) {
   out_6964421325727783787[0] = 0;
   out_6964421325727783787[1] = 0;
   out_6964421325727783787[2] = 0;
   out_6964421325727783787[3] = 0;
   out_6964421325727783787[4] = 0;
   out_6964421325727783787[5] = 0;
   out_6964421325727783787[6] = 0;
   out_6964421325727783787[7] = 0;
   out_6964421325727783787[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5636437553953314028) {
  err_fun(nom_x, delta_x, out_5636437553953314028);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_127342766874432092) {
  inv_err_fun(nom_x, true_x, out_127342766874432092);
}
void car_H_mod_fun(double *state, double *out_7881350123582689415) {
  H_mod_fun(state, out_7881350123582689415);
}
void car_f_fun(double *state, double dt, double *out_7862709455467625440) {
  f_fun(state,  dt, out_7862709455467625440);
}
void car_F_fun(double *state, double dt, double *out_8810424645333457135) {
  F_fun(state,  dt, out_8810424645333457135);
}
void car_h_25(double *state, double *unused, double *out_3942377437605520571) {
  h_25(state, unused, out_3942377437605520571);
}
void car_H_25(double *state, double *unused, double *out_7114611326874360129) {
  H_25(state, unused, out_7114611326874360129);
}
void car_h_24(double *state, double *unused, double *out_916649066520717673) {
  h_24(state, unused, out_916649066520717673);
}
void car_H_24(double *state, double *unused, double *out_7349809595437423589) {
  H_24(state, unused, out_7349809595437423589);
}
void car_h_30(double *state, double *unused, double *out_2645450488870774592) {
  h_30(state, unused, out_2645450488870774592);
}
void car_H_30(double *state, double *unused, double *out_4596278368367111502) {
  H_30(state, unused, out_4596278368367111502);
}
void car_h_26(double *state, double *unused, double *out_8901275210829821846) {
  h_26(state, unused, out_8901275210829821846);
}
void car_H_26(double *state, double *unused, double *out_7590629427961135263) {
  H_26(state, unused, out_7590629427961135263);
}
void car_h_27(double *state, double *unused, double *out_1059295830055325074) {
  h_27(state, unused, out_1059295830055325074);
}
void car_H_27(double *state, double *unused, double *out_2372684297183168285) {
  H_27(state, unused, out_2372684297183168285);
}
void car_h_29(double *state, double *unused, double *out_7647123756531620237) {
  h_29(state, unused, out_7647123756531620237);
}
void car_H_29(double *state, double *unused, double *out_4086047024052719318) {
  H_29(state, unused, out_4086047024052719318);
}
void car_h_28(double *state, double *unused, double *out_5681481859511433388) {
  h_28(state, unused, out_5681481859511433388);
}
void car_H_28(double *state, double *unused, double *out_9168446041122249892) {
  H_28(state, unused, out_9168446041122249892);
}
void car_h_31(double *state, double *unused, double *out_3820887819215151870) {
  h_31(state, unused, out_3820887819215151870);
}
void car_H_31(double *state, double *unused, double *out_6964421325727783787) {
  H_31(state, unused, out_6964421325727783787);
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

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
 *                      Code generated with SymPy 1.10.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7738871226124743511) {
   out_7738871226124743511[0] = delta_x[0] + nom_x[0];
   out_7738871226124743511[1] = delta_x[1] + nom_x[1];
   out_7738871226124743511[2] = delta_x[2] + nom_x[2];
   out_7738871226124743511[3] = delta_x[3] + nom_x[3];
   out_7738871226124743511[4] = delta_x[4] + nom_x[4];
   out_7738871226124743511[5] = delta_x[5] + nom_x[5];
   out_7738871226124743511[6] = delta_x[6] + nom_x[6];
   out_7738871226124743511[7] = delta_x[7] + nom_x[7];
   out_7738871226124743511[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1257810621133578395) {
   out_1257810621133578395[0] = -nom_x[0] + true_x[0];
   out_1257810621133578395[1] = -nom_x[1] + true_x[1];
   out_1257810621133578395[2] = -nom_x[2] + true_x[2];
   out_1257810621133578395[3] = -nom_x[3] + true_x[3];
   out_1257810621133578395[4] = -nom_x[4] + true_x[4];
   out_1257810621133578395[5] = -nom_x[5] + true_x[5];
   out_1257810621133578395[6] = -nom_x[6] + true_x[6];
   out_1257810621133578395[7] = -nom_x[7] + true_x[7];
   out_1257810621133578395[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1082627477705123836) {
   out_1082627477705123836[0] = 1.0;
   out_1082627477705123836[1] = 0;
   out_1082627477705123836[2] = 0;
   out_1082627477705123836[3] = 0;
   out_1082627477705123836[4] = 0;
   out_1082627477705123836[5] = 0;
   out_1082627477705123836[6] = 0;
   out_1082627477705123836[7] = 0;
   out_1082627477705123836[8] = 0;
   out_1082627477705123836[9] = 0;
   out_1082627477705123836[10] = 1.0;
   out_1082627477705123836[11] = 0;
   out_1082627477705123836[12] = 0;
   out_1082627477705123836[13] = 0;
   out_1082627477705123836[14] = 0;
   out_1082627477705123836[15] = 0;
   out_1082627477705123836[16] = 0;
   out_1082627477705123836[17] = 0;
   out_1082627477705123836[18] = 0;
   out_1082627477705123836[19] = 0;
   out_1082627477705123836[20] = 1.0;
   out_1082627477705123836[21] = 0;
   out_1082627477705123836[22] = 0;
   out_1082627477705123836[23] = 0;
   out_1082627477705123836[24] = 0;
   out_1082627477705123836[25] = 0;
   out_1082627477705123836[26] = 0;
   out_1082627477705123836[27] = 0;
   out_1082627477705123836[28] = 0;
   out_1082627477705123836[29] = 0;
   out_1082627477705123836[30] = 1.0;
   out_1082627477705123836[31] = 0;
   out_1082627477705123836[32] = 0;
   out_1082627477705123836[33] = 0;
   out_1082627477705123836[34] = 0;
   out_1082627477705123836[35] = 0;
   out_1082627477705123836[36] = 0;
   out_1082627477705123836[37] = 0;
   out_1082627477705123836[38] = 0;
   out_1082627477705123836[39] = 0;
   out_1082627477705123836[40] = 1.0;
   out_1082627477705123836[41] = 0;
   out_1082627477705123836[42] = 0;
   out_1082627477705123836[43] = 0;
   out_1082627477705123836[44] = 0;
   out_1082627477705123836[45] = 0;
   out_1082627477705123836[46] = 0;
   out_1082627477705123836[47] = 0;
   out_1082627477705123836[48] = 0;
   out_1082627477705123836[49] = 0;
   out_1082627477705123836[50] = 1.0;
   out_1082627477705123836[51] = 0;
   out_1082627477705123836[52] = 0;
   out_1082627477705123836[53] = 0;
   out_1082627477705123836[54] = 0;
   out_1082627477705123836[55] = 0;
   out_1082627477705123836[56] = 0;
   out_1082627477705123836[57] = 0;
   out_1082627477705123836[58] = 0;
   out_1082627477705123836[59] = 0;
   out_1082627477705123836[60] = 1.0;
   out_1082627477705123836[61] = 0;
   out_1082627477705123836[62] = 0;
   out_1082627477705123836[63] = 0;
   out_1082627477705123836[64] = 0;
   out_1082627477705123836[65] = 0;
   out_1082627477705123836[66] = 0;
   out_1082627477705123836[67] = 0;
   out_1082627477705123836[68] = 0;
   out_1082627477705123836[69] = 0;
   out_1082627477705123836[70] = 1.0;
   out_1082627477705123836[71] = 0;
   out_1082627477705123836[72] = 0;
   out_1082627477705123836[73] = 0;
   out_1082627477705123836[74] = 0;
   out_1082627477705123836[75] = 0;
   out_1082627477705123836[76] = 0;
   out_1082627477705123836[77] = 0;
   out_1082627477705123836[78] = 0;
   out_1082627477705123836[79] = 0;
   out_1082627477705123836[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8060049046834174365) {
   out_8060049046834174365[0] = state[0];
   out_8060049046834174365[1] = state[1];
   out_8060049046834174365[2] = state[2];
   out_8060049046834174365[3] = state[3];
   out_8060049046834174365[4] = state[4];
   out_8060049046834174365[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8060049046834174365[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8060049046834174365[7] = state[7];
   out_8060049046834174365[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5342944527585928037) {
   out_5342944527585928037[0] = 1;
   out_5342944527585928037[1] = 0;
   out_5342944527585928037[2] = 0;
   out_5342944527585928037[3] = 0;
   out_5342944527585928037[4] = 0;
   out_5342944527585928037[5] = 0;
   out_5342944527585928037[6] = 0;
   out_5342944527585928037[7] = 0;
   out_5342944527585928037[8] = 0;
   out_5342944527585928037[9] = 0;
   out_5342944527585928037[10] = 1;
   out_5342944527585928037[11] = 0;
   out_5342944527585928037[12] = 0;
   out_5342944527585928037[13] = 0;
   out_5342944527585928037[14] = 0;
   out_5342944527585928037[15] = 0;
   out_5342944527585928037[16] = 0;
   out_5342944527585928037[17] = 0;
   out_5342944527585928037[18] = 0;
   out_5342944527585928037[19] = 0;
   out_5342944527585928037[20] = 1;
   out_5342944527585928037[21] = 0;
   out_5342944527585928037[22] = 0;
   out_5342944527585928037[23] = 0;
   out_5342944527585928037[24] = 0;
   out_5342944527585928037[25] = 0;
   out_5342944527585928037[26] = 0;
   out_5342944527585928037[27] = 0;
   out_5342944527585928037[28] = 0;
   out_5342944527585928037[29] = 0;
   out_5342944527585928037[30] = 1;
   out_5342944527585928037[31] = 0;
   out_5342944527585928037[32] = 0;
   out_5342944527585928037[33] = 0;
   out_5342944527585928037[34] = 0;
   out_5342944527585928037[35] = 0;
   out_5342944527585928037[36] = 0;
   out_5342944527585928037[37] = 0;
   out_5342944527585928037[38] = 0;
   out_5342944527585928037[39] = 0;
   out_5342944527585928037[40] = 1;
   out_5342944527585928037[41] = 0;
   out_5342944527585928037[42] = 0;
   out_5342944527585928037[43] = 0;
   out_5342944527585928037[44] = 0;
   out_5342944527585928037[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5342944527585928037[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5342944527585928037[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5342944527585928037[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5342944527585928037[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5342944527585928037[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5342944527585928037[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5342944527585928037[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5342944527585928037[53] = -9.8000000000000007*dt;
   out_5342944527585928037[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5342944527585928037[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5342944527585928037[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5342944527585928037[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5342944527585928037[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5342944527585928037[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5342944527585928037[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5342944527585928037[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5342944527585928037[62] = 0;
   out_5342944527585928037[63] = 0;
   out_5342944527585928037[64] = 0;
   out_5342944527585928037[65] = 0;
   out_5342944527585928037[66] = 0;
   out_5342944527585928037[67] = 0;
   out_5342944527585928037[68] = 0;
   out_5342944527585928037[69] = 0;
   out_5342944527585928037[70] = 1;
   out_5342944527585928037[71] = 0;
   out_5342944527585928037[72] = 0;
   out_5342944527585928037[73] = 0;
   out_5342944527585928037[74] = 0;
   out_5342944527585928037[75] = 0;
   out_5342944527585928037[76] = 0;
   out_5342944527585928037[77] = 0;
   out_5342944527585928037[78] = 0;
   out_5342944527585928037[79] = 0;
   out_5342944527585928037[80] = 1;
}
void h_25(double *state, double *unused, double *out_2560243035918826463) {
   out_2560243035918826463[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2236407327837830560) {
   out_2236407327837830560[0] = 0;
   out_2236407327837830560[1] = 0;
   out_2236407327837830560[2] = 0;
   out_2236407327837830560[3] = 0;
   out_2236407327837830560[4] = 0;
   out_2236407327837830560[5] = 0;
   out_2236407327837830560[6] = 1;
   out_2236407327837830560[7] = 0;
   out_2236407327837830560[8] = 0;
}
void h_24(double *state, double *unused, double *out_5994894285391379092) {
   out_5994894285391379092[0] = state[4];
   out_5994894285391379092[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1395975543655785568) {
   out_1395975543655785568[0] = 0;
   out_1395975543655785568[1] = 0;
   out_1395975543655785568[2] = 0;
   out_1395975543655785568[3] = 0;
   out_1395975543655785568[4] = 1;
   out_1395975543655785568[5] = 0;
   out_1395975543655785568[6] = 0;
   out_1395975543655785568[7] = 0;
   out_1395975543655785568[8] = 0;
   out_1395975543655785568[9] = 0;
   out_1395975543655785568[10] = 0;
   out_1395975543655785568[11] = 0;
   out_1395975543655785568[12] = 0;
   out_1395975543655785568[13] = 0;
   out_1395975543655785568[14] = 1;
   out_1395975543655785568[15] = 0;
   out_1395975543655785568[16] = 0;
   out_1395975543655785568[17] = 0;
}
void h_30(double *state, double *unused, double *out_3385895141706669319) {
   out_3385895141706669319[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2291289002289777638) {
   out_2291289002289777638[0] = 0;
   out_2291289002289777638[1] = 0;
   out_2291289002289777638[2] = 0;
   out_2291289002289777638[3] = 0;
   out_2291289002289777638[4] = 1;
   out_2291289002289777638[5] = 0;
   out_2291289002289777638[6] = 0;
   out_2291289002289777638[7] = 0;
   out_2291289002289777638[8] = 0;
}
void h_26(double *state, double *unused, double *out_7783847222988298331) {
   out_7783847222988298331[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1505095991036225664) {
   out_1505095991036225664[0] = 0;
   out_1505095991036225664[1] = 0;
   out_1505095991036225664[2] = 0;
   out_1505095991036225664[3] = 0;
   out_1505095991036225664[4] = 0;
   out_1505095991036225664[5] = 0;
   out_1505095991036225664[6] = 0;
   out_1505095991036225664[7] = 1;
   out_1505095991036225664[8] = 0;
}
void h_27(double *state, double *unused, double *out_2441430231742019182) {
   out_2441430231742019182[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4466052314090202549) {
   out_4466052314090202549[0] = 0;
   out_4466052314090202549[1] = 0;
   out_4466052314090202549[2] = 0;
   out_4466052314090202549[3] = 1;
   out_4466052314090202549[4] = 0;
   out_4466052314090202549[5] = 0;
   out_4466052314090202549[6] = 0;
   out_4466052314090202549[7] = 0;
   out_4466052314090202549[8] = 0;
}
void h_29(double *state, double *unused, double *out_2716624294026525071) {
   out_2716624294026525071[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1781057657975385454) {
   out_1781057657975385454[0] = 0;
   out_1781057657975385454[1] = 1;
   out_1781057657975385454[2] = 0;
   out_1781057657975385454[3] = 0;
   out_1781057657975385454[4] = 0;
   out_1781057657975385454[5] = 0;
   out_1781057657975385454[6] = 0;
   out_1781057657975385454[7] = 0;
   out_1781057657975385454[8] = 0;
}
void h_28(double *state, double *unused, double *out_7443103499402864827) {
   out_7443103499402864827[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6863456675044916028) {
   out_6863456675044916028[0] = 1;
   out_6863456675044916028[1] = 0;
   out_6863456675044916028[2] = 0;
   out_6863456675044916028[3] = 0;
   out_6863456675044916028[4] = 0;
   out_6863456675044916028[5] = 0;
   out_6863456675044916028[6] = 0;
   out_6863456675044916028[7] = 0;
   out_6863456675044916028[8] = 0;
}
void h_31(double *state, double *unused, double *out_5203022220901845978) {
   out_5203022220901845978[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2131304093269577140) {
   out_2131304093269577140[0] = 0;
   out_2131304093269577140[1] = 0;
   out_2131304093269577140[2] = 0;
   out_2131304093269577140[3] = 0;
   out_2131304093269577140[4] = 0;
   out_2131304093269577140[5] = 0;
   out_2131304093269577140[6] = 0;
   out_2131304093269577140[7] = 0;
   out_2131304093269577140[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7738871226124743511) {
  err_fun(nom_x, delta_x, out_7738871226124743511);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1257810621133578395) {
  inv_err_fun(nom_x, true_x, out_1257810621133578395);
}
void car_H_mod_fun(double *state, double *out_1082627477705123836) {
  H_mod_fun(state, out_1082627477705123836);
}
void car_f_fun(double *state, double dt, double *out_8060049046834174365) {
  f_fun(state,  dt, out_8060049046834174365);
}
void car_F_fun(double *state, double dt, double *out_5342944527585928037) {
  F_fun(state,  dt, out_5342944527585928037);
}
void car_h_25(double *state, double *unused, double *out_2560243035918826463) {
  h_25(state, unused, out_2560243035918826463);
}
void car_H_25(double *state, double *unused, double *out_2236407327837830560) {
  H_25(state, unused, out_2236407327837830560);
}
void car_h_24(double *state, double *unused, double *out_5994894285391379092) {
  h_24(state, unused, out_5994894285391379092);
}
void car_H_24(double *state, double *unused, double *out_1395975543655785568) {
  H_24(state, unused, out_1395975543655785568);
}
void car_h_30(double *state, double *unused, double *out_3385895141706669319) {
  h_30(state, unused, out_3385895141706669319);
}
void car_H_30(double *state, double *unused, double *out_2291289002289777638) {
  H_30(state, unused, out_2291289002289777638);
}
void car_h_26(double *state, double *unused, double *out_7783847222988298331) {
  h_26(state, unused, out_7783847222988298331);
}
void car_H_26(double *state, double *unused, double *out_1505095991036225664) {
  H_26(state, unused, out_1505095991036225664);
}
void car_h_27(double *state, double *unused, double *out_2441430231742019182) {
  h_27(state, unused, out_2441430231742019182);
}
void car_H_27(double *state, double *unused, double *out_4466052314090202549) {
  H_27(state, unused, out_4466052314090202549);
}
void car_h_29(double *state, double *unused, double *out_2716624294026525071) {
  h_29(state, unused, out_2716624294026525071);
}
void car_H_29(double *state, double *unused, double *out_1781057657975385454) {
  H_29(state, unused, out_1781057657975385454);
}
void car_h_28(double *state, double *unused, double *out_7443103499402864827) {
  h_28(state, unused, out_7443103499402864827);
}
void car_H_28(double *state, double *unused, double *out_6863456675044916028) {
  H_28(state, unused, out_6863456675044916028);
}
void car_h_31(double *state, double *unused, double *out_5203022220901845978) {
  h_31(state, unused, out_5203022220901845978);
}
void car_H_31(double *state, double *unused, double *out_2131304093269577140) {
  H_31(state, unused, out_2131304093269577140);
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

ekf_init(car);

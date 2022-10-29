#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.10.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_9152697212499809699) {
   out_9152697212499809699[0] = delta_x[0] + nom_x[0];
   out_9152697212499809699[1] = delta_x[1] + nom_x[1];
   out_9152697212499809699[2] = delta_x[2] + nom_x[2];
   out_9152697212499809699[3] = delta_x[3] + nom_x[3];
   out_9152697212499809699[4] = delta_x[4] + nom_x[4];
   out_9152697212499809699[5] = delta_x[5] + nom_x[5];
   out_9152697212499809699[6] = delta_x[6] + nom_x[6];
   out_9152697212499809699[7] = delta_x[7] + nom_x[7];
   out_9152697212499809699[8] = delta_x[8] + nom_x[8];
   out_9152697212499809699[9] = delta_x[9] + nom_x[9];
   out_9152697212499809699[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6641208110102973198) {
   out_6641208110102973198[0] = -nom_x[0] + true_x[0];
   out_6641208110102973198[1] = -nom_x[1] + true_x[1];
   out_6641208110102973198[2] = -nom_x[2] + true_x[2];
   out_6641208110102973198[3] = -nom_x[3] + true_x[3];
   out_6641208110102973198[4] = -nom_x[4] + true_x[4];
   out_6641208110102973198[5] = -nom_x[5] + true_x[5];
   out_6641208110102973198[6] = -nom_x[6] + true_x[6];
   out_6641208110102973198[7] = -nom_x[7] + true_x[7];
   out_6641208110102973198[8] = -nom_x[8] + true_x[8];
   out_6641208110102973198[9] = -nom_x[9] + true_x[9];
   out_6641208110102973198[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_4882180096896426461) {
   out_4882180096896426461[0] = 1.0;
   out_4882180096896426461[1] = 0;
   out_4882180096896426461[2] = 0;
   out_4882180096896426461[3] = 0;
   out_4882180096896426461[4] = 0;
   out_4882180096896426461[5] = 0;
   out_4882180096896426461[6] = 0;
   out_4882180096896426461[7] = 0;
   out_4882180096896426461[8] = 0;
   out_4882180096896426461[9] = 0;
   out_4882180096896426461[10] = 0;
   out_4882180096896426461[11] = 0;
   out_4882180096896426461[12] = 1.0;
   out_4882180096896426461[13] = 0;
   out_4882180096896426461[14] = 0;
   out_4882180096896426461[15] = 0;
   out_4882180096896426461[16] = 0;
   out_4882180096896426461[17] = 0;
   out_4882180096896426461[18] = 0;
   out_4882180096896426461[19] = 0;
   out_4882180096896426461[20] = 0;
   out_4882180096896426461[21] = 0;
   out_4882180096896426461[22] = 0;
   out_4882180096896426461[23] = 0;
   out_4882180096896426461[24] = 1.0;
   out_4882180096896426461[25] = 0;
   out_4882180096896426461[26] = 0;
   out_4882180096896426461[27] = 0;
   out_4882180096896426461[28] = 0;
   out_4882180096896426461[29] = 0;
   out_4882180096896426461[30] = 0;
   out_4882180096896426461[31] = 0;
   out_4882180096896426461[32] = 0;
   out_4882180096896426461[33] = 0;
   out_4882180096896426461[34] = 0;
   out_4882180096896426461[35] = 0;
   out_4882180096896426461[36] = 1.0;
   out_4882180096896426461[37] = 0;
   out_4882180096896426461[38] = 0;
   out_4882180096896426461[39] = 0;
   out_4882180096896426461[40] = 0;
   out_4882180096896426461[41] = 0;
   out_4882180096896426461[42] = 0;
   out_4882180096896426461[43] = 0;
   out_4882180096896426461[44] = 0;
   out_4882180096896426461[45] = 0;
   out_4882180096896426461[46] = 0;
   out_4882180096896426461[47] = 0;
   out_4882180096896426461[48] = 1.0;
   out_4882180096896426461[49] = 0;
   out_4882180096896426461[50] = 0;
   out_4882180096896426461[51] = 0;
   out_4882180096896426461[52] = 0;
   out_4882180096896426461[53] = 0;
   out_4882180096896426461[54] = 0;
   out_4882180096896426461[55] = 0;
   out_4882180096896426461[56] = 0;
   out_4882180096896426461[57] = 0;
   out_4882180096896426461[58] = 0;
   out_4882180096896426461[59] = 0;
   out_4882180096896426461[60] = 1.0;
   out_4882180096896426461[61] = 0;
   out_4882180096896426461[62] = 0;
   out_4882180096896426461[63] = 0;
   out_4882180096896426461[64] = 0;
   out_4882180096896426461[65] = 0;
   out_4882180096896426461[66] = 0;
   out_4882180096896426461[67] = 0;
   out_4882180096896426461[68] = 0;
   out_4882180096896426461[69] = 0;
   out_4882180096896426461[70] = 0;
   out_4882180096896426461[71] = 0;
   out_4882180096896426461[72] = 1.0;
   out_4882180096896426461[73] = 0;
   out_4882180096896426461[74] = 0;
   out_4882180096896426461[75] = 0;
   out_4882180096896426461[76] = 0;
   out_4882180096896426461[77] = 0;
   out_4882180096896426461[78] = 0;
   out_4882180096896426461[79] = 0;
   out_4882180096896426461[80] = 0;
   out_4882180096896426461[81] = 0;
   out_4882180096896426461[82] = 0;
   out_4882180096896426461[83] = 0;
   out_4882180096896426461[84] = 1.0;
   out_4882180096896426461[85] = 0;
   out_4882180096896426461[86] = 0;
   out_4882180096896426461[87] = 0;
   out_4882180096896426461[88] = 0;
   out_4882180096896426461[89] = 0;
   out_4882180096896426461[90] = 0;
   out_4882180096896426461[91] = 0;
   out_4882180096896426461[92] = 0;
   out_4882180096896426461[93] = 0;
   out_4882180096896426461[94] = 0;
   out_4882180096896426461[95] = 0;
   out_4882180096896426461[96] = 1.0;
   out_4882180096896426461[97] = 0;
   out_4882180096896426461[98] = 0;
   out_4882180096896426461[99] = 0;
   out_4882180096896426461[100] = 0;
   out_4882180096896426461[101] = 0;
   out_4882180096896426461[102] = 0;
   out_4882180096896426461[103] = 0;
   out_4882180096896426461[104] = 0;
   out_4882180096896426461[105] = 0;
   out_4882180096896426461[106] = 0;
   out_4882180096896426461[107] = 0;
   out_4882180096896426461[108] = 1.0;
   out_4882180096896426461[109] = 0;
   out_4882180096896426461[110] = 0;
   out_4882180096896426461[111] = 0;
   out_4882180096896426461[112] = 0;
   out_4882180096896426461[113] = 0;
   out_4882180096896426461[114] = 0;
   out_4882180096896426461[115] = 0;
   out_4882180096896426461[116] = 0;
   out_4882180096896426461[117] = 0;
   out_4882180096896426461[118] = 0;
   out_4882180096896426461[119] = 0;
   out_4882180096896426461[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_8139212335340482105) {
   out_8139212335340482105[0] = dt*state[3] + state[0];
   out_8139212335340482105[1] = dt*state[4] + state[1];
   out_8139212335340482105[2] = dt*state[5] + state[2];
   out_8139212335340482105[3] = state[3];
   out_8139212335340482105[4] = state[4];
   out_8139212335340482105[5] = state[5];
   out_8139212335340482105[6] = dt*state[7] + state[6];
   out_8139212335340482105[7] = dt*state[8] + state[7];
   out_8139212335340482105[8] = state[8];
   out_8139212335340482105[9] = state[9];
   out_8139212335340482105[10] = state[10];
}
void F_fun(double *state, double dt, double *out_2110048490497678022) {
   out_2110048490497678022[0] = 1;
   out_2110048490497678022[1] = 0;
   out_2110048490497678022[2] = 0;
   out_2110048490497678022[3] = dt;
   out_2110048490497678022[4] = 0;
   out_2110048490497678022[5] = 0;
   out_2110048490497678022[6] = 0;
   out_2110048490497678022[7] = 0;
   out_2110048490497678022[8] = 0;
   out_2110048490497678022[9] = 0;
   out_2110048490497678022[10] = 0;
   out_2110048490497678022[11] = 0;
   out_2110048490497678022[12] = 1;
   out_2110048490497678022[13] = 0;
   out_2110048490497678022[14] = 0;
   out_2110048490497678022[15] = dt;
   out_2110048490497678022[16] = 0;
   out_2110048490497678022[17] = 0;
   out_2110048490497678022[18] = 0;
   out_2110048490497678022[19] = 0;
   out_2110048490497678022[20] = 0;
   out_2110048490497678022[21] = 0;
   out_2110048490497678022[22] = 0;
   out_2110048490497678022[23] = 0;
   out_2110048490497678022[24] = 1;
   out_2110048490497678022[25] = 0;
   out_2110048490497678022[26] = 0;
   out_2110048490497678022[27] = dt;
   out_2110048490497678022[28] = 0;
   out_2110048490497678022[29] = 0;
   out_2110048490497678022[30] = 0;
   out_2110048490497678022[31] = 0;
   out_2110048490497678022[32] = 0;
   out_2110048490497678022[33] = 0;
   out_2110048490497678022[34] = 0;
   out_2110048490497678022[35] = 0;
   out_2110048490497678022[36] = 1;
   out_2110048490497678022[37] = 0;
   out_2110048490497678022[38] = 0;
   out_2110048490497678022[39] = 0;
   out_2110048490497678022[40] = 0;
   out_2110048490497678022[41] = 0;
   out_2110048490497678022[42] = 0;
   out_2110048490497678022[43] = 0;
   out_2110048490497678022[44] = 0;
   out_2110048490497678022[45] = 0;
   out_2110048490497678022[46] = 0;
   out_2110048490497678022[47] = 0;
   out_2110048490497678022[48] = 1;
   out_2110048490497678022[49] = 0;
   out_2110048490497678022[50] = 0;
   out_2110048490497678022[51] = 0;
   out_2110048490497678022[52] = 0;
   out_2110048490497678022[53] = 0;
   out_2110048490497678022[54] = 0;
   out_2110048490497678022[55] = 0;
   out_2110048490497678022[56] = 0;
   out_2110048490497678022[57] = 0;
   out_2110048490497678022[58] = 0;
   out_2110048490497678022[59] = 0;
   out_2110048490497678022[60] = 1;
   out_2110048490497678022[61] = 0;
   out_2110048490497678022[62] = 0;
   out_2110048490497678022[63] = 0;
   out_2110048490497678022[64] = 0;
   out_2110048490497678022[65] = 0;
   out_2110048490497678022[66] = 0;
   out_2110048490497678022[67] = 0;
   out_2110048490497678022[68] = 0;
   out_2110048490497678022[69] = 0;
   out_2110048490497678022[70] = 0;
   out_2110048490497678022[71] = 0;
   out_2110048490497678022[72] = 1;
   out_2110048490497678022[73] = dt;
   out_2110048490497678022[74] = 0;
   out_2110048490497678022[75] = 0;
   out_2110048490497678022[76] = 0;
   out_2110048490497678022[77] = 0;
   out_2110048490497678022[78] = 0;
   out_2110048490497678022[79] = 0;
   out_2110048490497678022[80] = 0;
   out_2110048490497678022[81] = 0;
   out_2110048490497678022[82] = 0;
   out_2110048490497678022[83] = 0;
   out_2110048490497678022[84] = 1;
   out_2110048490497678022[85] = dt;
   out_2110048490497678022[86] = 0;
   out_2110048490497678022[87] = 0;
   out_2110048490497678022[88] = 0;
   out_2110048490497678022[89] = 0;
   out_2110048490497678022[90] = 0;
   out_2110048490497678022[91] = 0;
   out_2110048490497678022[92] = 0;
   out_2110048490497678022[93] = 0;
   out_2110048490497678022[94] = 0;
   out_2110048490497678022[95] = 0;
   out_2110048490497678022[96] = 1;
   out_2110048490497678022[97] = 0;
   out_2110048490497678022[98] = 0;
   out_2110048490497678022[99] = 0;
   out_2110048490497678022[100] = 0;
   out_2110048490497678022[101] = 0;
   out_2110048490497678022[102] = 0;
   out_2110048490497678022[103] = 0;
   out_2110048490497678022[104] = 0;
   out_2110048490497678022[105] = 0;
   out_2110048490497678022[106] = 0;
   out_2110048490497678022[107] = 0;
   out_2110048490497678022[108] = 1;
   out_2110048490497678022[109] = 0;
   out_2110048490497678022[110] = 0;
   out_2110048490497678022[111] = 0;
   out_2110048490497678022[112] = 0;
   out_2110048490497678022[113] = 0;
   out_2110048490497678022[114] = 0;
   out_2110048490497678022[115] = 0;
   out_2110048490497678022[116] = 0;
   out_2110048490497678022[117] = 0;
   out_2110048490497678022[118] = 0;
   out_2110048490497678022[119] = 0;
   out_2110048490497678022[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_2787231368960978811) {
   out_2787231368960978811[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_1075816899521368544) {
   out_1075816899521368544[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1075816899521368544[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1075816899521368544[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1075816899521368544[3] = 0;
   out_1075816899521368544[4] = 0;
   out_1075816899521368544[5] = 0;
   out_1075816899521368544[6] = 1;
   out_1075816899521368544[7] = 0;
   out_1075816899521368544[8] = 0;
   out_1075816899521368544[9] = 0;
   out_1075816899521368544[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_8259322119346965405) {
   out_8259322119346965405[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_3930923713889965456) {
   out_3930923713889965456[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3930923713889965456[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3930923713889965456[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3930923713889965456[3] = 0;
   out_3930923713889965456[4] = 0;
   out_3930923713889965456[5] = 0;
   out_3930923713889965456[6] = 1;
   out_3930923713889965456[7] = 0;
   out_3930923713889965456[8] = 0;
   out_3930923713889965456[9] = 1;
   out_3930923713889965456[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_5032550934911842458) {
   out_5032550934911842458[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_1106145761210530165) {
   out_1106145761210530165[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1106145761210530165[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1106145761210530165[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1106145761210530165[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1106145761210530165[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1106145761210530165[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1106145761210530165[6] = 0;
   out_1106145761210530165[7] = 1;
   out_1106145761210530165[8] = 0;
   out_1106145761210530165[9] = 0;
   out_1106145761210530165[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_5032550934911842458) {
   out_5032550934911842458[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_1106145761210530165) {
   out_1106145761210530165[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1106145761210530165[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1106145761210530165[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1106145761210530165[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1106145761210530165[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1106145761210530165[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1106145761210530165[6] = 0;
   out_1106145761210530165[7] = 1;
   out_1106145761210530165[8] = 0;
   out_1106145761210530165[9] = 0;
   out_1106145761210530165[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9152697212499809699) {
  err_fun(nom_x, delta_x, out_9152697212499809699);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_6641208110102973198) {
  inv_err_fun(nom_x, true_x, out_6641208110102973198);
}
void gnss_H_mod_fun(double *state, double *out_4882180096896426461) {
  H_mod_fun(state, out_4882180096896426461);
}
void gnss_f_fun(double *state, double dt, double *out_8139212335340482105) {
  f_fun(state,  dt, out_8139212335340482105);
}
void gnss_F_fun(double *state, double dt, double *out_2110048490497678022) {
  F_fun(state,  dt, out_2110048490497678022);
}
void gnss_h_6(double *state, double *sat_pos, double *out_2787231368960978811) {
  h_6(state, sat_pos, out_2787231368960978811);
}
void gnss_H_6(double *state, double *sat_pos, double *out_1075816899521368544) {
  H_6(state, sat_pos, out_1075816899521368544);
}
void gnss_h_20(double *state, double *sat_pos, double *out_8259322119346965405) {
  h_20(state, sat_pos, out_8259322119346965405);
}
void gnss_H_20(double *state, double *sat_pos, double *out_3930923713889965456) {
  H_20(state, sat_pos, out_3930923713889965456);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_5032550934911842458) {
  h_7(state, sat_pos_vel, out_5032550934911842458);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_1106145761210530165) {
  H_7(state, sat_pos_vel, out_1106145761210530165);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_5032550934911842458) {
  h_21(state, sat_pos_vel, out_5032550934911842458);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_1106145761210530165) {
  H_21(state, sat_pos_vel, out_1106145761210530165);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);

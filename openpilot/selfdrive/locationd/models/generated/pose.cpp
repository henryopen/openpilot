#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_342973137176612241) {
   out_342973137176612241[0] = delta_x[0] + nom_x[0];
   out_342973137176612241[1] = delta_x[1] + nom_x[1];
   out_342973137176612241[2] = delta_x[2] + nom_x[2];
   out_342973137176612241[3] = delta_x[3] + nom_x[3];
   out_342973137176612241[4] = delta_x[4] + nom_x[4];
   out_342973137176612241[5] = delta_x[5] + nom_x[5];
   out_342973137176612241[6] = delta_x[6] + nom_x[6];
   out_342973137176612241[7] = delta_x[7] + nom_x[7];
   out_342973137176612241[8] = delta_x[8] + nom_x[8];
   out_342973137176612241[9] = delta_x[9] + nom_x[9];
   out_342973137176612241[10] = delta_x[10] + nom_x[10];
   out_342973137176612241[11] = delta_x[11] + nom_x[11];
   out_342973137176612241[12] = delta_x[12] + nom_x[12];
   out_342973137176612241[13] = delta_x[13] + nom_x[13];
   out_342973137176612241[14] = delta_x[14] + nom_x[14];
   out_342973137176612241[15] = delta_x[15] + nom_x[15];
   out_342973137176612241[16] = delta_x[16] + nom_x[16];
   out_342973137176612241[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4414887224430184744) {
   out_4414887224430184744[0] = -nom_x[0] + true_x[0];
   out_4414887224430184744[1] = -nom_x[1] + true_x[1];
   out_4414887224430184744[2] = -nom_x[2] + true_x[2];
   out_4414887224430184744[3] = -nom_x[3] + true_x[3];
   out_4414887224430184744[4] = -nom_x[4] + true_x[4];
   out_4414887224430184744[5] = -nom_x[5] + true_x[5];
   out_4414887224430184744[6] = -nom_x[6] + true_x[6];
   out_4414887224430184744[7] = -nom_x[7] + true_x[7];
   out_4414887224430184744[8] = -nom_x[8] + true_x[8];
   out_4414887224430184744[9] = -nom_x[9] + true_x[9];
   out_4414887224430184744[10] = -nom_x[10] + true_x[10];
   out_4414887224430184744[11] = -nom_x[11] + true_x[11];
   out_4414887224430184744[12] = -nom_x[12] + true_x[12];
   out_4414887224430184744[13] = -nom_x[13] + true_x[13];
   out_4414887224430184744[14] = -nom_x[14] + true_x[14];
   out_4414887224430184744[15] = -nom_x[15] + true_x[15];
   out_4414887224430184744[16] = -nom_x[16] + true_x[16];
   out_4414887224430184744[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_824354775938542426) {
   out_824354775938542426[0] = 1.0;
   out_824354775938542426[1] = 0.0;
   out_824354775938542426[2] = 0.0;
   out_824354775938542426[3] = 0.0;
   out_824354775938542426[4] = 0.0;
   out_824354775938542426[5] = 0.0;
   out_824354775938542426[6] = 0.0;
   out_824354775938542426[7] = 0.0;
   out_824354775938542426[8] = 0.0;
   out_824354775938542426[9] = 0.0;
   out_824354775938542426[10] = 0.0;
   out_824354775938542426[11] = 0.0;
   out_824354775938542426[12] = 0.0;
   out_824354775938542426[13] = 0.0;
   out_824354775938542426[14] = 0.0;
   out_824354775938542426[15] = 0.0;
   out_824354775938542426[16] = 0.0;
   out_824354775938542426[17] = 0.0;
   out_824354775938542426[18] = 0.0;
   out_824354775938542426[19] = 1.0;
   out_824354775938542426[20] = 0.0;
   out_824354775938542426[21] = 0.0;
   out_824354775938542426[22] = 0.0;
   out_824354775938542426[23] = 0.0;
   out_824354775938542426[24] = 0.0;
   out_824354775938542426[25] = 0.0;
   out_824354775938542426[26] = 0.0;
   out_824354775938542426[27] = 0.0;
   out_824354775938542426[28] = 0.0;
   out_824354775938542426[29] = 0.0;
   out_824354775938542426[30] = 0.0;
   out_824354775938542426[31] = 0.0;
   out_824354775938542426[32] = 0.0;
   out_824354775938542426[33] = 0.0;
   out_824354775938542426[34] = 0.0;
   out_824354775938542426[35] = 0.0;
   out_824354775938542426[36] = 0.0;
   out_824354775938542426[37] = 0.0;
   out_824354775938542426[38] = 1.0;
   out_824354775938542426[39] = 0.0;
   out_824354775938542426[40] = 0.0;
   out_824354775938542426[41] = 0.0;
   out_824354775938542426[42] = 0.0;
   out_824354775938542426[43] = 0.0;
   out_824354775938542426[44] = 0.0;
   out_824354775938542426[45] = 0.0;
   out_824354775938542426[46] = 0.0;
   out_824354775938542426[47] = 0.0;
   out_824354775938542426[48] = 0.0;
   out_824354775938542426[49] = 0.0;
   out_824354775938542426[50] = 0.0;
   out_824354775938542426[51] = 0.0;
   out_824354775938542426[52] = 0.0;
   out_824354775938542426[53] = 0.0;
   out_824354775938542426[54] = 0.0;
   out_824354775938542426[55] = 0.0;
   out_824354775938542426[56] = 0.0;
   out_824354775938542426[57] = 1.0;
   out_824354775938542426[58] = 0.0;
   out_824354775938542426[59] = 0.0;
   out_824354775938542426[60] = 0.0;
   out_824354775938542426[61] = 0.0;
   out_824354775938542426[62] = 0.0;
   out_824354775938542426[63] = 0.0;
   out_824354775938542426[64] = 0.0;
   out_824354775938542426[65] = 0.0;
   out_824354775938542426[66] = 0.0;
   out_824354775938542426[67] = 0.0;
   out_824354775938542426[68] = 0.0;
   out_824354775938542426[69] = 0.0;
   out_824354775938542426[70] = 0.0;
   out_824354775938542426[71] = 0.0;
   out_824354775938542426[72] = 0.0;
   out_824354775938542426[73] = 0.0;
   out_824354775938542426[74] = 0.0;
   out_824354775938542426[75] = 0.0;
   out_824354775938542426[76] = 1.0;
   out_824354775938542426[77] = 0.0;
   out_824354775938542426[78] = 0.0;
   out_824354775938542426[79] = 0.0;
   out_824354775938542426[80] = 0.0;
   out_824354775938542426[81] = 0.0;
   out_824354775938542426[82] = 0.0;
   out_824354775938542426[83] = 0.0;
   out_824354775938542426[84] = 0.0;
   out_824354775938542426[85] = 0.0;
   out_824354775938542426[86] = 0.0;
   out_824354775938542426[87] = 0.0;
   out_824354775938542426[88] = 0.0;
   out_824354775938542426[89] = 0.0;
   out_824354775938542426[90] = 0.0;
   out_824354775938542426[91] = 0.0;
   out_824354775938542426[92] = 0.0;
   out_824354775938542426[93] = 0.0;
   out_824354775938542426[94] = 0.0;
   out_824354775938542426[95] = 1.0;
   out_824354775938542426[96] = 0.0;
   out_824354775938542426[97] = 0.0;
   out_824354775938542426[98] = 0.0;
   out_824354775938542426[99] = 0.0;
   out_824354775938542426[100] = 0.0;
   out_824354775938542426[101] = 0.0;
   out_824354775938542426[102] = 0.0;
   out_824354775938542426[103] = 0.0;
   out_824354775938542426[104] = 0.0;
   out_824354775938542426[105] = 0.0;
   out_824354775938542426[106] = 0.0;
   out_824354775938542426[107] = 0.0;
   out_824354775938542426[108] = 0.0;
   out_824354775938542426[109] = 0.0;
   out_824354775938542426[110] = 0.0;
   out_824354775938542426[111] = 0.0;
   out_824354775938542426[112] = 0.0;
   out_824354775938542426[113] = 0.0;
   out_824354775938542426[114] = 1.0;
   out_824354775938542426[115] = 0.0;
   out_824354775938542426[116] = 0.0;
   out_824354775938542426[117] = 0.0;
   out_824354775938542426[118] = 0.0;
   out_824354775938542426[119] = 0.0;
   out_824354775938542426[120] = 0.0;
   out_824354775938542426[121] = 0.0;
   out_824354775938542426[122] = 0.0;
   out_824354775938542426[123] = 0.0;
   out_824354775938542426[124] = 0.0;
   out_824354775938542426[125] = 0.0;
   out_824354775938542426[126] = 0.0;
   out_824354775938542426[127] = 0.0;
   out_824354775938542426[128] = 0.0;
   out_824354775938542426[129] = 0.0;
   out_824354775938542426[130] = 0.0;
   out_824354775938542426[131] = 0.0;
   out_824354775938542426[132] = 0.0;
   out_824354775938542426[133] = 1.0;
   out_824354775938542426[134] = 0.0;
   out_824354775938542426[135] = 0.0;
   out_824354775938542426[136] = 0.0;
   out_824354775938542426[137] = 0.0;
   out_824354775938542426[138] = 0.0;
   out_824354775938542426[139] = 0.0;
   out_824354775938542426[140] = 0.0;
   out_824354775938542426[141] = 0.0;
   out_824354775938542426[142] = 0.0;
   out_824354775938542426[143] = 0.0;
   out_824354775938542426[144] = 0.0;
   out_824354775938542426[145] = 0.0;
   out_824354775938542426[146] = 0.0;
   out_824354775938542426[147] = 0.0;
   out_824354775938542426[148] = 0.0;
   out_824354775938542426[149] = 0.0;
   out_824354775938542426[150] = 0.0;
   out_824354775938542426[151] = 0.0;
   out_824354775938542426[152] = 1.0;
   out_824354775938542426[153] = 0.0;
   out_824354775938542426[154] = 0.0;
   out_824354775938542426[155] = 0.0;
   out_824354775938542426[156] = 0.0;
   out_824354775938542426[157] = 0.0;
   out_824354775938542426[158] = 0.0;
   out_824354775938542426[159] = 0.0;
   out_824354775938542426[160] = 0.0;
   out_824354775938542426[161] = 0.0;
   out_824354775938542426[162] = 0.0;
   out_824354775938542426[163] = 0.0;
   out_824354775938542426[164] = 0.0;
   out_824354775938542426[165] = 0.0;
   out_824354775938542426[166] = 0.0;
   out_824354775938542426[167] = 0.0;
   out_824354775938542426[168] = 0.0;
   out_824354775938542426[169] = 0.0;
   out_824354775938542426[170] = 0.0;
   out_824354775938542426[171] = 1.0;
   out_824354775938542426[172] = 0.0;
   out_824354775938542426[173] = 0.0;
   out_824354775938542426[174] = 0.0;
   out_824354775938542426[175] = 0.0;
   out_824354775938542426[176] = 0.0;
   out_824354775938542426[177] = 0.0;
   out_824354775938542426[178] = 0.0;
   out_824354775938542426[179] = 0.0;
   out_824354775938542426[180] = 0.0;
   out_824354775938542426[181] = 0.0;
   out_824354775938542426[182] = 0.0;
   out_824354775938542426[183] = 0.0;
   out_824354775938542426[184] = 0.0;
   out_824354775938542426[185] = 0.0;
   out_824354775938542426[186] = 0.0;
   out_824354775938542426[187] = 0.0;
   out_824354775938542426[188] = 0.0;
   out_824354775938542426[189] = 0.0;
   out_824354775938542426[190] = 1.0;
   out_824354775938542426[191] = 0.0;
   out_824354775938542426[192] = 0.0;
   out_824354775938542426[193] = 0.0;
   out_824354775938542426[194] = 0.0;
   out_824354775938542426[195] = 0.0;
   out_824354775938542426[196] = 0.0;
   out_824354775938542426[197] = 0.0;
   out_824354775938542426[198] = 0.0;
   out_824354775938542426[199] = 0.0;
   out_824354775938542426[200] = 0.0;
   out_824354775938542426[201] = 0.0;
   out_824354775938542426[202] = 0.0;
   out_824354775938542426[203] = 0.0;
   out_824354775938542426[204] = 0.0;
   out_824354775938542426[205] = 0.0;
   out_824354775938542426[206] = 0.0;
   out_824354775938542426[207] = 0.0;
   out_824354775938542426[208] = 0.0;
   out_824354775938542426[209] = 1.0;
   out_824354775938542426[210] = 0.0;
   out_824354775938542426[211] = 0.0;
   out_824354775938542426[212] = 0.0;
   out_824354775938542426[213] = 0.0;
   out_824354775938542426[214] = 0.0;
   out_824354775938542426[215] = 0.0;
   out_824354775938542426[216] = 0.0;
   out_824354775938542426[217] = 0.0;
   out_824354775938542426[218] = 0.0;
   out_824354775938542426[219] = 0.0;
   out_824354775938542426[220] = 0.0;
   out_824354775938542426[221] = 0.0;
   out_824354775938542426[222] = 0.0;
   out_824354775938542426[223] = 0.0;
   out_824354775938542426[224] = 0.0;
   out_824354775938542426[225] = 0.0;
   out_824354775938542426[226] = 0.0;
   out_824354775938542426[227] = 0.0;
   out_824354775938542426[228] = 1.0;
   out_824354775938542426[229] = 0.0;
   out_824354775938542426[230] = 0.0;
   out_824354775938542426[231] = 0.0;
   out_824354775938542426[232] = 0.0;
   out_824354775938542426[233] = 0.0;
   out_824354775938542426[234] = 0.0;
   out_824354775938542426[235] = 0.0;
   out_824354775938542426[236] = 0.0;
   out_824354775938542426[237] = 0.0;
   out_824354775938542426[238] = 0.0;
   out_824354775938542426[239] = 0.0;
   out_824354775938542426[240] = 0.0;
   out_824354775938542426[241] = 0.0;
   out_824354775938542426[242] = 0.0;
   out_824354775938542426[243] = 0.0;
   out_824354775938542426[244] = 0.0;
   out_824354775938542426[245] = 0.0;
   out_824354775938542426[246] = 0.0;
   out_824354775938542426[247] = 1.0;
   out_824354775938542426[248] = 0.0;
   out_824354775938542426[249] = 0.0;
   out_824354775938542426[250] = 0.0;
   out_824354775938542426[251] = 0.0;
   out_824354775938542426[252] = 0.0;
   out_824354775938542426[253] = 0.0;
   out_824354775938542426[254] = 0.0;
   out_824354775938542426[255] = 0.0;
   out_824354775938542426[256] = 0.0;
   out_824354775938542426[257] = 0.0;
   out_824354775938542426[258] = 0.0;
   out_824354775938542426[259] = 0.0;
   out_824354775938542426[260] = 0.0;
   out_824354775938542426[261] = 0.0;
   out_824354775938542426[262] = 0.0;
   out_824354775938542426[263] = 0.0;
   out_824354775938542426[264] = 0.0;
   out_824354775938542426[265] = 0.0;
   out_824354775938542426[266] = 1.0;
   out_824354775938542426[267] = 0.0;
   out_824354775938542426[268] = 0.0;
   out_824354775938542426[269] = 0.0;
   out_824354775938542426[270] = 0.0;
   out_824354775938542426[271] = 0.0;
   out_824354775938542426[272] = 0.0;
   out_824354775938542426[273] = 0.0;
   out_824354775938542426[274] = 0.0;
   out_824354775938542426[275] = 0.0;
   out_824354775938542426[276] = 0.0;
   out_824354775938542426[277] = 0.0;
   out_824354775938542426[278] = 0.0;
   out_824354775938542426[279] = 0.0;
   out_824354775938542426[280] = 0.0;
   out_824354775938542426[281] = 0.0;
   out_824354775938542426[282] = 0.0;
   out_824354775938542426[283] = 0.0;
   out_824354775938542426[284] = 0.0;
   out_824354775938542426[285] = 1.0;
   out_824354775938542426[286] = 0.0;
   out_824354775938542426[287] = 0.0;
   out_824354775938542426[288] = 0.0;
   out_824354775938542426[289] = 0.0;
   out_824354775938542426[290] = 0.0;
   out_824354775938542426[291] = 0.0;
   out_824354775938542426[292] = 0.0;
   out_824354775938542426[293] = 0.0;
   out_824354775938542426[294] = 0.0;
   out_824354775938542426[295] = 0.0;
   out_824354775938542426[296] = 0.0;
   out_824354775938542426[297] = 0.0;
   out_824354775938542426[298] = 0.0;
   out_824354775938542426[299] = 0.0;
   out_824354775938542426[300] = 0.0;
   out_824354775938542426[301] = 0.0;
   out_824354775938542426[302] = 0.0;
   out_824354775938542426[303] = 0.0;
   out_824354775938542426[304] = 1.0;
   out_824354775938542426[305] = 0.0;
   out_824354775938542426[306] = 0.0;
   out_824354775938542426[307] = 0.0;
   out_824354775938542426[308] = 0.0;
   out_824354775938542426[309] = 0.0;
   out_824354775938542426[310] = 0.0;
   out_824354775938542426[311] = 0.0;
   out_824354775938542426[312] = 0.0;
   out_824354775938542426[313] = 0.0;
   out_824354775938542426[314] = 0.0;
   out_824354775938542426[315] = 0.0;
   out_824354775938542426[316] = 0.0;
   out_824354775938542426[317] = 0.0;
   out_824354775938542426[318] = 0.0;
   out_824354775938542426[319] = 0.0;
   out_824354775938542426[320] = 0.0;
   out_824354775938542426[321] = 0.0;
   out_824354775938542426[322] = 0.0;
   out_824354775938542426[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_432270031952541382) {
   out_432270031952541382[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_432270031952541382[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_432270031952541382[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_432270031952541382[3] = dt*state[12] + state[3];
   out_432270031952541382[4] = dt*state[13] + state[4];
   out_432270031952541382[5] = dt*state[14] + state[5];
   out_432270031952541382[6] = state[6];
   out_432270031952541382[7] = state[7];
   out_432270031952541382[8] = state[8];
   out_432270031952541382[9] = state[9];
   out_432270031952541382[10] = state[10];
   out_432270031952541382[11] = state[11];
   out_432270031952541382[12] = state[12];
   out_432270031952541382[13] = state[13];
   out_432270031952541382[14] = state[14];
   out_432270031952541382[15] = state[15];
   out_432270031952541382[16] = state[16];
   out_432270031952541382[17] = state[17];
}
void F_fun(double *state, double dt, double *out_6844361959189325516) {
   out_6844361959189325516[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6844361959189325516[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6844361959189325516[2] = 0;
   out_6844361959189325516[3] = 0;
   out_6844361959189325516[4] = 0;
   out_6844361959189325516[5] = 0;
   out_6844361959189325516[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6844361959189325516[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6844361959189325516[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6844361959189325516[9] = 0;
   out_6844361959189325516[10] = 0;
   out_6844361959189325516[11] = 0;
   out_6844361959189325516[12] = 0;
   out_6844361959189325516[13] = 0;
   out_6844361959189325516[14] = 0;
   out_6844361959189325516[15] = 0;
   out_6844361959189325516[16] = 0;
   out_6844361959189325516[17] = 0;
   out_6844361959189325516[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6844361959189325516[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6844361959189325516[20] = 0;
   out_6844361959189325516[21] = 0;
   out_6844361959189325516[22] = 0;
   out_6844361959189325516[23] = 0;
   out_6844361959189325516[24] = 0;
   out_6844361959189325516[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6844361959189325516[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6844361959189325516[27] = 0;
   out_6844361959189325516[28] = 0;
   out_6844361959189325516[29] = 0;
   out_6844361959189325516[30] = 0;
   out_6844361959189325516[31] = 0;
   out_6844361959189325516[32] = 0;
   out_6844361959189325516[33] = 0;
   out_6844361959189325516[34] = 0;
   out_6844361959189325516[35] = 0;
   out_6844361959189325516[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6844361959189325516[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6844361959189325516[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6844361959189325516[39] = 0;
   out_6844361959189325516[40] = 0;
   out_6844361959189325516[41] = 0;
   out_6844361959189325516[42] = 0;
   out_6844361959189325516[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6844361959189325516[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6844361959189325516[45] = 0;
   out_6844361959189325516[46] = 0;
   out_6844361959189325516[47] = 0;
   out_6844361959189325516[48] = 0;
   out_6844361959189325516[49] = 0;
   out_6844361959189325516[50] = 0;
   out_6844361959189325516[51] = 0;
   out_6844361959189325516[52] = 0;
   out_6844361959189325516[53] = 0;
   out_6844361959189325516[54] = 0;
   out_6844361959189325516[55] = 0;
   out_6844361959189325516[56] = 0;
   out_6844361959189325516[57] = 1;
   out_6844361959189325516[58] = 0;
   out_6844361959189325516[59] = 0;
   out_6844361959189325516[60] = 0;
   out_6844361959189325516[61] = 0;
   out_6844361959189325516[62] = 0;
   out_6844361959189325516[63] = 0;
   out_6844361959189325516[64] = 0;
   out_6844361959189325516[65] = 0;
   out_6844361959189325516[66] = dt;
   out_6844361959189325516[67] = 0;
   out_6844361959189325516[68] = 0;
   out_6844361959189325516[69] = 0;
   out_6844361959189325516[70] = 0;
   out_6844361959189325516[71] = 0;
   out_6844361959189325516[72] = 0;
   out_6844361959189325516[73] = 0;
   out_6844361959189325516[74] = 0;
   out_6844361959189325516[75] = 0;
   out_6844361959189325516[76] = 1;
   out_6844361959189325516[77] = 0;
   out_6844361959189325516[78] = 0;
   out_6844361959189325516[79] = 0;
   out_6844361959189325516[80] = 0;
   out_6844361959189325516[81] = 0;
   out_6844361959189325516[82] = 0;
   out_6844361959189325516[83] = 0;
   out_6844361959189325516[84] = 0;
   out_6844361959189325516[85] = dt;
   out_6844361959189325516[86] = 0;
   out_6844361959189325516[87] = 0;
   out_6844361959189325516[88] = 0;
   out_6844361959189325516[89] = 0;
   out_6844361959189325516[90] = 0;
   out_6844361959189325516[91] = 0;
   out_6844361959189325516[92] = 0;
   out_6844361959189325516[93] = 0;
   out_6844361959189325516[94] = 0;
   out_6844361959189325516[95] = 1;
   out_6844361959189325516[96] = 0;
   out_6844361959189325516[97] = 0;
   out_6844361959189325516[98] = 0;
   out_6844361959189325516[99] = 0;
   out_6844361959189325516[100] = 0;
   out_6844361959189325516[101] = 0;
   out_6844361959189325516[102] = 0;
   out_6844361959189325516[103] = 0;
   out_6844361959189325516[104] = dt;
   out_6844361959189325516[105] = 0;
   out_6844361959189325516[106] = 0;
   out_6844361959189325516[107] = 0;
   out_6844361959189325516[108] = 0;
   out_6844361959189325516[109] = 0;
   out_6844361959189325516[110] = 0;
   out_6844361959189325516[111] = 0;
   out_6844361959189325516[112] = 0;
   out_6844361959189325516[113] = 0;
   out_6844361959189325516[114] = 1;
   out_6844361959189325516[115] = 0;
   out_6844361959189325516[116] = 0;
   out_6844361959189325516[117] = 0;
   out_6844361959189325516[118] = 0;
   out_6844361959189325516[119] = 0;
   out_6844361959189325516[120] = 0;
   out_6844361959189325516[121] = 0;
   out_6844361959189325516[122] = 0;
   out_6844361959189325516[123] = 0;
   out_6844361959189325516[124] = 0;
   out_6844361959189325516[125] = 0;
   out_6844361959189325516[126] = 0;
   out_6844361959189325516[127] = 0;
   out_6844361959189325516[128] = 0;
   out_6844361959189325516[129] = 0;
   out_6844361959189325516[130] = 0;
   out_6844361959189325516[131] = 0;
   out_6844361959189325516[132] = 0;
   out_6844361959189325516[133] = 1;
   out_6844361959189325516[134] = 0;
   out_6844361959189325516[135] = 0;
   out_6844361959189325516[136] = 0;
   out_6844361959189325516[137] = 0;
   out_6844361959189325516[138] = 0;
   out_6844361959189325516[139] = 0;
   out_6844361959189325516[140] = 0;
   out_6844361959189325516[141] = 0;
   out_6844361959189325516[142] = 0;
   out_6844361959189325516[143] = 0;
   out_6844361959189325516[144] = 0;
   out_6844361959189325516[145] = 0;
   out_6844361959189325516[146] = 0;
   out_6844361959189325516[147] = 0;
   out_6844361959189325516[148] = 0;
   out_6844361959189325516[149] = 0;
   out_6844361959189325516[150] = 0;
   out_6844361959189325516[151] = 0;
   out_6844361959189325516[152] = 1;
   out_6844361959189325516[153] = 0;
   out_6844361959189325516[154] = 0;
   out_6844361959189325516[155] = 0;
   out_6844361959189325516[156] = 0;
   out_6844361959189325516[157] = 0;
   out_6844361959189325516[158] = 0;
   out_6844361959189325516[159] = 0;
   out_6844361959189325516[160] = 0;
   out_6844361959189325516[161] = 0;
   out_6844361959189325516[162] = 0;
   out_6844361959189325516[163] = 0;
   out_6844361959189325516[164] = 0;
   out_6844361959189325516[165] = 0;
   out_6844361959189325516[166] = 0;
   out_6844361959189325516[167] = 0;
   out_6844361959189325516[168] = 0;
   out_6844361959189325516[169] = 0;
   out_6844361959189325516[170] = 0;
   out_6844361959189325516[171] = 1;
   out_6844361959189325516[172] = 0;
   out_6844361959189325516[173] = 0;
   out_6844361959189325516[174] = 0;
   out_6844361959189325516[175] = 0;
   out_6844361959189325516[176] = 0;
   out_6844361959189325516[177] = 0;
   out_6844361959189325516[178] = 0;
   out_6844361959189325516[179] = 0;
   out_6844361959189325516[180] = 0;
   out_6844361959189325516[181] = 0;
   out_6844361959189325516[182] = 0;
   out_6844361959189325516[183] = 0;
   out_6844361959189325516[184] = 0;
   out_6844361959189325516[185] = 0;
   out_6844361959189325516[186] = 0;
   out_6844361959189325516[187] = 0;
   out_6844361959189325516[188] = 0;
   out_6844361959189325516[189] = 0;
   out_6844361959189325516[190] = 1;
   out_6844361959189325516[191] = 0;
   out_6844361959189325516[192] = 0;
   out_6844361959189325516[193] = 0;
   out_6844361959189325516[194] = 0;
   out_6844361959189325516[195] = 0;
   out_6844361959189325516[196] = 0;
   out_6844361959189325516[197] = 0;
   out_6844361959189325516[198] = 0;
   out_6844361959189325516[199] = 0;
   out_6844361959189325516[200] = 0;
   out_6844361959189325516[201] = 0;
   out_6844361959189325516[202] = 0;
   out_6844361959189325516[203] = 0;
   out_6844361959189325516[204] = 0;
   out_6844361959189325516[205] = 0;
   out_6844361959189325516[206] = 0;
   out_6844361959189325516[207] = 0;
   out_6844361959189325516[208] = 0;
   out_6844361959189325516[209] = 1;
   out_6844361959189325516[210] = 0;
   out_6844361959189325516[211] = 0;
   out_6844361959189325516[212] = 0;
   out_6844361959189325516[213] = 0;
   out_6844361959189325516[214] = 0;
   out_6844361959189325516[215] = 0;
   out_6844361959189325516[216] = 0;
   out_6844361959189325516[217] = 0;
   out_6844361959189325516[218] = 0;
   out_6844361959189325516[219] = 0;
   out_6844361959189325516[220] = 0;
   out_6844361959189325516[221] = 0;
   out_6844361959189325516[222] = 0;
   out_6844361959189325516[223] = 0;
   out_6844361959189325516[224] = 0;
   out_6844361959189325516[225] = 0;
   out_6844361959189325516[226] = 0;
   out_6844361959189325516[227] = 0;
   out_6844361959189325516[228] = 1;
   out_6844361959189325516[229] = 0;
   out_6844361959189325516[230] = 0;
   out_6844361959189325516[231] = 0;
   out_6844361959189325516[232] = 0;
   out_6844361959189325516[233] = 0;
   out_6844361959189325516[234] = 0;
   out_6844361959189325516[235] = 0;
   out_6844361959189325516[236] = 0;
   out_6844361959189325516[237] = 0;
   out_6844361959189325516[238] = 0;
   out_6844361959189325516[239] = 0;
   out_6844361959189325516[240] = 0;
   out_6844361959189325516[241] = 0;
   out_6844361959189325516[242] = 0;
   out_6844361959189325516[243] = 0;
   out_6844361959189325516[244] = 0;
   out_6844361959189325516[245] = 0;
   out_6844361959189325516[246] = 0;
   out_6844361959189325516[247] = 1;
   out_6844361959189325516[248] = 0;
   out_6844361959189325516[249] = 0;
   out_6844361959189325516[250] = 0;
   out_6844361959189325516[251] = 0;
   out_6844361959189325516[252] = 0;
   out_6844361959189325516[253] = 0;
   out_6844361959189325516[254] = 0;
   out_6844361959189325516[255] = 0;
   out_6844361959189325516[256] = 0;
   out_6844361959189325516[257] = 0;
   out_6844361959189325516[258] = 0;
   out_6844361959189325516[259] = 0;
   out_6844361959189325516[260] = 0;
   out_6844361959189325516[261] = 0;
   out_6844361959189325516[262] = 0;
   out_6844361959189325516[263] = 0;
   out_6844361959189325516[264] = 0;
   out_6844361959189325516[265] = 0;
   out_6844361959189325516[266] = 1;
   out_6844361959189325516[267] = 0;
   out_6844361959189325516[268] = 0;
   out_6844361959189325516[269] = 0;
   out_6844361959189325516[270] = 0;
   out_6844361959189325516[271] = 0;
   out_6844361959189325516[272] = 0;
   out_6844361959189325516[273] = 0;
   out_6844361959189325516[274] = 0;
   out_6844361959189325516[275] = 0;
   out_6844361959189325516[276] = 0;
   out_6844361959189325516[277] = 0;
   out_6844361959189325516[278] = 0;
   out_6844361959189325516[279] = 0;
   out_6844361959189325516[280] = 0;
   out_6844361959189325516[281] = 0;
   out_6844361959189325516[282] = 0;
   out_6844361959189325516[283] = 0;
   out_6844361959189325516[284] = 0;
   out_6844361959189325516[285] = 1;
   out_6844361959189325516[286] = 0;
   out_6844361959189325516[287] = 0;
   out_6844361959189325516[288] = 0;
   out_6844361959189325516[289] = 0;
   out_6844361959189325516[290] = 0;
   out_6844361959189325516[291] = 0;
   out_6844361959189325516[292] = 0;
   out_6844361959189325516[293] = 0;
   out_6844361959189325516[294] = 0;
   out_6844361959189325516[295] = 0;
   out_6844361959189325516[296] = 0;
   out_6844361959189325516[297] = 0;
   out_6844361959189325516[298] = 0;
   out_6844361959189325516[299] = 0;
   out_6844361959189325516[300] = 0;
   out_6844361959189325516[301] = 0;
   out_6844361959189325516[302] = 0;
   out_6844361959189325516[303] = 0;
   out_6844361959189325516[304] = 1;
   out_6844361959189325516[305] = 0;
   out_6844361959189325516[306] = 0;
   out_6844361959189325516[307] = 0;
   out_6844361959189325516[308] = 0;
   out_6844361959189325516[309] = 0;
   out_6844361959189325516[310] = 0;
   out_6844361959189325516[311] = 0;
   out_6844361959189325516[312] = 0;
   out_6844361959189325516[313] = 0;
   out_6844361959189325516[314] = 0;
   out_6844361959189325516[315] = 0;
   out_6844361959189325516[316] = 0;
   out_6844361959189325516[317] = 0;
   out_6844361959189325516[318] = 0;
   out_6844361959189325516[319] = 0;
   out_6844361959189325516[320] = 0;
   out_6844361959189325516[321] = 0;
   out_6844361959189325516[322] = 0;
   out_6844361959189325516[323] = 1;
}
void h_4(double *state, double *unused, double *out_948512472116913193) {
   out_948512472116913193[0] = state[6] + state[9];
   out_948512472116913193[1] = state[7] + state[10];
   out_948512472116913193[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_4098576854729491679) {
   out_4098576854729491679[0] = 0;
   out_4098576854729491679[1] = 0;
   out_4098576854729491679[2] = 0;
   out_4098576854729491679[3] = 0;
   out_4098576854729491679[4] = 0;
   out_4098576854729491679[5] = 0;
   out_4098576854729491679[6] = 1;
   out_4098576854729491679[7] = 0;
   out_4098576854729491679[8] = 0;
   out_4098576854729491679[9] = 1;
   out_4098576854729491679[10] = 0;
   out_4098576854729491679[11] = 0;
   out_4098576854729491679[12] = 0;
   out_4098576854729491679[13] = 0;
   out_4098576854729491679[14] = 0;
   out_4098576854729491679[15] = 0;
   out_4098576854729491679[16] = 0;
   out_4098576854729491679[17] = 0;
   out_4098576854729491679[18] = 0;
   out_4098576854729491679[19] = 0;
   out_4098576854729491679[20] = 0;
   out_4098576854729491679[21] = 0;
   out_4098576854729491679[22] = 0;
   out_4098576854729491679[23] = 0;
   out_4098576854729491679[24] = 0;
   out_4098576854729491679[25] = 1;
   out_4098576854729491679[26] = 0;
   out_4098576854729491679[27] = 0;
   out_4098576854729491679[28] = 1;
   out_4098576854729491679[29] = 0;
   out_4098576854729491679[30] = 0;
   out_4098576854729491679[31] = 0;
   out_4098576854729491679[32] = 0;
   out_4098576854729491679[33] = 0;
   out_4098576854729491679[34] = 0;
   out_4098576854729491679[35] = 0;
   out_4098576854729491679[36] = 0;
   out_4098576854729491679[37] = 0;
   out_4098576854729491679[38] = 0;
   out_4098576854729491679[39] = 0;
   out_4098576854729491679[40] = 0;
   out_4098576854729491679[41] = 0;
   out_4098576854729491679[42] = 0;
   out_4098576854729491679[43] = 0;
   out_4098576854729491679[44] = 1;
   out_4098576854729491679[45] = 0;
   out_4098576854729491679[46] = 0;
   out_4098576854729491679[47] = 1;
   out_4098576854729491679[48] = 0;
   out_4098576854729491679[49] = 0;
   out_4098576854729491679[50] = 0;
   out_4098576854729491679[51] = 0;
   out_4098576854729491679[52] = 0;
   out_4098576854729491679[53] = 0;
}
void h_10(double *state, double *unused, double *out_8780443367168833320) {
   out_8780443367168833320[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_8780443367168833320[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_8780443367168833320[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_7022073385523406729) {
   out_7022073385523406729[0] = 0;
   out_7022073385523406729[1] = 9.8100000000000005*cos(state[1]);
   out_7022073385523406729[2] = 0;
   out_7022073385523406729[3] = 0;
   out_7022073385523406729[4] = -state[8];
   out_7022073385523406729[5] = state[7];
   out_7022073385523406729[6] = 0;
   out_7022073385523406729[7] = state[5];
   out_7022073385523406729[8] = -state[4];
   out_7022073385523406729[9] = 0;
   out_7022073385523406729[10] = 0;
   out_7022073385523406729[11] = 0;
   out_7022073385523406729[12] = 1;
   out_7022073385523406729[13] = 0;
   out_7022073385523406729[14] = 0;
   out_7022073385523406729[15] = 1;
   out_7022073385523406729[16] = 0;
   out_7022073385523406729[17] = 0;
   out_7022073385523406729[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_7022073385523406729[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_7022073385523406729[20] = 0;
   out_7022073385523406729[21] = state[8];
   out_7022073385523406729[22] = 0;
   out_7022073385523406729[23] = -state[6];
   out_7022073385523406729[24] = -state[5];
   out_7022073385523406729[25] = 0;
   out_7022073385523406729[26] = state[3];
   out_7022073385523406729[27] = 0;
   out_7022073385523406729[28] = 0;
   out_7022073385523406729[29] = 0;
   out_7022073385523406729[30] = 0;
   out_7022073385523406729[31] = 1;
   out_7022073385523406729[32] = 0;
   out_7022073385523406729[33] = 0;
   out_7022073385523406729[34] = 1;
   out_7022073385523406729[35] = 0;
   out_7022073385523406729[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_7022073385523406729[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_7022073385523406729[38] = 0;
   out_7022073385523406729[39] = -state[7];
   out_7022073385523406729[40] = state[6];
   out_7022073385523406729[41] = 0;
   out_7022073385523406729[42] = state[4];
   out_7022073385523406729[43] = -state[3];
   out_7022073385523406729[44] = 0;
   out_7022073385523406729[45] = 0;
   out_7022073385523406729[46] = 0;
   out_7022073385523406729[47] = 0;
   out_7022073385523406729[48] = 0;
   out_7022073385523406729[49] = 0;
   out_7022073385523406729[50] = 1;
   out_7022073385523406729[51] = 0;
   out_7022073385523406729[52] = 0;
   out_7022073385523406729[53] = 1;
}
void h_13(double *state, double *unused, double *out_3854432803415109726) {
   out_3854432803415109726[0] = state[3];
   out_3854432803415109726[1] = state[4];
   out_3854432803415109726[2] = state[5];
}
void H_13(double *state, double *unused, double *out_3512054353587209250) {
   out_3512054353587209250[0] = 0;
   out_3512054353587209250[1] = 0;
   out_3512054353587209250[2] = 0;
   out_3512054353587209250[3] = 1;
   out_3512054353587209250[4] = 0;
   out_3512054353587209250[5] = 0;
   out_3512054353587209250[6] = 0;
   out_3512054353587209250[7] = 0;
   out_3512054353587209250[8] = 0;
   out_3512054353587209250[9] = 0;
   out_3512054353587209250[10] = 0;
   out_3512054353587209250[11] = 0;
   out_3512054353587209250[12] = 0;
   out_3512054353587209250[13] = 0;
   out_3512054353587209250[14] = 0;
   out_3512054353587209250[15] = 0;
   out_3512054353587209250[16] = 0;
   out_3512054353587209250[17] = 0;
   out_3512054353587209250[18] = 0;
   out_3512054353587209250[19] = 0;
   out_3512054353587209250[20] = 0;
   out_3512054353587209250[21] = 0;
   out_3512054353587209250[22] = 1;
   out_3512054353587209250[23] = 0;
   out_3512054353587209250[24] = 0;
   out_3512054353587209250[25] = 0;
   out_3512054353587209250[26] = 0;
   out_3512054353587209250[27] = 0;
   out_3512054353587209250[28] = 0;
   out_3512054353587209250[29] = 0;
   out_3512054353587209250[30] = 0;
   out_3512054353587209250[31] = 0;
   out_3512054353587209250[32] = 0;
   out_3512054353587209250[33] = 0;
   out_3512054353587209250[34] = 0;
   out_3512054353587209250[35] = 0;
   out_3512054353587209250[36] = 0;
   out_3512054353587209250[37] = 0;
   out_3512054353587209250[38] = 0;
   out_3512054353587209250[39] = 0;
   out_3512054353587209250[40] = 0;
   out_3512054353587209250[41] = 1;
   out_3512054353587209250[42] = 0;
   out_3512054353587209250[43] = 0;
   out_3512054353587209250[44] = 0;
   out_3512054353587209250[45] = 0;
   out_3512054353587209250[46] = 0;
   out_3512054353587209250[47] = 0;
   out_3512054353587209250[48] = 0;
   out_3512054353587209250[49] = 0;
   out_3512054353587209250[50] = 0;
   out_3512054353587209250[51] = 0;
   out_3512054353587209250[52] = 0;
   out_3512054353587209250[53] = 0;
}
void h_14(double *state, double *unused, double *out_3383842427869284419) {
   out_3383842427869284419[0] = state[6];
   out_3383842427869284419[1] = state[7];
   out_3383842427869284419[2] = state[8];
}
void H_14(double *state, double *unused, double *out_135335998390007150) {
   out_135335998390007150[0] = 0;
   out_135335998390007150[1] = 0;
   out_135335998390007150[2] = 0;
   out_135335998390007150[3] = 0;
   out_135335998390007150[4] = 0;
   out_135335998390007150[5] = 0;
   out_135335998390007150[6] = 1;
   out_135335998390007150[7] = 0;
   out_135335998390007150[8] = 0;
   out_135335998390007150[9] = 0;
   out_135335998390007150[10] = 0;
   out_135335998390007150[11] = 0;
   out_135335998390007150[12] = 0;
   out_135335998390007150[13] = 0;
   out_135335998390007150[14] = 0;
   out_135335998390007150[15] = 0;
   out_135335998390007150[16] = 0;
   out_135335998390007150[17] = 0;
   out_135335998390007150[18] = 0;
   out_135335998390007150[19] = 0;
   out_135335998390007150[20] = 0;
   out_135335998390007150[21] = 0;
   out_135335998390007150[22] = 0;
   out_135335998390007150[23] = 0;
   out_135335998390007150[24] = 0;
   out_135335998390007150[25] = 1;
   out_135335998390007150[26] = 0;
   out_135335998390007150[27] = 0;
   out_135335998390007150[28] = 0;
   out_135335998390007150[29] = 0;
   out_135335998390007150[30] = 0;
   out_135335998390007150[31] = 0;
   out_135335998390007150[32] = 0;
   out_135335998390007150[33] = 0;
   out_135335998390007150[34] = 0;
   out_135335998390007150[35] = 0;
   out_135335998390007150[36] = 0;
   out_135335998390007150[37] = 0;
   out_135335998390007150[38] = 0;
   out_135335998390007150[39] = 0;
   out_135335998390007150[40] = 0;
   out_135335998390007150[41] = 0;
   out_135335998390007150[42] = 0;
   out_135335998390007150[43] = 0;
   out_135335998390007150[44] = 1;
   out_135335998390007150[45] = 0;
   out_135335998390007150[46] = 0;
   out_135335998390007150[47] = 0;
   out_135335998390007150[48] = 0;
   out_135335998390007150[49] = 0;
   out_135335998390007150[50] = 0;
   out_135335998390007150[51] = 0;
   out_135335998390007150[52] = 0;
   out_135335998390007150[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_342973137176612241) {
  err_fun(nom_x, delta_x, out_342973137176612241);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4414887224430184744) {
  inv_err_fun(nom_x, true_x, out_4414887224430184744);
}
void pose_H_mod_fun(double *state, double *out_824354775938542426) {
  H_mod_fun(state, out_824354775938542426);
}
void pose_f_fun(double *state, double dt, double *out_432270031952541382) {
  f_fun(state,  dt, out_432270031952541382);
}
void pose_F_fun(double *state, double dt, double *out_6844361959189325516) {
  F_fun(state,  dt, out_6844361959189325516);
}
void pose_h_4(double *state, double *unused, double *out_948512472116913193) {
  h_4(state, unused, out_948512472116913193);
}
void pose_H_4(double *state, double *unused, double *out_4098576854729491679) {
  H_4(state, unused, out_4098576854729491679);
}
void pose_h_10(double *state, double *unused, double *out_8780443367168833320) {
  h_10(state, unused, out_8780443367168833320);
}
void pose_H_10(double *state, double *unused, double *out_7022073385523406729) {
  H_10(state, unused, out_7022073385523406729);
}
void pose_h_13(double *state, double *unused, double *out_3854432803415109726) {
  h_13(state, unused, out_3854432803415109726);
}
void pose_H_13(double *state, double *unused, double *out_3512054353587209250) {
  H_13(state, unused, out_3512054353587209250);
}
void pose_h_14(double *state, double *unused, double *out_3383842427869284419) {
  h_14(state, unused, out_3383842427869284419);
}
void pose_H_14(double *state, double *unused, double *out_135335998390007150) {
  H_14(state, unused, out_135335998390007150);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)

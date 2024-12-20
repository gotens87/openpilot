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
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3842342068078672063) {
   out_3842342068078672063[0] = delta_x[0] + nom_x[0];
   out_3842342068078672063[1] = delta_x[1] + nom_x[1];
   out_3842342068078672063[2] = delta_x[2] + nom_x[2];
   out_3842342068078672063[3] = delta_x[3] + nom_x[3];
   out_3842342068078672063[4] = delta_x[4] + nom_x[4];
   out_3842342068078672063[5] = delta_x[5] + nom_x[5];
   out_3842342068078672063[6] = delta_x[6] + nom_x[6];
   out_3842342068078672063[7] = delta_x[7] + nom_x[7];
   out_3842342068078672063[8] = delta_x[8] + nom_x[8];
   out_3842342068078672063[9] = delta_x[9] + nom_x[9];
   out_3842342068078672063[10] = delta_x[10] + nom_x[10];
   out_3842342068078672063[11] = delta_x[11] + nom_x[11];
   out_3842342068078672063[12] = delta_x[12] + nom_x[12];
   out_3842342068078672063[13] = delta_x[13] + nom_x[13];
   out_3842342068078672063[14] = delta_x[14] + nom_x[14];
   out_3842342068078672063[15] = delta_x[15] + nom_x[15];
   out_3842342068078672063[16] = delta_x[16] + nom_x[16];
   out_3842342068078672063[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8486186134848101365) {
   out_8486186134848101365[0] = -nom_x[0] + true_x[0];
   out_8486186134848101365[1] = -nom_x[1] + true_x[1];
   out_8486186134848101365[2] = -nom_x[2] + true_x[2];
   out_8486186134848101365[3] = -nom_x[3] + true_x[3];
   out_8486186134848101365[4] = -nom_x[4] + true_x[4];
   out_8486186134848101365[5] = -nom_x[5] + true_x[5];
   out_8486186134848101365[6] = -nom_x[6] + true_x[6];
   out_8486186134848101365[7] = -nom_x[7] + true_x[7];
   out_8486186134848101365[8] = -nom_x[8] + true_x[8];
   out_8486186134848101365[9] = -nom_x[9] + true_x[9];
   out_8486186134848101365[10] = -nom_x[10] + true_x[10];
   out_8486186134848101365[11] = -nom_x[11] + true_x[11];
   out_8486186134848101365[12] = -nom_x[12] + true_x[12];
   out_8486186134848101365[13] = -nom_x[13] + true_x[13];
   out_8486186134848101365[14] = -nom_x[14] + true_x[14];
   out_8486186134848101365[15] = -nom_x[15] + true_x[15];
   out_8486186134848101365[16] = -nom_x[16] + true_x[16];
   out_8486186134848101365[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_6473267640003251484) {
   out_6473267640003251484[0] = 1.0;
   out_6473267640003251484[1] = 0.0;
   out_6473267640003251484[2] = 0.0;
   out_6473267640003251484[3] = 0.0;
   out_6473267640003251484[4] = 0.0;
   out_6473267640003251484[5] = 0.0;
   out_6473267640003251484[6] = 0.0;
   out_6473267640003251484[7] = 0.0;
   out_6473267640003251484[8] = 0.0;
   out_6473267640003251484[9] = 0.0;
   out_6473267640003251484[10] = 0.0;
   out_6473267640003251484[11] = 0.0;
   out_6473267640003251484[12] = 0.0;
   out_6473267640003251484[13] = 0.0;
   out_6473267640003251484[14] = 0.0;
   out_6473267640003251484[15] = 0.0;
   out_6473267640003251484[16] = 0.0;
   out_6473267640003251484[17] = 0.0;
   out_6473267640003251484[18] = 0.0;
   out_6473267640003251484[19] = 1.0;
   out_6473267640003251484[20] = 0.0;
   out_6473267640003251484[21] = 0.0;
   out_6473267640003251484[22] = 0.0;
   out_6473267640003251484[23] = 0.0;
   out_6473267640003251484[24] = 0.0;
   out_6473267640003251484[25] = 0.0;
   out_6473267640003251484[26] = 0.0;
   out_6473267640003251484[27] = 0.0;
   out_6473267640003251484[28] = 0.0;
   out_6473267640003251484[29] = 0.0;
   out_6473267640003251484[30] = 0.0;
   out_6473267640003251484[31] = 0.0;
   out_6473267640003251484[32] = 0.0;
   out_6473267640003251484[33] = 0.0;
   out_6473267640003251484[34] = 0.0;
   out_6473267640003251484[35] = 0.0;
   out_6473267640003251484[36] = 0.0;
   out_6473267640003251484[37] = 0.0;
   out_6473267640003251484[38] = 1.0;
   out_6473267640003251484[39] = 0.0;
   out_6473267640003251484[40] = 0.0;
   out_6473267640003251484[41] = 0.0;
   out_6473267640003251484[42] = 0.0;
   out_6473267640003251484[43] = 0.0;
   out_6473267640003251484[44] = 0.0;
   out_6473267640003251484[45] = 0.0;
   out_6473267640003251484[46] = 0.0;
   out_6473267640003251484[47] = 0.0;
   out_6473267640003251484[48] = 0.0;
   out_6473267640003251484[49] = 0.0;
   out_6473267640003251484[50] = 0.0;
   out_6473267640003251484[51] = 0.0;
   out_6473267640003251484[52] = 0.0;
   out_6473267640003251484[53] = 0.0;
   out_6473267640003251484[54] = 0.0;
   out_6473267640003251484[55] = 0.0;
   out_6473267640003251484[56] = 0.0;
   out_6473267640003251484[57] = 1.0;
   out_6473267640003251484[58] = 0.0;
   out_6473267640003251484[59] = 0.0;
   out_6473267640003251484[60] = 0.0;
   out_6473267640003251484[61] = 0.0;
   out_6473267640003251484[62] = 0.0;
   out_6473267640003251484[63] = 0.0;
   out_6473267640003251484[64] = 0.0;
   out_6473267640003251484[65] = 0.0;
   out_6473267640003251484[66] = 0.0;
   out_6473267640003251484[67] = 0.0;
   out_6473267640003251484[68] = 0.0;
   out_6473267640003251484[69] = 0.0;
   out_6473267640003251484[70] = 0.0;
   out_6473267640003251484[71] = 0.0;
   out_6473267640003251484[72] = 0.0;
   out_6473267640003251484[73] = 0.0;
   out_6473267640003251484[74] = 0.0;
   out_6473267640003251484[75] = 0.0;
   out_6473267640003251484[76] = 1.0;
   out_6473267640003251484[77] = 0.0;
   out_6473267640003251484[78] = 0.0;
   out_6473267640003251484[79] = 0.0;
   out_6473267640003251484[80] = 0.0;
   out_6473267640003251484[81] = 0.0;
   out_6473267640003251484[82] = 0.0;
   out_6473267640003251484[83] = 0.0;
   out_6473267640003251484[84] = 0.0;
   out_6473267640003251484[85] = 0.0;
   out_6473267640003251484[86] = 0.0;
   out_6473267640003251484[87] = 0.0;
   out_6473267640003251484[88] = 0.0;
   out_6473267640003251484[89] = 0.0;
   out_6473267640003251484[90] = 0.0;
   out_6473267640003251484[91] = 0.0;
   out_6473267640003251484[92] = 0.0;
   out_6473267640003251484[93] = 0.0;
   out_6473267640003251484[94] = 0.0;
   out_6473267640003251484[95] = 1.0;
   out_6473267640003251484[96] = 0.0;
   out_6473267640003251484[97] = 0.0;
   out_6473267640003251484[98] = 0.0;
   out_6473267640003251484[99] = 0.0;
   out_6473267640003251484[100] = 0.0;
   out_6473267640003251484[101] = 0.0;
   out_6473267640003251484[102] = 0.0;
   out_6473267640003251484[103] = 0.0;
   out_6473267640003251484[104] = 0.0;
   out_6473267640003251484[105] = 0.0;
   out_6473267640003251484[106] = 0.0;
   out_6473267640003251484[107] = 0.0;
   out_6473267640003251484[108] = 0.0;
   out_6473267640003251484[109] = 0.0;
   out_6473267640003251484[110] = 0.0;
   out_6473267640003251484[111] = 0.0;
   out_6473267640003251484[112] = 0.0;
   out_6473267640003251484[113] = 0.0;
   out_6473267640003251484[114] = 1.0;
   out_6473267640003251484[115] = 0.0;
   out_6473267640003251484[116] = 0.0;
   out_6473267640003251484[117] = 0.0;
   out_6473267640003251484[118] = 0.0;
   out_6473267640003251484[119] = 0.0;
   out_6473267640003251484[120] = 0.0;
   out_6473267640003251484[121] = 0.0;
   out_6473267640003251484[122] = 0.0;
   out_6473267640003251484[123] = 0.0;
   out_6473267640003251484[124] = 0.0;
   out_6473267640003251484[125] = 0.0;
   out_6473267640003251484[126] = 0.0;
   out_6473267640003251484[127] = 0.0;
   out_6473267640003251484[128] = 0.0;
   out_6473267640003251484[129] = 0.0;
   out_6473267640003251484[130] = 0.0;
   out_6473267640003251484[131] = 0.0;
   out_6473267640003251484[132] = 0.0;
   out_6473267640003251484[133] = 1.0;
   out_6473267640003251484[134] = 0.0;
   out_6473267640003251484[135] = 0.0;
   out_6473267640003251484[136] = 0.0;
   out_6473267640003251484[137] = 0.0;
   out_6473267640003251484[138] = 0.0;
   out_6473267640003251484[139] = 0.0;
   out_6473267640003251484[140] = 0.0;
   out_6473267640003251484[141] = 0.0;
   out_6473267640003251484[142] = 0.0;
   out_6473267640003251484[143] = 0.0;
   out_6473267640003251484[144] = 0.0;
   out_6473267640003251484[145] = 0.0;
   out_6473267640003251484[146] = 0.0;
   out_6473267640003251484[147] = 0.0;
   out_6473267640003251484[148] = 0.0;
   out_6473267640003251484[149] = 0.0;
   out_6473267640003251484[150] = 0.0;
   out_6473267640003251484[151] = 0.0;
   out_6473267640003251484[152] = 1.0;
   out_6473267640003251484[153] = 0.0;
   out_6473267640003251484[154] = 0.0;
   out_6473267640003251484[155] = 0.0;
   out_6473267640003251484[156] = 0.0;
   out_6473267640003251484[157] = 0.0;
   out_6473267640003251484[158] = 0.0;
   out_6473267640003251484[159] = 0.0;
   out_6473267640003251484[160] = 0.0;
   out_6473267640003251484[161] = 0.0;
   out_6473267640003251484[162] = 0.0;
   out_6473267640003251484[163] = 0.0;
   out_6473267640003251484[164] = 0.0;
   out_6473267640003251484[165] = 0.0;
   out_6473267640003251484[166] = 0.0;
   out_6473267640003251484[167] = 0.0;
   out_6473267640003251484[168] = 0.0;
   out_6473267640003251484[169] = 0.0;
   out_6473267640003251484[170] = 0.0;
   out_6473267640003251484[171] = 1.0;
   out_6473267640003251484[172] = 0.0;
   out_6473267640003251484[173] = 0.0;
   out_6473267640003251484[174] = 0.0;
   out_6473267640003251484[175] = 0.0;
   out_6473267640003251484[176] = 0.0;
   out_6473267640003251484[177] = 0.0;
   out_6473267640003251484[178] = 0.0;
   out_6473267640003251484[179] = 0.0;
   out_6473267640003251484[180] = 0.0;
   out_6473267640003251484[181] = 0.0;
   out_6473267640003251484[182] = 0.0;
   out_6473267640003251484[183] = 0.0;
   out_6473267640003251484[184] = 0.0;
   out_6473267640003251484[185] = 0.0;
   out_6473267640003251484[186] = 0.0;
   out_6473267640003251484[187] = 0.0;
   out_6473267640003251484[188] = 0.0;
   out_6473267640003251484[189] = 0.0;
   out_6473267640003251484[190] = 1.0;
   out_6473267640003251484[191] = 0.0;
   out_6473267640003251484[192] = 0.0;
   out_6473267640003251484[193] = 0.0;
   out_6473267640003251484[194] = 0.0;
   out_6473267640003251484[195] = 0.0;
   out_6473267640003251484[196] = 0.0;
   out_6473267640003251484[197] = 0.0;
   out_6473267640003251484[198] = 0.0;
   out_6473267640003251484[199] = 0.0;
   out_6473267640003251484[200] = 0.0;
   out_6473267640003251484[201] = 0.0;
   out_6473267640003251484[202] = 0.0;
   out_6473267640003251484[203] = 0.0;
   out_6473267640003251484[204] = 0.0;
   out_6473267640003251484[205] = 0.0;
   out_6473267640003251484[206] = 0.0;
   out_6473267640003251484[207] = 0.0;
   out_6473267640003251484[208] = 0.0;
   out_6473267640003251484[209] = 1.0;
   out_6473267640003251484[210] = 0.0;
   out_6473267640003251484[211] = 0.0;
   out_6473267640003251484[212] = 0.0;
   out_6473267640003251484[213] = 0.0;
   out_6473267640003251484[214] = 0.0;
   out_6473267640003251484[215] = 0.0;
   out_6473267640003251484[216] = 0.0;
   out_6473267640003251484[217] = 0.0;
   out_6473267640003251484[218] = 0.0;
   out_6473267640003251484[219] = 0.0;
   out_6473267640003251484[220] = 0.0;
   out_6473267640003251484[221] = 0.0;
   out_6473267640003251484[222] = 0.0;
   out_6473267640003251484[223] = 0.0;
   out_6473267640003251484[224] = 0.0;
   out_6473267640003251484[225] = 0.0;
   out_6473267640003251484[226] = 0.0;
   out_6473267640003251484[227] = 0.0;
   out_6473267640003251484[228] = 1.0;
   out_6473267640003251484[229] = 0.0;
   out_6473267640003251484[230] = 0.0;
   out_6473267640003251484[231] = 0.0;
   out_6473267640003251484[232] = 0.0;
   out_6473267640003251484[233] = 0.0;
   out_6473267640003251484[234] = 0.0;
   out_6473267640003251484[235] = 0.0;
   out_6473267640003251484[236] = 0.0;
   out_6473267640003251484[237] = 0.0;
   out_6473267640003251484[238] = 0.0;
   out_6473267640003251484[239] = 0.0;
   out_6473267640003251484[240] = 0.0;
   out_6473267640003251484[241] = 0.0;
   out_6473267640003251484[242] = 0.0;
   out_6473267640003251484[243] = 0.0;
   out_6473267640003251484[244] = 0.0;
   out_6473267640003251484[245] = 0.0;
   out_6473267640003251484[246] = 0.0;
   out_6473267640003251484[247] = 1.0;
   out_6473267640003251484[248] = 0.0;
   out_6473267640003251484[249] = 0.0;
   out_6473267640003251484[250] = 0.0;
   out_6473267640003251484[251] = 0.0;
   out_6473267640003251484[252] = 0.0;
   out_6473267640003251484[253] = 0.0;
   out_6473267640003251484[254] = 0.0;
   out_6473267640003251484[255] = 0.0;
   out_6473267640003251484[256] = 0.0;
   out_6473267640003251484[257] = 0.0;
   out_6473267640003251484[258] = 0.0;
   out_6473267640003251484[259] = 0.0;
   out_6473267640003251484[260] = 0.0;
   out_6473267640003251484[261] = 0.0;
   out_6473267640003251484[262] = 0.0;
   out_6473267640003251484[263] = 0.0;
   out_6473267640003251484[264] = 0.0;
   out_6473267640003251484[265] = 0.0;
   out_6473267640003251484[266] = 1.0;
   out_6473267640003251484[267] = 0.0;
   out_6473267640003251484[268] = 0.0;
   out_6473267640003251484[269] = 0.0;
   out_6473267640003251484[270] = 0.0;
   out_6473267640003251484[271] = 0.0;
   out_6473267640003251484[272] = 0.0;
   out_6473267640003251484[273] = 0.0;
   out_6473267640003251484[274] = 0.0;
   out_6473267640003251484[275] = 0.0;
   out_6473267640003251484[276] = 0.0;
   out_6473267640003251484[277] = 0.0;
   out_6473267640003251484[278] = 0.0;
   out_6473267640003251484[279] = 0.0;
   out_6473267640003251484[280] = 0.0;
   out_6473267640003251484[281] = 0.0;
   out_6473267640003251484[282] = 0.0;
   out_6473267640003251484[283] = 0.0;
   out_6473267640003251484[284] = 0.0;
   out_6473267640003251484[285] = 1.0;
   out_6473267640003251484[286] = 0.0;
   out_6473267640003251484[287] = 0.0;
   out_6473267640003251484[288] = 0.0;
   out_6473267640003251484[289] = 0.0;
   out_6473267640003251484[290] = 0.0;
   out_6473267640003251484[291] = 0.0;
   out_6473267640003251484[292] = 0.0;
   out_6473267640003251484[293] = 0.0;
   out_6473267640003251484[294] = 0.0;
   out_6473267640003251484[295] = 0.0;
   out_6473267640003251484[296] = 0.0;
   out_6473267640003251484[297] = 0.0;
   out_6473267640003251484[298] = 0.0;
   out_6473267640003251484[299] = 0.0;
   out_6473267640003251484[300] = 0.0;
   out_6473267640003251484[301] = 0.0;
   out_6473267640003251484[302] = 0.0;
   out_6473267640003251484[303] = 0.0;
   out_6473267640003251484[304] = 1.0;
   out_6473267640003251484[305] = 0.0;
   out_6473267640003251484[306] = 0.0;
   out_6473267640003251484[307] = 0.0;
   out_6473267640003251484[308] = 0.0;
   out_6473267640003251484[309] = 0.0;
   out_6473267640003251484[310] = 0.0;
   out_6473267640003251484[311] = 0.0;
   out_6473267640003251484[312] = 0.0;
   out_6473267640003251484[313] = 0.0;
   out_6473267640003251484[314] = 0.0;
   out_6473267640003251484[315] = 0.0;
   out_6473267640003251484[316] = 0.0;
   out_6473267640003251484[317] = 0.0;
   out_6473267640003251484[318] = 0.0;
   out_6473267640003251484[319] = 0.0;
   out_6473267640003251484[320] = 0.0;
   out_6473267640003251484[321] = 0.0;
   out_6473267640003251484[322] = 0.0;
   out_6473267640003251484[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_6565120308118865312) {
   out_6565120308118865312[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_6565120308118865312[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_6565120308118865312[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_6565120308118865312[3] = dt*state[12] + state[3];
   out_6565120308118865312[4] = dt*state[13] + state[4];
   out_6565120308118865312[5] = dt*state[14] + state[5];
   out_6565120308118865312[6] = state[6];
   out_6565120308118865312[7] = state[7];
   out_6565120308118865312[8] = state[8];
   out_6565120308118865312[9] = state[9];
   out_6565120308118865312[10] = state[10];
   out_6565120308118865312[11] = state[11];
   out_6565120308118865312[12] = state[12];
   out_6565120308118865312[13] = state[13];
   out_6565120308118865312[14] = state[14];
   out_6565120308118865312[15] = state[15];
   out_6565120308118865312[16] = state[16];
   out_6565120308118865312[17] = state[17];
}
void F_fun(double *state, double dt, double *out_5609468243508396124) {
   out_5609468243508396124[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5609468243508396124[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5609468243508396124[2] = 0;
   out_5609468243508396124[3] = 0;
   out_5609468243508396124[4] = 0;
   out_5609468243508396124[5] = 0;
   out_5609468243508396124[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5609468243508396124[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5609468243508396124[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5609468243508396124[9] = 0;
   out_5609468243508396124[10] = 0;
   out_5609468243508396124[11] = 0;
   out_5609468243508396124[12] = 0;
   out_5609468243508396124[13] = 0;
   out_5609468243508396124[14] = 0;
   out_5609468243508396124[15] = 0;
   out_5609468243508396124[16] = 0;
   out_5609468243508396124[17] = 0;
   out_5609468243508396124[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5609468243508396124[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5609468243508396124[20] = 0;
   out_5609468243508396124[21] = 0;
   out_5609468243508396124[22] = 0;
   out_5609468243508396124[23] = 0;
   out_5609468243508396124[24] = 0;
   out_5609468243508396124[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5609468243508396124[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5609468243508396124[27] = 0;
   out_5609468243508396124[28] = 0;
   out_5609468243508396124[29] = 0;
   out_5609468243508396124[30] = 0;
   out_5609468243508396124[31] = 0;
   out_5609468243508396124[32] = 0;
   out_5609468243508396124[33] = 0;
   out_5609468243508396124[34] = 0;
   out_5609468243508396124[35] = 0;
   out_5609468243508396124[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5609468243508396124[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5609468243508396124[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5609468243508396124[39] = 0;
   out_5609468243508396124[40] = 0;
   out_5609468243508396124[41] = 0;
   out_5609468243508396124[42] = 0;
   out_5609468243508396124[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5609468243508396124[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5609468243508396124[45] = 0;
   out_5609468243508396124[46] = 0;
   out_5609468243508396124[47] = 0;
   out_5609468243508396124[48] = 0;
   out_5609468243508396124[49] = 0;
   out_5609468243508396124[50] = 0;
   out_5609468243508396124[51] = 0;
   out_5609468243508396124[52] = 0;
   out_5609468243508396124[53] = 0;
   out_5609468243508396124[54] = 0;
   out_5609468243508396124[55] = 0;
   out_5609468243508396124[56] = 0;
   out_5609468243508396124[57] = 1;
   out_5609468243508396124[58] = 0;
   out_5609468243508396124[59] = 0;
   out_5609468243508396124[60] = 0;
   out_5609468243508396124[61] = 0;
   out_5609468243508396124[62] = 0;
   out_5609468243508396124[63] = 0;
   out_5609468243508396124[64] = 0;
   out_5609468243508396124[65] = 0;
   out_5609468243508396124[66] = dt;
   out_5609468243508396124[67] = 0;
   out_5609468243508396124[68] = 0;
   out_5609468243508396124[69] = 0;
   out_5609468243508396124[70] = 0;
   out_5609468243508396124[71] = 0;
   out_5609468243508396124[72] = 0;
   out_5609468243508396124[73] = 0;
   out_5609468243508396124[74] = 0;
   out_5609468243508396124[75] = 0;
   out_5609468243508396124[76] = 1;
   out_5609468243508396124[77] = 0;
   out_5609468243508396124[78] = 0;
   out_5609468243508396124[79] = 0;
   out_5609468243508396124[80] = 0;
   out_5609468243508396124[81] = 0;
   out_5609468243508396124[82] = 0;
   out_5609468243508396124[83] = 0;
   out_5609468243508396124[84] = 0;
   out_5609468243508396124[85] = dt;
   out_5609468243508396124[86] = 0;
   out_5609468243508396124[87] = 0;
   out_5609468243508396124[88] = 0;
   out_5609468243508396124[89] = 0;
   out_5609468243508396124[90] = 0;
   out_5609468243508396124[91] = 0;
   out_5609468243508396124[92] = 0;
   out_5609468243508396124[93] = 0;
   out_5609468243508396124[94] = 0;
   out_5609468243508396124[95] = 1;
   out_5609468243508396124[96] = 0;
   out_5609468243508396124[97] = 0;
   out_5609468243508396124[98] = 0;
   out_5609468243508396124[99] = 0;
   out_5609468243508396124[100] = 0;
   out_5609468243508396124[101] = 0;
   out_5609468243508396124[102] = 0;
   out_5609468243508396124[103] = 0;
   out_5609468243508396124[104] = dt;
   out_5609468243508396124[105] = 0;
   out_5609468243508396124[106] = 0;
   out_5609468243508396124[107] = 0;
   out_5609468243508396124[108] = 0;
   out_5609468243508396124[109] = 0;
   out_5609468243508396124[110] = 0;
   out_5609468243508396124[111] = 0;
   out_5609468243508396124[112] = 0;
   out_5609468243508396124[113] = 0;
   out_5609468243508396124[114] = 1;
   out_5609468243508396124[115] = 0;
   out_5609468243508396124[116] = 0;
   out_5609468243508396124[117] = 0;
   out_5609468243508396124[118] = 0;
   out_5609468243508396124[119] = 0;
   out_5609468243508396124[120] = 0;
   out_5609468243508396124[121] = 0;
   out_5609468243508396124[122] = 0;
   out_5609468243508396124[123] = 0;
   out_5609468243508396124[124] = 0;
   out_5609468243508396124[125] = 0;
   out_5609468243508396124[126] = 0;
   out_5609468243508396124[127] = 0;
   out_5609468243508396124[128] = 0;
   out_5609468243508396124[129] = 0;
   out_5609468243508396124[130] = 0;
   out_5609468243508396124[131] = 0;
   out_5609468243508396124[132] = 0;
   out_5609468243508396124[133] = 1;
   out_5609468243508396124[134] = 0;
   out_5609468243508396124[135] = 0;
   out_5609468243508396124[136] = 0;
   out_5609468243508396124[137] = 0;
   out_5609468243508396124[138] = 0;
   out_5609468243508396124[139] = 0;
   out_5609468243508396124[140] = 0;
   out_5609468243508396124[141] = 0;
   out_5609468243508396124[142] = 0;
   out_5609468243508396124[143] = 0;
   out_5609468243508396124[144] = 0;
   out_5609468243508396124[145] = 0;
   out_5609468243508396124[146] = 0;
   out_5609468243508396124[147] = 0;
   out_5609468243508396124[148] = 0;
   out_5609468243508396124[149] = 0;
   out_5609468243508396124[150] = 0;
   out_5609468243508396124[151] = 0;
   out_5609468243508396124[152] = 1;
   out_5609468243508396124[153] = 0;
   out_5609468243508396124[154] = 0;
   out_5609468243508396124[155] = 0;
   out_5609468243508396124[156] = 0;
   out_5609468243508396124[157] = 0;
   out_5609468243508396124[158] = 0;
   out_5609468243508396124[159] = 0;
   out_5609468243508396124[160] = 0;
   out_5609468243508396124[161] = 0;
   out_5609468243508396124[162] = 0;
   out_5609468243508396124[163] = 0;
   out_5609468243508396124[164] = 0;
   out_5609468243508396124[165] = 0;
   out_5609468243508396124[166] = 0;
   out_5609468243508396124[167] = 0;
   out_5609468243508396124[168] = 0;
   out_5609468243508396124[169] = 0;
   out_5609468243508396124[170] = 0;
   out_5609468243508396124[171] = 1;
   out_5609468243508396124[172] = 0;
   out_5609468243508396124[173] = 0;
   out_5609468243508396124[174] = 0;
   out_5609468243508396124[175] = 0;
   out_5609468243508396124[176] = 0;
   out_5609468243508396124[177] = 0;
   out_5609468243508396124[178] = 0;
   out_5609468243508396124[179] = 0;
   out_5609468243508396124[180] = 0;
   out_5609468243508396124[181] = 0;
   out_5609468243508396124[182] = 0;
   out_5609468243508396124[183] = 0;
   out_5609468243508396124[184] = 0;
   out_5609468243508396124[185] = 0;
   out_5609468243508396124[186] = 0;
   out_5609468243508396124[187] = 0;
   out_5609468243508396124[188] = 0;
   out_5609468243508396124[189] = 0;
   out_5609468243508396124[190] = 1;
   out_5609468243508396124[191] = 0;
   out_5609468243508396124[192] = 0;
   out_5609468243508396124[193] = 0;
   out_5609468243508396124[194] = 0;
   out_5609468243508396124[195] = 0;
   out_5609468243508396124[196] = 0;
   out_5609468243508396124[197] = 0;
   out_5609468243508396124[198] = 0;
   out_5609468243508396124[199] = 0;
   out_5609468243508396124[200] = 0;
   out_5609468243508396124[201] = 0;
   out_5609468243508396124[202] = 0;
   out_5609468243508396124[203] = 0;
   out_5609468243508396124[204] = 0;
   out_5609468243508396124[205] = 0;
   out_5609468243508396124[206] = 0;
   out_5609468243508396124[207] = 0;
   out_5609468243508396124[208] = 0;
   out_5609468243508396124[209] = 1;
   out_5609468243508396124[210] = 0;
   out_5609468243508396124[211] = 0;
   out_5609468243508396124[212] = 0;
   out_5609468243508396124[213] = 0;
   out_5609468243508396124[214] = 0;
   out_5609468243508396124[215] = 0;
   out_5609468243508396124[216] = 0;
   out_5609468243508396124[217] = 0;
   out_5609468243508396124[218] = 0;
   out_5609468243508396124[219] = 0;
   out_5609468243508396124[220] = 0;
   out_5609468243508396124[221] = 0;
   out_5609468243508396124[222] = 0;
   out_5609468243508396124[223] = 0;
   out_5609468243508396124[224] = 0;
   out_5609468243508396124[225] = 0;
   out_5609468243508396124[226] = 0;
   out_5609468243508396124[227] = 0;
   out_5609468243508396124[228] = 1;
   out_5609468243508396124[229] = 0;
   out_5609468243508396124[230] = 0;
   out_5609468243508396124[231] = 0;
   out_5609468243508396124[232] = 0;
   out_5609468243508396124[233] = 0;
   out_5609468243508396124[234] = 0;
   out_5609468243508396124[235] = 0;
   out_5609468243508396124[236] = 0;
   out_5609468243508396124[237] = 0;
   out_5609468243508396124[238] = 0;
   out_5609468243508396124[239] = 0;
   out_5609468243508396124[240] = 0;
   out_5609468243508396124[241] = 0;
   out_5609468243508396124[242] = 0;
   out_5609468243508396124[243] = 0;
   out_5609468243508396124[244] = 0;
   out_5609468243508396124[245] = 0;
   out_5609468243508396124[246] = 0;
   out_5609468243508396124[247] = 1;
   out_5609468243508396124[248] = 0;
   out_5609468243508396124[249] = 0;
   out_5609468243508396124[250] = 0;
   out_5609468243508396124[251] = 0;
   out_5609468243508396124[252] = 0;
   out_5609468243508396124[253] = 0;
   out_5609468243508396124[254] = 0;
   out_5609468243508396124[255] = 0;
   out_5609468243508396124[256] = 0;
   out_5609468243508396124[257] = 0;
   out_5609468243508396124[258] = 0;
   out_5609468243508396124[259] = 0;
   out_5609468243508396124[260] = 0;
   out_5609468243508396124[261] = 0;
   out_5609468243508396124[262] = 0;
   out_5609468243508396124[263] = 0;
   out_5609468243508396124[264] = 0;
   out_5609468243508396124[265] = 0;
   out_5609468243508396124[266] = 1;
   out_5609468243508396124[267] = 0;
   out_5609468243508396124[268] = 0;
   out_5609468243508396124[269] = 0;
   out_5609468243508396124[270] = 0;
   out_5609468243508396124[271] = 0;
   out_5609468243508396124[272] = 0;
   out_5609468243508396124[273] = 0;
   out_5609468243508396124[274] = 0;
   out_5609468243508396124[275] = 0;
   out_5609468243508396124[276] = 0;
   out_5609468243508396124[277] = 0;
   out_5609468243508396124[278] = 0;
   out_5609468243508396124[279] = 0;
   out_5609468243508396124[280] = 0;
   out_5609468243508396124[281] = 0;
   out_5609468243508396124[282] = 0;
   out_5609468243508396124[283] = 0;
   out_5609468243508396124[284] = 0;
   out_5609468243508396124[285] = 1;
   out_5609468243508396124[286] = 0;
   out_5609468243508396124[287] = 0;
   out_5609468243508396124[288] = 0;
   out_5609468243508396124[289] = 0;
   out_5609468243508396124[290] = 0;
   out_5609468243508396124[291] = 0;
   out_5609468243508396124[292] = 0;
   out_5609468243508396124[293] = 0;
   out_5609468243508396124[294] = 0;
   out_5609468243508396124[295] = 0;
   out_5609468243508396124[296] = 0;
   out_5609468243508396124[297] = 0;
   out_5609468243508396124[298] = 0;
   out_5609468243508396124[299] = 0;
   out_5609468243508396124[300] = 0;
   out_5609468243508396124[301] = 0;
   out_5609468243508396124[302] = 0;
   out_5609468243508396124[303] = 0;
   out_5609468243508396124[304] = 1;
   out_5609468243508396124[305] = 0;
   out_5609468243508396124[306] = 0;
   out_5609468243508396124[307] = 0;
   out_5609468243508396124[308] = 0;
   out_5609468243508396124[309] = 0;
   out_5609468243508396124[310] = 0;
   out_5609468243508396124[311] = 0;
   out_5609468243508396124[312] = 0;
   out_5609468243508396124[313] = 0;
   out_5609468243508396124[314] = 0;
   out_5609468243508396124[315] = 0;
   out_5609468243508396124[316] = 0;
   out_5609468243508396124[317] = 0;
   out_5609468243508396124[318] = 0;
   out_5609468243508396124[319] = 0;
   out_5609468243508396124[320] = 0;
   out_5609468243508396124[321] = 0;
   out_5609468243508396124[322] = 0;
   out_5609468243508396124[323] = 1;
}
void h_4(double *state, double *unused, double *out_4855683315993454061) {
   out_4855683315993454061[0] = state[6] + state[9];
   out_4855683315993454061[1] = state[7] + state[10];
   out_4855683315993454061[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_8201669223862392560) {
   out_8201669223862392560[0] = 0;
   out_8201669223862392560[1] = 0;
   out_8201669223862392560[2] = 0;
   out_8201669223862392560[3] = 0;
   out_8201669223862392560[4] = 0;
   out_8201669223862392560[5] = 0;
   out_8201669223862392560[6] = 1;
   out_8201669223862392560[7] = 0;
   out_8201669223862392560[8] = 0;
   out_8201669223862392560[9] = 1;
   out_8201669223862392560[10] = 0;
   out_8201669223862392560[11] = 0;
   out_8201669223862392560[12] = 0;
   out_8201669223862392560[13] = 0;
   out_8201669223862392560[14] = 0;
   out_8201669223862392560[15] = 0;
   out_8201669223862392560[16] = 0;
   out_8201669223862392560[17] = 0;
   out_8201669223862392560[18] = 0;
   out_8201669223862392560[19] = 0;
   out_8201669223862392560[20] = 0;
   out_8201669223862392560[21] = 0;
   out_8201669223862392560[22] = 0;
   out_8201669223862392560[23] = 0;
   out_8201669223862392560[24] = 0;
   out_8201669223862392560[25] = 1;
   out_8201669223862392560[26] = 0;
   out_8201669223862392560[27] = 0;
   out_8201669223862392560[28] = 1;
   out_8201669223862392560[29] = 0;
   out_8201669223862392560[30] = 0;
   out_8201669223862392560[31] = 0;
   out_8201669223862392560[32] = 0;
   out_8201669223862392560[33] = 0;
   out_8201669223862392560[34] = 0;
   out_8201669223862392560[35] = 0;
   out_8201669223862392560[36] = 0;
   out_8201669223862392560[37] = 0;
   out_8201669223862392560[38] = 0;
   out_8201669223862392560[39] = 0;
   out_8201669223862392560[40] = 0;
   out_8201669223862392560[41] = 0;
   out_8201669223862392560[42] = 0;
   out_8201669223862392560[43] = 0;
   out_8201669223862392560[44] = 1;
   out_8201669223862392560[45] = 0;
   out_8201669223862392560[46] = 0;
   out_8201669223862392560[47] = 1;
   out_8201669223862392560[48] = 0;
   out_8201669223862392560[49] = 0;
   out_8201669223862392560[50] = 0;
   out_8201669223862392560[51] = 0;
   out_8201669223862392560[52] = 0;
   out_8201669223862392560[53] = 0;
}
void h_10(double *state, double *unused, double *out_851235722336489475) {
   out_851235722336489475[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_851235722336489475[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_851235722336489475[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_3556931133651902201) {
   out_3556931133651902201[0] = 0;
   out_3556931133651902201[1] = 9.8100000000000005*cos(state[1]);
   out_3556931133651902201[2] = 0;
   out_3556931133651902201[3] = 0;
   out_3556931133651902201[4] = -state[8];
   out_3556931133651902201[5] = state[7];
   out_3556931133651902201[6] = 0;
   out_3556931133651902201[7] = state[5];
   out_3556931133651902201[8] = -state[4];
   out_3556931133651902201[9] = 0;
   out_3556931133651902201[10] = 0;
   out_3556931133651902201[11] = 0;
   out_3556931133651902201[12] = 1;
   out_3556931133651902201[13] = 0;
   out_3556931133651902201[14] = 0;
   out_3556931133651902201[15] = 1;
   out_3556931133651902201[16] = 0;
   out_3556931133651902201[17] = 0;
   out_3556931133651902201[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_3556931133651902201[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_3556931133651902201[20] = 0;
   out_3556931133651902201[21] = state[8];
   out_3556931133651902201[22] = 0;
   out_3556931133651902201[23] = -state[6];
   out_3556931133651902201[24] = -state[5];
   out_3556931133651902201[25] = 0;
   out_3556931133651902201[26] = state[3];
   out_3556931133651902201[27] = 0;
   out_3556931133651902201[28] = 0;
   out_3556931133651902201[29] = 0;
   out_3556931133651902201[30] = 0;
   out_3556931133651902201[31] = 1;
   out_3556931133651902201[32] = 0;
   out_3556931133651902201[33] = 0;
   out_3556931133651902201[34] = 1;
   out_3556931133651902201[35] = 0;
   out_3556931133651902201[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_3556931133651902201[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_3556931133651902201[38] = 0;
   out_3556931133651902201[39] = -state[7];
   out_3556931133651902201[40] = state[6];
   out_3556931133651902201[41] = 0;
   out_3556931133651902201[42] = state[4];
   out_3556931133651902201[43] = -state[3];
   out_3556931133651902201[44] = 0;
   out_3556931133651902201[45] = 0;
   out_3556931133651902201[46] = 0;
   out_3556931133651902201[47] = 0;
   out_3556931133651902201[48] = 0;
   out_3556931133651902201[49] = 0;
   out_3556931133651902201[50] = 1;
   out_3556931133651902201[51] = 0;
   out_3556931133651902201[52] = 0;
   out_3556931133651902201[53] = 1;
}
void h_13(double *state, double *unused, double *out_4193103824640271278) {
   out_4193103824640271278[0] = state[3];
   out_4193103824640271278[1] = state[4];
   out_4193103824640271278[2] = state[5];
}
void H_13(double *state, double *unused, double *out_591038015545691631) {
   out_591038015545691631[0] = 0;
   out_591038015545691631[1] = 0;
   out_591038015545691631[2] = 0;
   out_591038015545691631[3] = 1;
   out_591038015545691631[4] = 0;
   out_591038015545691631[5] = 0;
   out_591038015545691631[6] = 0;
   out_591038015545691631[7] = 0;
   out_591038015545691631[8] = 0;
   out_591038015545691631[9] = 0;
   out_591038015545691631[10] = 0;
   out_591038015545691631[11] = 0;
   out_591038015545691631[12] = 0;
   out_591038015545691631[13] = 0;
   out_591038015545691631[14] = 0;
   out_591038015545691631[15] = 0;
   out_591038015545691631[16] = 0;
   out_591038015545691631[17] = 0;
   out_591038015545691631[18] = 0;
   out_591038015545691631[19] = 0;
   out_591038015545691631[20] = 0;
   out_591038015545691631[21] = 0;
   out_591038015545691631[22] = 1;
   out_591038015545691631[23] = 0;
   out_591038015545691631[24] = 0;
   out_591038015545691631[25] = 0;
   out_591038015545691631[26] = 0;
   out_591038015545691631[27] = 0;
   out_591038015545691631[28] = 0;
   out_591038015545691631[29] = 0;
   out_591038015545691631[30] = 0;
   out_591038015545691631[31] = 0;
   out_591038015545691631[32] = 0;
   out_591038015545691631[33] = 0;
   out_591038015545691631[34] = 0;
   out_591038015545691631[35] = 0;
   out_591038015545691631[36] = 0;
   out_591038015545691631[37] = 0;
   out_591038015545691631[38] = 0;
   out_591038015545691631[39] = 0;
   out_591038015545691631[40] = 0;
   out_591038015545691631[41] = 1;
   out_591038015545691631[42] = 0;
   out_591038015545691631[43] = 0;
   out_591038015545691631[44] = 0;
   out_591038015545691631[45] = 0;
   out_591038015545691631[46] = 0;
   out_591038015545691631[47] = 0;
   out_591038015545691631[48] = 0;
   out_591038015545691631[49] = 0;
   out_591038015545691631[50] = 0;
   out_591038015545691631[51] = 0;
   out_591038015545691631[52] = 0;
   out_591038015545691631[53] = 0;
}
void h_14(double *state, double *unused, double *out_379962720619032263) {
   out_379962720619032263[0] = state[6];
   out_379962720619032263[1] = state[7];
   out_379962720619032263[2] = state[8];
}
void H_14(double *state, double *unused, double *out_7162286417551786760) {
   out_7162286417551786760[0] = 0;
   out_7162286417551786760[1] = 0;
   out_7162286417551786760[2] = 0;
   out_7162286417551786760[3] = 0;
   out_7162286417551786760[4] = 0;
   out_7162286417551786760[5] = 0;
   out_7162286417551786760[6] = 1;
   out_7162286417551786760[7] = 0;
   out_7162286417551786760[8] = 0;
   out_7162286417551786760[9] = 0;
   out_7162286417551786760[10] = 0;
   out_7162286417551786760[11] = 0;
   out_7162286417551786760[12] = 0;
   out_7162286417551786760[13] = 0;
   out_7162286417551786760[14] = 0;
   out_7162286417551786760[15] = 0;
   out_7162286417551786760[16] = 0;
   out_7162286417551786760[17] = 0;
   out_7162286417551786760[18] = 0;
   out_7162286417551786760[19] = 0;
   out_7162286417551786760[20] = 0;
   out_7162286417551786760[21] = 0;
   out_7162286417551786760[22] = 0;
   out_7162286417551786760[23] = 0;
   out_7162286417551786760[24] = 0;
   out_7162286417551786760[25] = 1;
   out_7162286417551786760[26] = 0;
   out_7162286417551786760[27] = 0;
   out_7162286417551786760[28] = 0;
   out_7162286417551786760[29] = 0;
   out_7162286417551786760[30] = 0;
   out_7162286417551786760[31] = 0;
   out_7162286417551786760[32] = 0;
   out_7162286417551786760[33] = 0;
   out_7162286417551786760[34] = 0;
   out_7162286417551786760[35] = 0;
   out_7162286417551786760[36] = 0;
   out_7162286417551786760[37] = 0;
   out_7162286417551786760[38] = 0;
   out_7162286417551786760[39] = 0;
   out_7162286417551786760[40] = 0;
   out_7162286417551786760[41] = 0;
   out_7162286417551786760[42] = 0;
   out_7162286417551786760[43] = 0;
   out_7162286417551786760[44] = 1;
   out_7162286417551786760[45] = 0;
   out_7162286417551786760[46] = 0;
   out_7162286417551786760[47] = 0;
   out_7162286417551786760[48] = 0;
   out_7162286417551786760[49] = 0;
   out_7162286417551786760[50] = 0;
   out_7162286417551786760[51] = 0;
   out_7162286417551786760[52] = 0;
   out_7162286417551786760[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_3842342068078672063) {
  err_fun(nom_x, delta_x, out_3842342068078672063);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_8486186134848101365) {
  inv_err_fun(nom_x, true_x, out_8486186134848101365);
}
void pose_H_mod_fun(double *state, double *out_6473267640003251484) {
  H_mod_fun(state, out_6473267640003251484);
}
void pose_f_fun(double *state, double dt, double *out_6565120308118865312) {
  f_fun(state,  dt, out_6565120308118865312);
}
void pose_F_fun(double *state, double dt, double *out_5609468243508396124) {
  F_fun(state,  dt, out_5609468243508396124);
}
void pose_h_4(double *state, double *unused, double *out_4855683315993454061) {
  h_4(state, unused, out_4855683315993454061);
}
void pose_H_4(double *state, double *unused, double *out_8201669223862392560) {
  H_4(state, unused, out_8201669223862392560);
}
void pose_h_10(double *state, double *unused, double *out_851235722336489475) {
  h_10(state, unused, out_851235722336489475);
}
void pose_H_10(double *state, double *unused, double *out_3556931133651902201) {
  H_10(state, unused, out_3556931133651902201);
}
void pose_h_13(double *state, double *unused, double *out_4193103824640271278) {
  h_13(state, unused, out_4193103824640271278);
}
void pose_H_13(double *state, double *unused, double *out_591038015545691631) {
  H_13(state, unused, out_591038015545691631);
}
void pose_h_14(double *state, double *unused, double *out_379962720619032263) {
  h_14(state, unused, out_379962720619032263);
}
void pose_H_14(double *state, double *unused, double *out_7162286417551786760) {
  H_14(state, unused, out_7162286417551786760);
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

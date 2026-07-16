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
void err_fun(double *nom_x, double *delta_x, double *out_3037457306540495367) {
   out_3037457306540495367[0] = delta_x[0] + nom_x[0];
   out_3037457306540495367[1] = delta_x[1] + nom_x[1];
   out_3037457306540495367[2] = delta_x[2] + nom_x[2];
   out_3037457306540495367[3] = delta_x[3] + nom_x[3];
   out_3037457306540495367[4] = delta_x[4] + nom_x[4];
   out_3037457306540495367[5] = delta_x[5] + nom_x[5];
   out_3037457306540495367[6] = delta_x[6] + nom_x[6];
   out_3037457306540495367[7] = delta_x[7] + nom_x[7];
   out_3037457306540495367[8] = delta_x[8] + nom_x[8];
   out_3037457306540495367[9] = delta_x[9] + nom_x[9];
   out_3037457306540495367[10] = delta_x[10] + nom_x[10];
   out_3037457306540495367[11] = delta_x[11] + nom_x[11];
   out_3037457306540495367[12] = delta_x[12] + nom_x[12];
   out_3037457306540495367[13] = delta_x[13] + nom_x[13];
   out_3037457306540495367[14] = delta_x[14] + nom_x[14];
   out_3037457306540495367[15] = delta_x[15] + nom_x[15];
   out_3037457306540495367[16] = delta_x[16] + nom_x[16];
   out_3037457306540495367[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5504742843249496146) {
   out_5504742843249496146[0] = -nom_x[0] + true_x[0];
   out_5504742843249496146[1] = -nom_x[1] + true_x[1];
   out_5504742843249496146[2] = -nom_x[2] + true_x[2];
   out_5504742843249496146[3] = -nom_x[3] + true_x[3];
   out_5504742843249496146[4] = -nom_x[4] + true_x[4];
   out_5504742843249496146[5] = -nom_x[5] + true_x[5];
   out_5504742843249496146[6] = -nom_x[6] + true_x[6];
   out_5504742843249496146[7] = -nom_x[7] + true_x[7];
   out_5504742843249496146[8] = -nom_x[8] + true_x[8];
   out_5504742843249496146[9] = -nom_x[9] + true_x[9];
   out_5504742843249496146[10] = -nom_x[10] + true_x[10];
   out_5504742843249496146[11] = -nom_x[11] + true_x[11];
   out_5504742843249496146[12] = -nom_x[12] + true_x[12];
   out_5504742843249496146[13] = -nom_x[13] + true_x[13];
   out_5504742843249496146[14] = -nom_x[14] + true_x[14];
   out_5504742843249496146[15] = -nom_x[15] + true_x[15];
   out_5504742843249496146[16] = -nom_x[16] + true_x[16];
   out_5504742843249496146[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4825348341631687231) {
   out_4825348341631687231[0] = 1.0;
   out_4825348341631687231[1] = 0.0;
   out_4825348341631687231[2] = 0.0;
   out_4825348341631687231[3] = 0.0;
   out_4825348341631687231[4] = 0.0;
   out_4825348341631687231[5] = 0.0;
   out_4825348341631687231[6] = 0.0;
   out_4825348341631687231[7] = 0.0;
   out_4825348341631687231[8] = 0.0;
   out_4825348341631687231[9] = 0.0;
   out_4825348341631687231[10] = 0.0;
   out_4825348341631687231[11] = 0.0;
   out_4825348341631687231[12] = 0.0;
   out_4825348341631687231[13] = 0.0;
   out_4825348341631687231[14] = 0.0;
   out_4825348341631687231[15] = 0.0;
   out_4825348341631687231[16] = 0.0;
   out_4825348341631687231[17] = 0.0;
   out_4825348341631687231[18] = 0.0;
   out_4825348341631687231[19] = 1.0;
   out_4825348341631687231[20] = 0.0;
   out_4825348341631687231[21] = 0.0;
   out_4825348341631687231[22] = 0.0;
   out_4825348341631687231[23] = 0.0;
   out_4825348341631687231[24] = 0.0;
   out_4825348341631687231[25] = 0.0;
   out_4825348341631687231[26] = 0.0;
   out_4825348341631687231[27] = 0.0;
   out_4825348341631687231[28] = 0.0;
   out_4825348341631687231[29] = 0.0;
   out_4825348341631687231[30] = 0.0;
   out_4825348341631687231[31] = 0.0;
   out_4825348341631687231[32] = 0.0;
   out_4825348341631687231[33] = 0.0;
   out_4825348341631687231[34] = 0.0;
   out_4825348341631687231[35] = 0.0;
   out_4825348341631687231[36] = 0.0;
   out_4825348341631687231[37] = 0.0;
   out_4825348341631687231[38] = 1.0;
   out_4825348341631687231[39] = 0.0;
   out_4825348341631687231[40] = 0.0;
   out_4825348341631687231[41] = 0.0;
   out_4825348341631687231[42] = 0.0;
   out_4825348341631687231[43] = 0.0;
   out_4825348341631687231[44] = 0.0;
   out_4825348341631687231[45] = 0.0;
   out_4825348341631687231[46] = 0.0;
   out_4825348341631687231[47] = 0.0;
   out_4825348341631687231[48] = 0.0;
   out_4825348341631687231[49] = 0.0;
   out_4825348341631687231[50] = 0.0;
   out_4825348341631687231[51] = 0.0;
   out_4825348341631687231[52] = 0.0;
   out_4825348341631687231[53] = 0.0;
   out_4825348341631687231[54] = 0.0;
   out_4825348341631687231[55] = 0.0;
   out_4825348341631687231[56] = 0.0;
   out_4825348341631687231[57] = 1.0;
   out_4825348341631687231[58] = 0.0;
   out_4825348341631687231[59] = 0.0;
   out_4825348341631687231[60] = 0.0;
   out_4825348341631687231[61] = 0.0;
   out_4825348341631687231[62] = 0.0;
   out_4825348341631687231[63] = 0.0;
   out_4825348341631687231[64] = 0.0;
   out_4825348341631687231[65] = 0.0;
   out_4825348341631687231[66] = 0.0;
   out_4825348341631687231[67] = 0.0;
   out_4825348341631687231[68] = 0.0;
   out_4825348341631687231[69] = 0.0;
   out_4825348341631687231[70] = 0.0;
   out_4825348341631687231[71] = 0.0;
   out_4825348341631687231[72] = 0.0;
   out_4825348341631687231[73] = 0.0;
   out_4825348341631687231[74] = 0.0;
   out_4825348341631687231[75] = 0.0;
   out_4825348341631687231[76] = 1.0;
   out_4825348341631687231[77] = 0.0;
   out_4825348341631687231[78] = 0.0;
   out_4825348341631687231[79] = 0.0;
   out_4825348341631687231[80] = 0.0;
   out_4825348341631687231[81] = 0.0;
   out_4825348341631687231[82] = 0.0;
   out_4825348341631687231[83] = 0.0;
   out_4825348341631687231[84] = 0.0;
   out_4825348341631687231[85] = 0.0;
   out_4825348341631687231[86] = 0.0;
   out_4825348341631687231[87] = 0.0;
   out_4825348341631687231[88] = 0.0;
   out_4825348341631687231[89] = 0.0;
   out_4825348341631687231[90] = 0.0;
   out_4825348341631687231[91] = 0.0;
   out_4825348341631687231[92] = 0.0;
   out_4825348341631687231[93] = 0.0;
   out_4825348341631687231[94] = 0.0;
   out_4825348341631687231[95] = 1.0;
   out_4825348341631687231[96] = 0.0;
   out_4825348341631687231[97] = 0.0;
   out_4825348341631687231[98] = 0.0;
   out_4825348341631687231[99] = 0.0;
   out_4825348341631687231[100] = 0.0;
   out_4825348341631687231[101] = 0.0;
   out_4825348341631687231[102] = 0.0;
   out_4825348341631687231[103] = 0.0;
   out_4825348341631687231[104] = 0.0;
   out_4825348341631687231[105] = 0.0;
   out_4825348341631687231[106] = 0.0;
   out_4825348341631687231[107] = 0.0;
   out_4825348341631687231[108] = 0.0;
   out_4825348341631687231[109] = 0.0;
   out_4825348341631687231[110] = 0.0;
   out_4825348341631687231[111] = 0.0;
   out_4825348341631687231[112] = 0.0;
   out_4825348341631687231[113] = 0.0;
   out_4825348341631687231[114] = 1.0;
   out_4825348341631687231[115] = 0.0;
   out_4825348341631687231[116] = 0.0;
   out_4825348341631687231[117] = 0.0;
   out_4825348341631687231[118] = 0.0;
   out_4825348341631687231[119] = 0.0;
   out_4825348341631687231[120] = 0.0;
   out_4825348341631687231[121] = 0.0;
   out_4825348341631687231[122] = 0.0;
   out_4825348341631687231[123] = 0.0;
   out_4825348341631687231[124] = 0.0;
   out_4825348341631687231[125] = 0.0;
   out_4825348341631687231[126] = 0.0;
   out_4825348341631687231[127] = 0.0;
   out_4825348341631687231[128] = 0.0;
   out_4825348341631687231[129] = 0.0;
   out_4825348341631687231[130] = 0.0;
   out_4825348341631687231[131] = 0.0;
   out_4825348341631687231[132] = 0.0;
   out_4825348341631687231[133] = 1.0;
   out_4825348341631687231[134] = 0.0;
   out_4825348341631687231[135] = 0.0;
   out_4825348341631687231[136] = 0.0;
   out_4825348341631687231[137] = 0.0;
   out_4825348341631687231[138] = 0.0;
   out_4825348341631687231[139] = 0.0;
   out_4825348341631687231[140] = 0.0;
   out_4825348341631687231[141] = 0.0;
   out_4825348341631687231[142] = 0.0;
   out_4825348341631687231[143] = 0.0;
   out_4825348341631687231[144] = 0.0;
   out_4825348341631687231[145] = 0.0;
   out_4825348341631687231[146] = 0.0;
   out_4825348341631687231[147] = 0.0;
   out_4825348341631687231[148] = 0.0;
   out_4825348341631687231[149] = 0.0;
   out_4825348341631687231[150] = 0.0;
   out_4825348341631687231[151] = 0.0;
   out_4825348341631687231[152] = 1.0;
   out_4825348341631687231[153] = 0.0;
   out_4825348341631687231[154] = 0.0;
   out_4825348341631687231[155] = 0.0;
   out_4825348341631687231[156] = 0.0;
   out_4825348341631687231[157] = 0.0;
   out_4825348341631687231[158] = 0.0;
   out_4825348341631687231[159] = 0.0;
   out_4825348341631687231[160] = 0.0;
   out_4825348341631687231[161] = 0.0;
   out_4825348341631687231[162] = 0.0;
   out_4825348341631687231[163] = 0.0;
   out_4825348341631687231[164] = 0.0;
   out_4825348341631687231[165] = 0.0;
   out_4825348341631687231[166] = 0.0;
   out_4825348341631687231[167] = 0.0;
   out_4825348341631687231[168] = 0.0;
   out_4825348341631687231[169] = 0.0;
   out_4825348341631687231[170] = 0.0;
   out_4825348341631687231[171] = 1.0;
   out_4825348341631687231[172] = 0.0;
   out_4825348341631687231[173] = 0.0;
   out_4825348341631687231[174] = 0.0;
   out_4825348341631687231[175] = 0.0;
   out_4825348341631687231[176] = 0.0;
   out_4825348341631687231[177] = 0.0;
   out_4825348341631687231[178] = 0.0;
   out_4825348341631687231[179] = 0.0;
   out_4825348341631687231[180] = 0.0;
   out_4825348341631687231[181] = 0.0;
   out_4825348341631687231[182] = 0.0;
   out_4825348341631687231[183] = 0.0;
   out_4825348341631687231[184] = 0.0;
   out_4825348341631687231[185] = 0.0;
   out_4825348341631687231[186] = 0.0;
   out_4825348341631687231[187] = 0.0;
   out_4825348341631687231[188] = 0.0;
   out_4825348341631687231[189] = 0.0;
   out_4825348341631687231[190] = 1.0;
   out_4825348341631687231[191] = 0.0;
   out_4825348341631687231[192] = 0.0;
   out_4825348341631687231[193] = 0.0;
   out_4825348341631687231[194] = 0.0;
   out_4825348341631687231[195] = 0.0;
   out_4825348341631687231[196] = 0.0;
   out_4825348341631687231[197] = 0.0;
   out_4825348341631687231[198] = 0.0;
   out_4825348341631687231[199] = 0.0;
   out_4825348341631687231[200] = 0.0;
   out_4825348341631687231[201] = 0.0;
   out_4825348341631687231[202] = 0.0;
   out_4825348341631687231[203] = 0.0;
   out_4825348341631687231[204] = 0.0;
   out_4825348341631687231[205] = 0.0;
   out_4825348341631687231[206] = 0.0;
   out_4825348341631687231[207] = 0.0;
   out_4825348341631687231[208] = 0.0;
   out_4825348341631687231[209] = 1.0;
   out_4825348341631687231[210] = 0.0;
   out_4825348341631687231[211] = 0.0;
   out_4825348341631687231[212] = 0.0;
   out_4825348341631687231[213] = 0.0;
   out_4825348341631687231[214] = 0.0;
   out_4825348341631687231[215] = 0.0;
   out_4825348341631687231[216] = 0.0;
   out_4825348341631687231[217] = 0.0;
   out_4825348341631687231[218] = 0.0;
   out_4825348341631687231[219] = 0.0;
   out_4825348341631687231[220] = 0.0;
   out_4825348341631687231[221] = 0.0;
   out_4825348341631687231[222] = 0.0;
   out_4825348341631687231[223] = 0.0;
   out_4825348341631687231[224] = 0.0;
   out_4825348341631687231[225] = 0.0;
   out_4825348341631687231[226] = 0.0;
   out_4825348341631687231[227] = 0.0;
   out_4825348341631687231[228] = 1.0;
   out_4825348341631687231[229] = 0.0;
   out_4825348341631687231[230] = 0.0;
   out_4825348341631687231[231] = 0.0;
   out_4825348341631687231[232] = 0.0;
   out_4825348341631687231[233] = 0.0;
   out_4825348341631687231[234] = 0.0;
   out_4825348341631687231[235] = 0.0;
   out_4825348341631687231[236] = 0.0;
   out_4825348341631687231[237] = 0.0;
   out_4825348341631687231[238] = 0.0;
   out_4825348341631687231[239] = 0.0;
   out_4825348341631687231[240] = 0.0;
   out_4825348341631687231[241] = 0.0;
   out_4825348341631687231[242] = 0.0;
   out_4825348341631687231[243] = 0.0;
   out_4825348341631687231[244] = 0.0;
   out_4825348341631687231[245] = 0.0;
   out_4825348341631687231[246] = 0.0;
   out_4825348341631687231[247] = 1.0;
   out_4825348341631687231[248] = 0.0;
   out_4825348341631687231[249] = 0.0;
   out_4825348341631687231[250] = 0.0;
   out_4825348341631687231[251] = 0.0;
   out_4825348341631687231[252] = 0.0;
   out_4825348341631687231[253] = 0.0;
   out_4825348341631687231[254] = 0.0;
   out_4825348341631687231[255] = 0.0;
   out_4825348341631687231[256] = 0.0;
   out_4825348341631687231[257] = 0.0;
   out_4825348341631687231[258] = 0.0;
   out_4825348341631687231[259] = 0.0;
   out_4825348341631687231[260] = 0.0;
   out_4825348341631687231[261] = 0.0;
   out_4825348341631687231[262] = 0.0;
   out_4825348341631687231[263] = 0.0;
   out_4825348341631687231[264] = 0.0;
   out_4825348341631687231[265] = 0.0;
   out_4825348341631687231[266] = 1.0;
   out_4825348341631687231[267] = 0.0;
   out_4825348341631687231[268] = 0.0;
   out_4825348341631687231[269] = 0.0;
   out_4825348341631687231[270] = 0.0;
   out_4825348341631687231[271] = 0.0;
   out_4825348341631687231[272] = 0.0;
   out_4825348341631687231[273] = 0.0;
   out_4825348341631687231[274] = 0.0;
   out_4825348341631687231[275] = 0.0;
   out_4825348341631687231[276] = 0.0;
   out_4825348341631687231[277] = 0.0;
   out_4825348341631687231[278] = 0.0;
   out_4825348341631687231[279] = 0.0;
   out_4825348341631687231[280] = 0.0;
   out_4825348341631687231[281] = 0.0;
   out_4825348341631687231[282] = 0.0;
   out_4825348341631687231[283] = 0.0;
   out_4825348341631687231[284] = 0.0;
   out_4825348341631687231[285] = 1.0;
   out_4825348341631687231[286] = 0.0;
   out_4825348341631687231[287] = 0.0;
   out_4825348341631687231[288] = 0.0;
   out_4825348341631687231[289] = 0.0;
   out_4825348341631687231[290] = 0.0;
   out_4825348341631687231[291] = 0.0;
   out_4825348341631687231[292] = 0.0;
   out_4825348341631687231[293] = 0.0;
   out_4825348341631687231[294] = 0.0;
   out_4825348341631687231[295] = 0.0;
   out_4825348341631687231[296] = 0.0;
   out_4825348341631687231[297] = 0.0;
   out_4825348341631687231[298] = 0.0;
   out_4825348341631687231[299] = 0.0;
   out_4825348341631687231[300] = 0.0;
   out_4825348341631687231[301] = 0.0;
   out_4825348341631687231[302] = 0.0;
   out_4825348341631687231[303] = 0.0;
   out_4825348341631687231[304] = 1.0;
   out_4825348341631687231[305] = 0.0;
   out_4825348341631687231[306] = 0.0;
   out_4825348341631687231[307] = 0.0;
   out_4825348341631687231[308] = 0.0;
   out_4825348341631687231[309] = 0.0;
   out_4825348341631687231[310] = 0.0;
   out_4825348341631687231[311] = 0.0;
   out_4825348341631687231[312] = 0.0;
   out_4825348341631687231[313] = 0.0;
   out_4825348341631687231[314] = 0.0;
   out_4825348341631687231[315] = 0.0;
   out_4825348341631687231[316] = 0.0;
   out_4825348341631687231[317] = 0.0;
   out_4825348341631687231[318] = 0.0;
   out_4825348341631687231[319] = 0.0;
   out_4825348341631687231[320] = 0.0;
   out_4825348341631687231[321] = 0.0;
   out_4825348341631687231[322] = 0.0;
   out_4825348341631687231[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_6515554172087681296) {
   out_6515554172087681296[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_6515554172087681296[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_6515554172087681296[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_6515554172087681296[3] = dt*state[12] + state[3];
   out_6515554172087681296[4] = dt*state[13] + state[4];
   out_6515554172087681296[5] = dt*state[14] + state[5];
   out_6515554172087681296[6] = state[6];
   out_6515554172087681296[7] = state[7];
   out_6515554172087681296[8] = state[8];
   out_6515554172087681296[9] = state[9];
   out_6515554172087681296[10] = state[10];
   out_6515554172087681296[11] = state[11];
   out_6515554172087681296[12] = state[12];
   out_6515554172087681296[13] = state[13];
   out_6515554172087681296[14] = state[14];
   out_6515554172087681296[15] = state[15];
   out_6515554172087681296[16] = state[16];
   out_6515554172087681296[17] = state[17];
}
void F_fun(double *state, double dt, double *out_6200830679267345445) {
   out_6200830679267345445[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6200830679267345445[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6200830679267345445[2] = 0;
   out_6200830679267345445[3] = 0;
   out_6200830679267345445[4] = 0;
   out_6200830679267345445[5] = 0;
   out_6200830679267345445[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6200830679267345445[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6200830679267345445[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6200830679267345445[9] = 0;
   out_6200830679267345445[10] = 0;
   out_6200830679267345445[11] = 0;
   out_6200830679267345445[12] = 0;
   out_6200830679267345445[13] = 0;
   out_6200830679267345445[14] = 0;
   out_6200830679267345445[15] = 0;
   out_6200830679267345445[16] = 0;
   out_6200830679267345445[17] = 0;
   out_6200830679267345445[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6200830679267345445[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6200830679267345445[20] = 0;
   out_6200830679267345445[21] = 0;
   out_6200830679267345445[22] = 0;
   out_6200830679267345445[23] = 0;
   out_6200830679267345445[24] = 0;
   out_6200830679267345445[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6200830679267345445[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6200830679267345445[27] = 0;
   out_6200830679267345445[28] = 0;
   out_6200830679267345445[29] = 0;
   out_6200830679267345445[30] = 0;
   out_6200830679267345445[31] = 0;
   out_6200830679267345445[32] = 0;
   out_6200830679267345445[33] = 0;
   out_6200830679267345445[34] = 0;
   out_6200830679267345445[35] = 0;
   out_6200830679267345445[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6200830679267345445[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6200830679267345445[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6200830679267345445[39] = 0;
   out_6200830679267345445[40] = 0;
   out_6200830679267345445[41] = 0;
   out_6200830679267345445[42] = 0;
   out_6200830679267345445[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6200830679267345445[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6200830679267345445[45] = 0;
   out_6200830679267345445[46] = 0;
   out_6200830679267345445[47] = 0;
   out_6200830679267345445[48] = 0;
   out_6200830679267345445[49] = 0;
   out_6200830679267345445[50] = 0;
   out_6200830679267345445[51] = 0;
   out_6200830679267345445[52] = 0;
   out_6200830679267345445[53] = 0;
   out_6200830679267345445[54] = 0;
   out_6200830679267345445[55] = 0;
   out_6200830679267345445[56] = 0;
   out_6200830679267345445[57] = 1;
   out_6200830679267345445[58] = 0;
   out_6200830679267345445[59] = 0;
   out_6200830679267345445[60] = 0;
   out_6200830679267345445[61] = 0;
   out_6200830679267345445[62] = 0;
   out_6200830679267345445[63] = 0;
   out_6200830679267345445[64] = 0;
   out_6200830679267345445[65] = 0;
   out_6200830679267345445[66] = dt;
   out_6200830679267345445[67] = 0;
   out_6200830679267345445[68] = 0;
   out_6200830679267345445[69] = 0;
   out_6200830679267345445[70] = 0;
   out_6200830679267345445[71] = 0;
   out_6200830679267345445[72] = 0;
   out_6200830679267345445[73] = 0;
   out_6200830679267345445[74] = 0;
   out_6200830679267345445[75] = 0;
   out_6200830679267345445[76] = 1;
   out_6200830679267345445[77] = 0;
   out_6200830679267345445[78] = 0;
   out_6200830679267345445[79] = 0;
   out_6200830679267345445[80] = 0;
   out_6200830679267345445[81] = 0;
   out_6200830679267345445[82] = 0;
   out_6200830679267345445[83] = 0;
   out_6200830679267345445[84] = 0;
   out_6200830679267345445[85] = dt;
   out_6200830679267345445[86] = 0;
   out_6200830679267345445[87] = 0;
   out_6200830679267345445[88] = 0;
   out_6200830679267345445[89] = 0;
   out_6200830679267345445[90] = 0;
   out_6200830679267345445[91] = 0;
   out_6200830679267345445[92] = 0;
   out_6200830679267345445[93] = 0;
   out_6200830679267345445[94] = 0;
   out_6200830679267345445[95] = 1;
   out_6200830679267345445[96] = 0;
   out_6200830679267345445[97] = 0;
   out_6200830679267345445[98] = 0;
   out_6200830679267345445[99] = 0;
   out_6200830679267345445[100] = 0;
   out_6200830679267345445[101] = 0;
   out_6200830679267345445[102] = 0;
   out_6200830679267345445[103] = 0;
   out_6200830679267345445[104] = dt;
   out_6200830679267345445[105] = 0;
   out_6200830679267345445[106] = 0;
   out_6200830679267345445[107] = 0;
   out_6200830679267345445[108] = 0;
   out_6200830679267345445[109] = 0;
   out_6200830679267345445[110] = 0;
   out_6200830679267345445[111] = 0;
   out_6200830679267345445[112] = 0;
   out_6200830679267345445[113] = 0;
   out_6200830679267345445[114] = 1;
   out_6200830679267345445[115] = 0;
   out_6200830679267345445[116] = 0;
   out_6200830679267345445[117] = 0;
   out_6200830679267345445[118] = 0;
   out_6200830679267345445[119] = 0;
   out_6200830679267345445[120] = 0;
   out_6200830679267345445[121] = 0;
   out_6200830679267345445[122] = 0;
   out_6200830679267345445[123] = 0;
   out_6200830679267345445[124] = 0;
   out_6200830679267345445[125] = 0;
   out_6200830679267345445[126] = 0;
   out_6200830679267345445[127] = 0;
   out_6200830679267345445[128] = 0;
   out_6200830679267345445[129] = 0;
   out_6200830679267345445[130] = 0;
   out_6200830679267345445[131] = 0;
   out_6200830679267345445[132] = 0;
   out_6200830679267345445[133] = 1;
   out_6200830679267345445[134] = 0;
   out_6200830679267345445[135] = 0;
   out_6200830679267345445[136] = 0;
   out_6200830679267345445[137] = 0;
   out_6200830679267345445[138] = 0;
   out_6200830679267345445[139] = 0;
   out_6200830679267345445[140] = 0;
   out_6200830679267345445[141] = 0;
   out_6200830679267345445[142] = 0;
   out_6200830679267345445[143] = 0;
   out_6200830679267345445[144] = 0;
   out_6200830679267345445[145] = 0;
   out_6200830679267345445[146] = 0;
   out_6200830679267345445[147] = 0;
   out_6200830679267345445[148] = 0;
   out_6200830679267345445[149] = 0;
   out_6200830679267345445[150] = 0;
   out_6200830679267345445[151] = 0;
   out_6200830679267345445[152] = 1;
   out_6200830679267345445[153] = 0;
   out_6200830679267345445[154] = 0;
   out_6200830679267345445[155] = 0;
   out_6200830679267345445[156] = 0;
   out_6200830679267345445[157] = 0;
   out_6200830679267345445[158] = 0;
   out_6200830679267345445[159] = 0;
   out_6200830679267345445[160] = 0;
   out_6200830679267345445[161] = 0;
   out_6200830679267345445[162] = 0;
   out_6200830679267345445[163] = 0;
   out_6200830679267345445[164] = 0;
   out_6200830679267345445[165] = 0;
   out_6200830679267345445[166] = 0;
   out_6200830679267345445[167] = 0;
   out_6200830679267345445[168] = 0;
   out_6200830679267345445[169] = 0;
   out_6200830679267345445[170] = 0;
   out_6200830679267345445[171] = 1;
   out_6200830679267345445[172] = 0;
   out_6200830679267345445[173] = 0;
   out_6200830679267345445[174] = 0;
   out_6200830679267345445[175] = 0;
   out_6200830679267345445[176] = 0;
   out_6200830679267345445[177] = 0;
   out_6200830679267345445[178] = 0;
   out_6200830679267345445[179] = 0;
   out_6200830679267345445[180] = 0;
   out_6200830679267345445[181] = 0;
   out_6200830679267345445[182] = 0;
   out_6200830679267345445[183] = 0;
   out_6200830679267345445[184] = 0;
   out_6200830679267345445[185] = 0;
   out_6200830679267345445[186] = 0;
   out_6200830679267345445[187] = 0;
   out_6200830679267345445[188] = 0;
   out_6200830679267345445[189] = 0;
   out_6200830679267345445[190] = 1;
   out_6200830679267345445[191] = 0;
   out_6200830679267345445[192] = 0;
   out_6200830679267345445[193] = 0;
   out_6200830679267345445[194] = 0;
   out_6200830679267345445[195] = 0;
   out_6200830679267345445[196] = 0;
   out_6200830679267345445[197] = 0;
   out_6200830679267345445[198] = 0;
   out_6200830679267345445[199] = 0;
   out_6200830679267345445[200] = 0;
   out_6200830679267345445[201] = 0;
   out_6200830679267345445[202] = 0;
   out_6200830679267345445[203] = 0;
   out_6200830679267345445[204] = 0;
   out_6200830679267345445[205] = 0;
   out_6200830679267345445[206] = 0;
   out_6200830679267345445[207] = 0;
   out_6200830679267345445[208] = 0;
   out_6200830679267345445[209] = 1;
   out_6200830679267345445[210] = 0;
   out_6200830679267345445[211] = 0;
   out_6200830679267345445[212] = 0;
   out_6200830679267345445[213] = 0;
   out_6200830679267345445[214] = 0;
   out_6200830679267345445[215] = 0;
   out_6200830679267345445[216] = 0;
   out_6200830679267345445[217] = 0;
   out_6200830679267345445[218] = 0;
   out_6200830679267345445[219] = 0;
   out_6200830679267345445[220] = 0;
   out_6200830679267345445[221] = 0;
   out_6200830679267345445[222] = 0;
   out_6200830679267345445[223] = 0;
   out_6200830679267345445[224] = 0;
   out_6200830679267345445[225] = 0;
   out_6200830679267345445[226] = 0;
   out_6200830679267345445[227] = 0;
   out_6200830679267345445[228] = 1;
   out_6200830679267345445[229] = 0;
   out_6200830679267345445[230] = 0;
   out_6200830679267345445[231] = 0;
   out_6200830679267345445[232] = 0;
   out_6200830679267345445[233] = 0;
   out_6200830679267345445[234] = 0;
   out_6200830679267345445[235] = 0;
   out_6200830679267345445[236] = 0;
   out_6200830679267345445[237] = 0;
   out_6200830679267345445[238] = 0;
   out_6200830679267345445[239] = 0;
   out_6200830679267345445[240] = 0;
   out_6200830679267345445[241] = 0;
   out_6200830679267345445[242] = 0;
   out_6200830679267345445[243] = 0;
   out_6200830679267345445[244] = 0;
   out_6200830679267345445[245] = 0;
   out_6200830679267345445[246] = 0;
   out_6200830679267345445[247] = 1;
   out_6200830679267345445[248] = 0;
   out_6200830679267345445[249] = 0;
   out_6200830679267345445[250] = 0;
   out_6200830679267345445[251] = 0;
   out_6200830679267345445[252] = 0;
   out_6200830679267345445[253] = 0;
   out_6200830679267345445[254] = 0;
   out_6200830679267345445[255] = 0;
   out_6200830679267345445[256] = 0;
   out_6200830679267345445[257] = 0;
   out_6200830679267345445[258] = 0;
   out_6200830679267345445[259] = 0;
   out_6200830679267345445[260] = 0;
   out_6200830679267345445[261] = 0;
   out_6200830679267345445[262] = 0;
   out_6200830679267345445[263] = 0;
   out_6200830679267345445[264] = 0;
   out_6200830679267345445[265] = 0;
   out_6200830679267345445[266] = 1;
   out_6200830679267345445[267] = 0;
   out_6200830679267345445[268] = 0;
   out_6200830679267345445[269] = 0;
   out_6200830679267345445[270] = 0;
   out_6200830679267345445[271] = 0;
   out_6200830679267345445[272] = 0;
   out_6200830679267345445[273] = 0;
   out_6200830679267345445[274] = 0;
   out_6200830679267345445[275] = 0;
   out_6200830679267345445[276] = 0;
   out_6200830679267345445[277] = 0;
   out_6200830679267345445[278] = 0;
   out_6200830679267345445[279] = 0;
   out_6200830679267345445[280] = 0;
   out_6200830679267345445[281] = 0;
   out_6200830679267345445[282] = 0;
   out_6200830679267345445[283] = 0;
   out_6200830679267345445[284] = 0;
   out_6200830679267345445[285] = 1;
   out_6200830679267345445[286] = 0;
   out_6200830679267345445[287] = 0;
   out_6200830679267345445[288] = 0;
   out_6200830679267345445[289] = 0;
   out_6200830679267345445[290] = 0;
   out_6200830679267345445[291] = 0;
   out_6200830679267345445[292] = 0;
   out_6200830679267345445[293] = 0;
   out_6200830679267345445[294] = 0;
   out_6200830679267345445[295] = 0;
   out_6200830679267345445[296] = 0;
   out_6200830679267345445[297] = 0;
   out_6200830679267345445[298] = 0;
   out_6200830679267345445[299] = 0;
   out_6200830679267345445[300] = 0;
   out_6200830679267345445[301] = 0;
   out_6200830679267345445[302] = 0;
   out_6200830679267345445[303] = 0;
   out_6200830679267345445[304] = 1;
   out_6200830679267345445[305] = 0;
   out_6200830679267345445[306] = 0;
   out_6200830679267345445[307] = 0;
   out_6200830679267345445[308] = 0;
   out_6200830679267345445[309] = 0;
   out_6200830679267345445[310] = 0;
   out_6200830679267345445[311] = 0;
   out_6200830679267345445[312] = 0;
   out_6200830679267345445[313] = 0;
   out_6200830679267345445[314] = 0;
   out_6200830679267345445[315] = 0;
   out_6200830679267345445[316] = 0;
   out_6200830679267345445[317] = 0;
   out_6200830679267345445[318] = 0;
   out_6200830679267345445[319] = 0;
   out_6200830679267345445[320] = 0;
   out_6200830679267345445[321] = 0;
   out_6200830679267345445[322] = 0;
   out_6200830679267345445[323] = 1;
}
void h_4(double *state, double *unused, double *out_2281311047032518481) {
   out_2281311047032518481[0] = state[6] + state[9];
   out_2281311047032518481[1] = state[7] + state[10];
   out_2281311047032518481[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_1022733465120271490) {
   out_1022733465120271490[0] = 0;
   out_1022733465120271490[1] = 0;
   out_1022733465120271490[2] = 0;
   out_1022733465120271490[3] = 0;
   out_1022733465120271490[4] = 0;
   out_1022733465120271490[5] = 0;
   out_1022733465120271490[6] = 1;
   out_1022733465120271490[7] = 0;
   out_1022733465120271490[8] = 0;
   out_1022733465120271490[9] = 1;
   out_1022733465120271490[10] = 0;
   out_1022733465120271490[11] = 0;
   out_1022733465120271490[12] = 0;
   out_1022733465120271490[13] = 0;
   out_1022733465120271490[14] = 0;
   out_1022733465120271490[15] = 0;
   out_1022733465120271490[16] = 0;
   out_1022733465120271490[17] = 0;
   out_1022733465120271490[18] = 0;
   out_1022733465120271490[19] = 0;
   out_1022733465120271490[20] = 0;
   out_1022733465120271490[21] = 0;
   out_1022733465120271490[22] = 0;
   out_1022733465120271490[23] = 0;
   out_1022733465120271490[24] = 0;
   out_1022733465120271490[25] = 1;
   out_1022733465120271490[26] = 0;
   out_1022733465120271490[27] = 0;
   out_1022733465120271490[28] = 1;
   out_1022733465120271490[29] = 0;
   out_1022733465120271490[30] = 0;
   out_1022733465120271490[31] = 0;
   out_1022733465120271490[32] = 0;
   out_1022733465120271490[33] = 0;
   out_1022733465120271490[34] = 0;
   out_1022733465120271490[35] = 0;
   out_1022733465120271490[36] = 0;
   out_1022733465120271490[37] = 0;
   out_1022733465120271490[38] = 0;
   out_1022733465120271490[39] = 0;
   out_1022733465120271490[40] = 0;
   out_1022733465120271490[41] = 0;
   out_1022733465120271490[42] = 0;
   out_1022733465120271490[43] = 0;
   out_1022733465120271490[44] = 1;
   out_1022733465120271490[45] = 0;
   out_1022733465120271490[46] = 0;
   out_1022733465120271490[47] = 1;
   out_1022733465120271490[48] = 0;
   out_1022733465120271490[49] = 0;
   out_1022733465120271490[50] = 0;
   out_1022733465120271490[51] = 0;
   out_1022733465120271490[52] = 0;
   out_1022733465120271490[53] = 0;
}
void h_10(double *state, double *unused, double *out_7298821485634386065) {
   out_7298821485634386065[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_7298821485634386065[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_7298821485634386065[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_4580916266697147042) {
   out_4580916266697147042[0] = 0;
   out_4580916266697147042[1] = 9.8100000000000005*cos(state[1]);
   out_4580916266697147042[2] = 0;
   out_4580916266697147042[3] = 0;
   out_4580916266697147042[4] = -state[8];
   out_4580916266697147042[5] = state[7];
   out_4580916266697147042[6] = 0;
   out_4580916266697147042[7] = state[5];
   out_4580916266697147042[8] = -state[4];
   out_4580916266697147042[9] = 0;
   out_4580916266697147042[10] = 0;
   out_4580916266697147042[11] = 0;
   out_4580916266697147042[12] = 1;
   out_4580916266697147042[13] = 0;
   out_4580916266697147042[14] = 0;
   out_4580916266697147042[15] = 1;
   out_4580916266697147042[16] = 0;
   out_4580916266697147042[17] = 0;
   out_4580916266697147042[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_4580916266697147042[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_4580916266697147042[20] = 0;
   out_4580916266697147042[21] = state[8];
   out_4580916266697147042[22] = 0;
   out_4580916266697147042[23] = -state[6];
   out_4580916266697147042[24] = -state[5];
   out_4580916266697147042[25] = 0;
   out_4580916266697147042[26] = state[3];
   out_4580916266697147042[27] = 0;
   out_4580916266697147042[28] = 0;
   out_4580916266697147042[29] = 0;
   out_4580916266697147042[30] = 0;
   out_4580916266697147042[31] = 1;
   out_4580916266697147042[32] = 0;
   out_4580916266697147042[33] = 0;
   out_4580916266697147042[34] = 1;
   out_4580916266697147042[35] = 0;
   out_4580916266697147042[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_4580916266697147042[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_4580916266697147042[38] = 0;
   out_4580916266697147042[39] = -state[7];
   out_4580916266697147042[40] = state[6];
   out_4580916266697147042[41] = 0;
   out_4580916266697147042[42] = state[4];
   out_4580916266697147042[43] = -state[3];
   out_4580916266697147042[44] = 0;
   out_4580916266697147042[45] = 0;
   out_4580916266697147042[46] = 0;
   out_4580916266697147042[47] = 0;
   out_4580916266697147042[48] = 0;
   out_4580916266697147042[49] = 0;
   out_4580916266697147042[50] = 1;
   out_4580916266697147042[51] = 0;
   out_4580916266697147042[52] = 0;
   out_4580916266697147042[53] = 1;
}
void h_13(double *state, double *unused, double *out_1884508169225221663) {
   out_1884508169225221663[0] = state[3];
   out_1884508169225221663[1] = state[4];
   out_1884508169225221663[2] = state[5];
}
void H_13(double *state, double *unused, double *out_2189540360212061311) {
   out_2189540360212061311[0] = 0;
   out_2189540360212061311[1] = 0;
   out_2189540360212061311[2] = 0;
   out_2189540360212061311[3] = 1;
   out_2189540360212061311[4] = 0;
   out_2189540360212061311[5] = 0;
   out_2189540360212061311[6] = 0;
   out_2189540360212061311[7] = 0;
   out_2189540360212061311[8] = 0;
   out_2189540360212061311[9] = 0;
   out_2189540360212061311[10] = 0;
   out_2189540360212061311[11] = 0;
   out_2189540360212061311[12] = 0;
   out_2189540360212061311[13] = 0;
   out_2189540360212061311[14] = 0;
   out_2189540360212061311[15] = 0;
   out_2189540360212061311[16] = 0;
   out_2189540360212061311[17] = 0;
   out_2189540360212061311[18] = 0;
   out_2189540360212061311[19] = 0;
   out_2189540360212061311[20] = 0;
   out_2189540360212061311[21] = 0;
   out_2189540360212061311[22] = 1;
   out_2189540360212061311[23] = 0;
   out_2189540360212061311[24] = 0;
   out_2189540360212061311[25] = 0;
   out_2189540360212061311[26] = 0;
   out_2189540360212061311[27] = 0;
   out_2189540360212061311[28] = 0;
   out_2189540360212061311[29] = 0;
   out_2189540360212061311[30] = 0;
   out_2189540360212061311[31] = 0;
   out_2189540360212061311[32] = 0;
   out_2189540360212061311[33] = 0;
   out_2189540360212061311[34] = 0;
   out_2189540360212061311[35] = 0;
   out_2189540360212061311[36] = 0;
   out_2189540360212061311[37] = 0;
   out_2189540360212061311[38] = 0;
   out_2189540360212061311[39] = 0;
   out_2189540360212061311[40] = 0;
   out_2189540360212061311[41] = 1;
   out_2189540360212061311[42] = 0;
   out_2189540360212061311[43] = 0;
   out_2189540360212061311[44] = 0;
   out_2189540360212061311[45] = 0;
   out_2189540360212061311[46] = 0;
   out_2189540360212061311[47] = 0;
   out_2189540360212061311[48] = 0;
   out_2189540360212061311[49] = 0;
   out_2189540360212061311[50] = 0;
   out_2189540360212061311[51] = 0;
   out_2189540360212061311[52] = 0;
   out_2189540360212061311[53] = 0;
}
void h_14(double *state, double *unused, double *out_1873182816391455999) {
   out_1873182816391455999[0] = state[6];
   out_1873182816391455999[1] = state[7];
   out_1873182816391455999[2] = state[8];
}
void H_14(double *state, double *unused, double *out_2940507391219213039) {
   out_2940507391219213039[0] = 0;
   out_2940507391219213039[1] = 0;
   out_2940507391219213039[2] = 0;
   out_2940507391219213039[3] = 0;
   out_2940507391219213039[4] = 0;
   out_2940507391219213039[5] = 0;
   out_2940507391219213039[6] = 1;
   out_2940507391219213039[7] = 0;
   out_2940507391219213039[8] = 0;
   out_2940507391219213039[9] = 0;
   out_2940507391219213039[10] = 0;
   out_2940507391219213039[11] = 0;
   out_2940507391219213039[12] = 0;
   out_2940507391219213039[13] = 0;
   out_2940507391219213039[14] = 0;
   out_2940507391219213039[15] = 0;
   out_2940507391219213039[16] = 0;
   out_2940507391219213039[17] = 0;
   out_2940507391219213039[18] = 0;
   out_2940507391219213039[19] = 0;
   out_2940507391219213039[20] = 0;
   out_2940507391219213039[21] = 0;
   out_2940507391219213039[22] = 0;
   out_2940507391219213039[23] = 0;
   out_2940507391219213039[24] = 0;
   out_2940507391219213039[25] = 1;
   out_2940507391219213039[26] = 0;
   out_2940507391219213039[27] = 0;
   out_2940507391219213039[28] = 0;
   out_2940507391219213039[29] = 0;
   out_2940507391219213039[30] = 0;
   out_2940507391219213039[31] = 0;
   out_2940507391219213039[32] = 0;
   out_2940507391219213039[33] = 0;
   out_2940507391219213039[34] = 0;
   out_2940507391219213039[35] = 0;
   out_2940507391219213039[36] = 0;
   out_2940507391219213039[37] = 0;
   out_2940507391219213039[38] = 0;
   out_2940507391219213039[39] = 0;
   out_2940507391219213039[40] = 0;
   out_2940507391219213039[41] = 0;
   out_2940507391219213039[42] = 0;
   out_2940507391219213039[43] = 0;
   out_2940507391219213039[44] = 1;
   out_2940507391219213039[45] = 0;
   out_2940507391219213039[46] = 0;
   out_2940507391219213039[47] = 0;
   out_2940507391219213039[48] = 0;
   out_2940507391219213039[49] = 0;
   out_2940507391219213039[50] = 0;
   out_2940507391219213039[51] = 0;
   out_2940507391219213039[52] = 0;
   out_2940507391219213039[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_3037457306540495367) {
  err_fun(nom_x, delta_x, out_3037457306540495367);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5504742843249496146) {
  inv_err_fun(nom_x, true_x, out_5504742843249496146);
}
void pose_H_mod_fun(double *state, double *out_4825348341631687231) {
  H_mod_fun(state, out_4825348341631687231);
}
void pose_f_fun(double *state, double dt, double *out_6515554172087681296) {
  f_fun(state,  dt, out_6515554172087681296);
}
void pose_F_fun(double *state, double dt, double *out_6200830679267345445) {
  F_fun(state,  dt, out_6200830679267345445);
}
void pose_h_4(double *state, double *unused, double *out_2281311047032518481) {
  h_4(state, unused, out_2281311047032518481);
}
void pose_H_4(double *state, double *unused, double *out_1022733465120271490) {
  H_4(state, unused, out_1022733465120271490);
}
void pose_h_10(double *state, double *unused, double *out_7298821485634386065) {
  h_10(state, unused, out_7298821485634386065);
}
void pose_H_10(double *state, double *unused, double *out_4580916266697147042) {
  H_10(state, unused, out_4580916266697147042);
}
void pose_h_13(double *state, double *unused, double *out_1884508169225221663) {
  h_13(state, unused, out_1884508169225221663);
}
void pose_H_13(double *state, double *unused, double *out_2189540360212061311) {
  H_13(state, unused, out_2189540360212061311);
}
void pose_h_14(double *state, double *unused, double *out_1873182816391455999) {
  h_14(state, unused, out_1873182816391455999);
}
void pose_H_14(double *state, double *unused, double *out_2940507391219213039) {
  H_14(state, unused, out_2940507391219213039);
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

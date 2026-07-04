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
void err_fun(double *nom_x, double *delta_x, double *out_8560828130109272763) {
   out_8560828130109272763[0] = delta_x[0] + nom_x[0];
   out_8560828130109272763[1] = delta_x[1] + nom_x[1];
   out_8560828130109272763[2] = delta_x[2] + nom_x[2];
   out_8560828130109272763[3] = delta_x[3] + nom_x[3];
   out_8560828130109272763[4] = delta_x[4] + nom_x[4];
   out_8560828130109272763[5] = delta_x[5] + nom_x[5];
   out_8560828130109272763[6] = delta_x[6] + nom_x[6];
   out_8560828130109272763[7] = delta_x[7] + nom_x[7];
   out_8560828130109272763[8] = delta_x[8] + nom_x[8];
   out_8560828130109272763[9] = delta_x[9] + nom_x[9];
   out_8560828130109272763[10] = delta_x[10] + nom_x[10];
   out_8560828130109272763[11] = delta_x[11] + nom_x[11];
   out_8560828130109272763[12] = delta_x[12] + nom_x[12];
   out_8560828130109272763[13] = delta_x[13] + nom_x[13];
   out_8560828130109272763[14] = delta_x[14] + nom_x[14];
   out_8560828130109272763[15] = delta_x[15] + nom_x[15];
   out_8560828130109272763[16] = delta_x[16] + nom_x[16];
   out_8560828130109272763[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_157920368283105894) {
   out_157920368283105894[0] = -nom_x[0] + true_x[0];
   out_157920368283105894[1] = -nom_x[1] + true_x[1];
   out_157920368283105894[2] = -nom_x[2] + true_x[2];
   out_157920368283105894[3] = -nom_x[3] + true_x[3];
   out_157920368283105894[4] = -nom_x[4] + true_x[4];
   out_157920368283105894[5] = -nom_x[5] + true_x[5];
   out_157920368283105894[6] = -nom_x[6] + true_x[6];
   out_157920368283105894[7] = -nom_x[7] + true_x[7];
   out_157920368283105894[8] = -nom_x[8] + true_x[8];
   out_157920368283105894[9] = -nom_x[9] + true_x[9];
   out_157920368283105894[10] = -nom_x[10] + true_x[10];
   out_157920368283105894[11] = -nom_x[11] + true_x[11];
   out_157920368283105894[12] = -nom_x[12] + true_x[12];
   out_157920368283105894[13] = -nom_x[13] + true_x[13];
   out_157920368283105894[14] = -nom_x[14] + true_x[14];
   out_157920368283105894[15] = -nom_x[15] + true_x[15];
   out_157920368283105894[16] = -nom_x[16] + true_x[16];
   out_157920368283105894[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_3313916881971386680) {
   out_3313916881971386680[0] = 1.0;
   out_3313916881971386680[1] = 0.0;
   out_3313916881971386680[2] = 0.0;
   out_3313916881971386680[3] = 0.0;
   out_3313916881971386680[4] = 0.0;
   out_3313916881971386680[5] = 0.0;
   out_3313916881971386680[6] = 0.0;
   out_3313916881971386680[7] = 0.0;
   out_3313916881971386680[8] = 0.0;
   out_3313916881971386680[9] = 0.0;
   out_3313916881971386680[10] = 0.0;
   out_3313916881971386680[11] = 0.0;
   out_3313916881971386680[12] = 0.0;
   out_3313916881971386680[13] = 0.0;
   out_3313916881971386680[14] = 0.0;
   out_3313916881971386680[15] = 0.0;
   out_3313916881971386680[16] = 0.0;
   out_3313916881971386680[17] = 0.0;
   out_3313916881971386680[18] = 0.0;
   out_3313916881971386680[19] = 1.0;
   out_3313916881971386680[20] = 0.0;
   out_3313916881971386680[21] = 0.0;
   out_3313916881971386680[22] = 0.0;
   out_3313916881971386680[23] = 0.0;
   out_3313916881971386680[24] = 0.0;
   out_3313916881971386680[25] = 0.0;
   out_3313916881971386680[26] = 0.0;
   out_3313916881971386680[27] = 0.0;
   out_3313916881971386680[28] = 0.0;
   out_3313916881971386680[29] = 0.0;
   out_3313916881971386680[30] = 0.0;
   out_3313916881971386680[31] = 0.0;
   out_3313916881971386680[32] = 0.0;
   out_3313916881971386680[33] = 0.0;
   out_3313916881971386680[34] = 0.0;
   out_3313916881971386680[35] = 0.0;
   out_3313916881971386680[36] = 0.0;
   out_3313916881971386680[37] = 0.0;
   out_3313916881971386680[38] = 1.0;
   out_3313916881971386680[39] = 0.0;
   out_3313916881971386680[40] = 0.0;
   out_3313916881971386680[41] = 0.0;
   out_3313916881971386680[42] = 0.0;
   out_3313916881971386680[43] = 0.0;
   out_3313916881971386680[44] = 0.0;
   out_3313916881971386680[45] = 0.0;
   out_3313916881971386680[46] = 0.0;
   out_3313916881971386680[47] = 0.0;
   out_3313916881971386680[48] = 0.0;
   out_3313916881971386680[49] = 0.0;
   out_3313916881971386680[50] = 0.0;
   out_3313916881971386680[51] = 0.0;
   out_3313916881971386680[52] = 0.0;
   out_3313916881971386680[53] = 0.0;
   out_3313916881971386680[54] = 0.0;
   out_3313916881971386680[55] = 0.0;
   out_3313916881971386680[56] = 0.0;
   out_3313916881971386680[57] = 1.0;
   out_3313916881971386680[58] = 0.0;
   out_3313916881971386680[59] = 0.0;
   out_3313916881971386680[60] = 0.0;
   out_3313916881971386680[61] = 0.0;
   out_3313916881971386680[62] = 0.0;
   out_3313916881971386680[63] = 0.0;
   out_3313916881971386680[64] = 0.0;
   out_3313916881971386680[65] = 0.0;
   out_3313916881971386680[66] = 0.0;
   out_3313916881971386680[67] = 0.0;
   out_3313916881971386680[68] = 0.0;
   out_3313916881971386680[69] = 0.0;
   out_3313916881971386680[70] = 0.0;
   out_3313916881971386680[71] = 0.0;
   out_3313916881971386680[72] = 0.0;
   out_3313916881971386680[73] = 0.0;
   out_3313916881971386680[74] = 0.0;
   out_3313916881971386680[75] = 0.0;
   out_3313916881971386680[76] = 1.0;
   out_3313916881971386680[77] = 0.0;
   out_3313916881971386680[78] = 0.0;
   out_3313916881971386680[79] = 0.0;
   out_3313916881971386680[80] = 0.0;
   out_3313916881971386680[81] = 0.0;
   out_3313916881971386680[82] = 0.0;
   out_3313916881971386680[83] = 0.0;
   out_3313916881971386680[84] = 0.0;
   out_3313916881971386680[85] = 0.0;
   out_3313916881971386680[86] = 0.0;
   out_3313916881971386680[87] = 0.0;
   out_3313916881971386680[88] = 0.0;
   out_3313916881971386680[89] = 0.0;
   out_3313916881971386680[90] = 0.0;
   out_3313916881971386680[91] = 0.0;
   out_3313916881971386680[92] = 0.0;
   out_3313916881971386680[93] = 0.0;
   out_3313916881971386680[94] = 0.0;
   out_3313916881971386680[95] = 1.0;
   out_3313916881971386680[96] = 0.0;
   out_3313916881971386680[97] = 0.0;
   out_3313916881971386680[98] = 0.0;
   out_3313916881971386680[99] = 0.0;
   out_3313916881971386680[100] = 0.0;
   out_3313916881971386680[101] = 0.0;
   out_3313916881971386680[102] = 0.0;
   out_3313916881971386680[103] = 0.0;
   out_3313916881971386680[104] = 0.0;
   out_3313916881971386680[105] = 0.0;
   out_3313916881971386680[106] = 0.0;
   out_3313916881971386680[107] = 0.0;
   out_3313916881971386680[108] = 0.0;
   out_3313916881971386680[109] = 0.0;
   out_3313916881971386680[110] = 0.0;
   out_3313916881971386680[111] = 0.0;
   out_3313916881971386680[112] = 0.0;
   out_3313916881971386680[113] = 0.0;
   out_3313916881971386680[114] = 1.0;
   out_3313916881971386680[115] = 0.0;
   out_3313916881971386680[116] = 0.0;
   out_3313916881971386680[117] = 0.0;
   out_3313916881971386680[118] = 0.0;
   out_3313916881971386680[119] = 0.0;
   out_3313916881971386680[120] = 0.0;
   out_3313916881971386680[121] = 0.0;
   out_3313916881971386680[122] = 0.0;
   out_3313916881971386680[123] = 0.0;
   out_3313916881971386680[124] = 0.0;
   out_3313916881971386680[125] = 0.0;
   out_3313916881971386680[126] = 0.0;
   out_3313916881971386680[127] = 0.0;
   out_3313916881971386680[128] = 0.0;
   out_3313916881971386680[129] = 0.0;
   out_3313916881971386680[130] = 0.0;
   out_3313916881971386680[131] = 0.0;
   out_3313916881971386680[132] = 0.0;
   out_3313916881971386680[133] = 1.0;
   out_3313916881971386680[134] = 0.0;
   out_3313916881971386680[135] = 0.0;
   out_3313916881971386680[136] = 0.0;
   out_3313916881971386680[137] = 0.0;
   out_3313916881971386680[138] = 0.0;
   out_3313916881971386680[139] = 0.0;
   out_3313916881971386680[140] = 0.0;
   out_3313916881971386680[141] = 0.0;
   out_3313916881971386680[142] = 0.0;
   out_3313916881971386680[143] = 0.0;
   out_3313916881971386680[144] = 0.0;
   out_3313916881971386680[145] = 0.0;
   out_3313916881971386680[146] = 0.0;
   out_3313916881971386680[147] = 0.0;
   out_3313916881971386680[148] = 0.0;
   out_3313916881971386680[149] = 0.0;
   out_3313916881971386680[150] = 0.0;
   out_3313916881971386680[151] = 0.0;
   out_3313916881971386680[152] = 1.0;
   out_3313916881971386680[153] = 0.0;
   out_3313916881971386680[154] = 0.0;
   out_3313916881971386680[155] = 0.0;
   out_3313916881971386680[156] = 0.0;
   out_3313916881971386680[157] = 0.0;
   out_3313916881971386680[158] = 0.0;
   out_3313916881971386680[159] = 0.0;
   out_3313916881971386680[160] = 0.0;
   out_3313916881971386680[161] = 0.0;
   out_3313916881971386680[162] = 0.0;
   out_3313916881971386680[163] = 0.0;
   out_3313916881971386680[164] = 0.0;
   out_3313916881971386680[165] = 0.0;
   out_3313916881971386680[166] = 0.0;
   out_3313916881971386680[167] = 0.0;
   out_3313916881971386680[168] = 0.0;
   out_3313916881971386680[169] = 0.0;
   out_3313916881971386680[170] = 0.0;
   out_3313916881971386680[171] = 1.0;
   out_3313916881971386680[172] = 0.0;
   out_3313916881971386680[173] = 0.0;
   out_3313916881971386680[174] = 0.0;
   out_3313916881971386680[175] = 0.0;
   out_3313916881971386680[176] = 0.0;
   out_3313916881971386680[177] = 0.0;
   out_3313916881971386680[178] = 0.0;
   out_3313916881971386680[179] = 0.0;
   out_3313916881971386680[180] = 0.0;
   out_3313916881971386680[181] = 0.0;
   out_3313916881971386680[182] = 0.0;
   out_3313916881971386680[183] = 0.0;
   out_3313916881971386680[184] = 0.0;
   out_3313916881971386680[185] = 0.0;
   out_3313916881971386680[186] = 0.0;
   out_3313916881971386680[187] = 0.0;
   out_3313916881971386680[188] = 0.0;
   out_3313916881971386680[189] = 0.0;
   out_3313916881971386680[190] = 1.0;
   out_3313916881971386680[191] = 0.0;
   out_3313916881971386680[192] = 0.0;
   out_3313916881971386680[193] = 0.0;
   out_3313916881971386680[194] = 0.0;
   out_3313916881971386680[195] = 0.0;
   out_3313916881971386680[196] = 0.0;
   out_3313916881971386680[197] = 0.0;
   out_3313916881971386680[198] = 0.0;
   out_3313916881971386680[199] = 0.0;
   out_3313916881971386680[200] = 0.0;
   out_3313916881971386680[201] = 0.0;
   out_3313916881971386680[202] = 0.0;
   out_3313916881971386680[203] = 0.0;
   out_3313916881971386680[204] = 0.0;
   out_3313916881971386680[205] = 0.0;
   out_3313916881971386680[206] = 0.0;
   out_3313916881971386680[207] = 0.0;
   out_3313916881971386680[208] = 0.0;
   out_3313916881971386680[209] = 1.0;
   out_3313916881971386680[210] = 0.0;
   out_3313916881971386680[211] = 0.0;
   out_3313916881971386680[212] = 0.0;
   out_3313916881971386680[213] = 0.0;
   out_3313916881971386680[214] = 0.0;
   out_3313916881971386680[215] = 0.0;
   out_3313916881971386680[216] = 0.0;
   out_3313916881971386680[217] = 0.0;
   out_3313916881971386680[218] = 0.0;
   out_3313916881971386680[219] = 0.0;
   out_3313916881971386680[220] = 0.0;
   out_3313916881971386680[221] = 0.0;
   out_3313916881971386680[222] = 0.0;
   out_3313916881971386680[223] = 0.0;
   out_3313916881971386680[224] = 0.0;
   out_3313916881971386680[225] = 0.0;
   out_3313916881971386680[226] = 0.0;
   out_3313916881971386680[227] = 0.0;
   out_3313916881971386680[228] = 1.0;
   out_3313916881971386680[229] = 0.0;
   out_3313916881971386680[230] = 0.0;
   out_3313916881971386680[231] = 0.0;
   out_3313916881971386680[232] = 0.0;
   out_3313916881971386680[233] = 0.0;
   out_3313916881971386680[234] = 0.0;
   out_3313916881971386680[235] = 0.0;
   out_3313916881971386680[236] = 0.0;
   out_3313916881971386680[237] = 0.0;
   out_3313916881971386680[238] = 0.0;
   out_3313916881971386680[239] = 0.0;
   out_3313916881971386680[240] = 0.0;
   out_3313916881971386680[241] = 0.0;
   out_3313916881971386680[242] = 0.0;
   out_3313916881971386680[243] = 0.0;
   out_3313916881971386680[244] = 0.0;
   out_3313916881971386680[245] = 0.0;
   out_3313916881971386680[246] = 0.0;
   out_3313916881971386680[247] = 1.0;
   out_3313916881971386680[248] = 0.0;
   out_3313916881971386680[249] = 0.0;
   out_3313916881971386680[250] = 0.0;
   out_3313916881971386680[251] = 0.0;
   out_3313916881971386680[252] = 0.0;
   out_3313916881971386680[253] = 0.0;
   out_3313916881971386680[254] = 0.0;
   out_3313916881971386680[255] = 0.0;
   out_3313916881971386680[256] = 0.0;
   out_3313916881971386680[257] = 0.0;
   out_3313916881971386680[258] = 0.0;
   out_3313916881971386680[259] = 0.0;
   out_3313916881971386680[260] = 0.0;
   out_3313916881971386680[261] = 0.0;
   out_3313916881971386680[262] = 0.0;
   out_3313916881971386680[263] = 0.0;
   out_3313916881971386680[264] = 0.0;
   out_3313916881971386680[265] = 0.0;
   out_3313916881971386680[266] = 1.0;
   out_3313916881971386680[267] = 0.0;
   out_3313916881971386680[268] = 0.0;
   out_3313916881971386680[269] = 0.0;
   out_3313916881971386680[270] = 0.0;
   out_3313916881971386680[271] = 0.0;
   out_3313916881971386680[272] = 0.0;
   out_3313916881971386680[273] = 0.0;
   out_3313916881971386680[274] = 0.0;
   out_3313916881971386680[275] = 0.0;
   out_3313916881971386680[276] = 0.0;
   out_3313916881971386680[277] = 0.0;
   out_3313916881971386680[278] = 0.0;
   out_3313916881971386680[279] = 0.0;
   out_3313916881971386680[280] = 0.0;
   out_3313916881971386680[281] = 0.0;
   out_3313916881971386680[282] = 0.0;
   out_3313916881971386680[283] = 0.0;
   out_3313916881971386680[284] = 0.0;
   out_3313916881971386680[285] = 1.0;
   out_3313916881971386680[286] = 0.0;
   out_3313916881971386680[287] = 0.0;
   out_3313916881971386680[288] = 0.0;
   out_3313916881971386680[289] = 0.0;
   out_3313916881971386680[290] = 0.0;
   out_3313916881971386680[291] = 0.0;
   out_3313916881971386680[292] = 0.0;
   out_3313916881971386680[293] = 0.0;
   out_3313916881971386680[294] = 0.0;
   out_3313916881971386680[295] = 0.0;
   out_3313916881971386680[296] = 0.0;
   out_3313916881971386680[297] = 0.0;
   out_3313916881971386680[298] = 0.0;
   out_3313916881971386680[299] = 0.0;
   out_3313916881971386680[300] = 0.0;
   out_3313916881971386680[301] = 0.0;
   out_3313916881971386680[302] = 0.0;
   out_3313916881971386680[303] = 0.0;
   out_3313916881971386680[304] = 1.0;
   out_3313916881971386680[305] = 0.0;
   out_3313916881971386680[306] = 0.0;
   out_3313916881971386680[307] = 0.0;
   out_3313916881971386680[308] = 0.0;
   out_3313916881971386680[309] = 0.0;
   out_3313916881971386680[310] = 0.0;
   out_3313916881971386680[311] = 0.0;
   out_3313916881971386680[312] = 0.0;
   out_3313916881971386680[313] = 0.0;
   out_3313916881971386680[314] = 0.0;
   out_3313916881971386680[315] = 0.0;
   out_3313916881971386680[316] = 0.0;
   out_3313916881971386680[317] = 0.0;
   out_3313916881971386680[318] = 0.0;
   out_3313916881971386680[319] = 0.0;
   out_3313916881971386680[320] = 0.0;
   out_3313916881971386680[321] = 0.0;
   out_3313916881971386680[322] = 0.0;
   out_3313916881971386680[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_5589389168188525463) {
   out_5589389168188525463[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_5589389168188525463[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_5589389168188525463[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_5589389168188525463[3] = dt*state[12] + state[3];
   out_5589389168188525463[4] = dt*state[13] + state[4];
   out_5589389168188525463[5] = dt*state[14] + state[5];
   out_5589389168188525463[6] = state[6];
   out_5589389168188525463[7] = state[7];
   out_5589389168188525463[8] = state[8];
   out_5589389168188525463[9] = state[9];
   out_5589389168188525463[10] = state[10];
   out_5589389168188525463[11] = state[11];
   out_5589389168188525463[12] = state[12];
   out_5589389168188525463[13] = state[13];
   out_5589389168188525463[14] = state[14];
   out_5589389168188525463[15] = state[15];
   out_5589389168188525463[16] = state[16];
   out_5589389168188525463[17] = state[17];
}
void F_fun(double *state, double dt, double *out_5032970381228147467) {
   out_5032970381228147467[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5032970381228147467[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5032970381228147467[2] = 0;
   out_5032970381228147467[3] = 0;
   out_5032970381228147467[4] = 0;
   out_5032970381228147467[5] = 0;
   out_5032970381228147467[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5032970381228147467[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5032970381228147467[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5032970381228147467[9] = 0;
   out_5032970381228147467[10] = 0;
   out_5032970381228147467[11] = 0;
   out_5032970381228147467[12] = 0;
   out_5032970381228147467[13] = 0;
   out_5032970381228147467[14] = 0;
   out_5032970381228147467[15] = 0;
   out_5032970381228147467[16] = 0;
   out_5032970381228147467[17] = 0;
   out_5032970381228147467[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5032970381228147467[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5032970381228147467[20] = 0;
   out_5032970381228147467[21] = 0;
   out_5032970381228147467[22] = 0;
   out_5032970381228147467[23] = 0;
   out_5032970381228147467[24] = 0;
   out_5032970381228147467[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5032970381228147467[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5032970381228147467[27] = 0;
   out_5032970381228147467[28] = 0;
   out_5032970381228147467[29] = 0;
   out_5032970381228147467[30] = 0;
   out_5032970381228147467[31] = 0;
   out_5032970381228147467[32] = 0;
   out_5032970381228147467[33] = 0;
   out_5032970381228147467[34] = 0;
   out_5032970381228147467[35] = 0;
   out_5032970381228147467[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5032970381228147467[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5032970381228147467[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5032970381228147467[39] = 0;
   out_5032970381228147467[40] = 0;
   out_5032970381228147467[41] = 0;
   out_5032970381228147467[42] = 0;
   out_5032970381228147467[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5032970381228147467[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5032970381228147467[45] = 0;
   out_5032970381228147467[46] = 0;
   out_5032970381228147467[47] = 0;
   out_5032970381228147467[48] = 0;
   out_5032970381228147467[49] = 0;
   out_5032970381228147467[50] = 0;
   out_5032970381228147467[51] = 0;
   out_5032970381228147467[52] = 0;
   out_5032970381228147467[53] = 0;
   out_5032970381228147467[54] = 0;
   out_5032970381228147467[55] = 0;
   out_5032970381228147467[56] = 0;
   out_5032970381228147467[57] = 1;
   out_5032970381228147467[58] = 0;
   out_5032970381228147467[59] = 0;
   out_5032970381228147467[60] = 0;
   out_5032970381228147467[61] = 0;
   out_5032970381228147467[62] = 0;
   out_5032970381228147467[63] = 0;
   out_5032970381228147467[64] = 0;
   out_5032970381228147467[65] = 0;
   out_5032970381228147467[66] = dt;
   out_5032970381228147467[67] = 0;
   out_5032970381228147467[68] = 0;
   out_5032970381228147467[69] = 0;
   out_5032970381228147467[70] = 0;
   out_5032970381228147467[71] = 0;
   out_5032970381228147467[72] = 0;
   out_5032970381228147467[73] = 0;
   out_5032970381228147467[74] = 0;
   out_5032970381228147467[75] = 0;
   out_5032970381228147467[76] = 1;
   out_5032970381228147467[77] = 0;
   out_5032970381228147467[78] = 0;
   out_5032970381228147467[79] = 0;
   out_5032970381228147467[80] = 0;
   out_5032970381228147467[81] = 0;
   out_5032970381228147467[82] = 0;
   out_5032970381228147467[83] = 0;
   out_5032970381228147467[84] = 0;
   out_5032970381228147467[85] = dt;
   out_5032970381228147467[86] = 0;
   out_5032970381228147467[87] = 0;
   out_5032970381228147467[88] = 0;
   out_5032970381228147467[89] = 0;
   out_5032970381228147467[90] = 0;
   out_5032970381228147467[91] = 0;
   out_5032970381228147467[92] = 0;
   out_5032970381228147467[93] = 0;
   out_5032970381228147467[94] = 0;
   out_5032970381228147467[95] = 1;
   out_5032970381228147467[96] = 0;
   out_5032970381228147467[97] = 0;
   out_5032970381228147467[98] = 0;
   out_5032970381228147467[99] = 0;
   out_5032970381228147467[100] = 0;
   out_5032970381228147467[101] = 0;
   out_5032970381228147467[102] = 0;
   out_5032970381228147467[103] = 0;
   out_5032970381228147467[104] = dt;
   out_5032970381228147467[105] = 0;
   out_5032970381228147467[106] = 0;
   out_5032970381228147467[107] = 0;
   out_5032970381228147467[108] = 0;
   out_5032970381228147467[109] = 0;
   out_5032970381228147467[110] = 0;
   out_5032970381228147467[111] = 0;
   out_5032970381228147467[112] = 0;
   out_5032970381228147467[113] = 0;
   out_5032970381228147467[114] = 1;
   out_5032970381228147467[115] = 0;
   out_5032970381228147467[116] = 0;
   out_5032970381228147467[117] = 0;
   out_5032970381228147467[118] = 0;
   out_5032970381228147467[119] = 0;
   out_5032970381228147467[120] = 0;
   out_5032970381228147467[121] = 0;
   out_5032970381228147467[122] = 0;
   out_5032970381228147467[123] = 0;
   out_5032970381228147467[124] = 0;
   out_5032970381228147467[125] = 0;
   out_5032970381228147467[126] = 0;
   out_5032970381228147467[127] = 0;
   out_5032970381228147467[128] = 0;
   out_5032970381228147467[129] = 0;
   out_5032970381228147467[130] = 0;
   out_5032970381228147467[131] = 0;
   out_5032970381228147467[132] = 0;
   out_5032970381228147467[133] = 1;
   out_5032970381228147467[134] = 0;
   out_5032970381228147467[135] = 0;
   out_5032970381228147467[136] = 0;
   out_5032970381228147467[137] = 0;
   out_5032970381228147467[138] = 0;
   out_5032970381228147467[139] = 0;
   out_5032970381228147467[140] = 0;
   out_5032970381228147467[141] = 0;
   out_5032970381228147467[142] = 0;
   out_5032970381228147467[143] = 0;
   out_5032970381228147467[144] = 0;
   out_5032970381228147467[145] = 0;
   out_5032970381228147467[146] = 0;
   out_5032970381228147467[147] = 0;
   out_5032970381228147467[148] = 0;
   out_5032970381228147467[149] = 0;
   out_5032970381228147467[150] = 0;
   out_5032970381228147467[151] = 0;
   out_5032970381228147467[152] = 1;
   out_5032970381228147467[153] = 0;
   out_5032970381228147467[154] = 0;
   out_5032970381228147467[155] = 0;
   out_5032970381228147467[156] = 0;
   out_5032970381228147467[157] = 0;
   out_5032970381228147467[158] = 0;
   out_5032970381228147467[159] = 0;
   out_5032970381228147467[160] = 0;
   out_5032970381228147467[161] = 0;
   out_5032970381228147467[162] = 0;
   out_5032970381228147467[163] = 0;
   out_5032970381228147467[164] = 0;
   out_5032970381228147467[165] = 0;
   out_5032970381228147467[166] = 0;
   out_5032970381228147467[167] = 0;
   out_5032970381228147467[168] = 0;
   out_5032970381228147467[169] = 0;
   out_5032970381228147467[170] = 0;
   out_5032970381228147467[171] = 1;
   out_5032970381228147467[172] = 0;
   out_5032970381228147467[173] = 0;
   out_5032970381228147467[174] = 0;
   out_5032970381228147467[175] = 0;
   out_5032970381228147467[176] = 0;
   out_5032970381228147467[177] = 0;
   out_5032970381228147467[178] = 0;
   out_5032970381228147467[179] = 0;
   out_5032970381228147467[180] = 0;
   out_5032970381228147467[181] = 0;
   out_5032970381228147467[182] = 0;
   out_5032970381228147467[183] = 0;
   out_5032970381228147467[184] = 0;
   out_5032970381228147467[185] = 0;
   out_5032970381228147467[186] = 0;
   out_5032970381228147467[187] = 0;
   out_5032970381228147467[188] = 0;
   out_5032970381228147467[189] = 0;
   out_5032970381228147467[190] = 1;
   out_5032970381228147467[191] = 0;
   out_5032970381228147467[192] = 0;
   out_5032970381228147467[193] = 0;
   out_5032970381228147467[194] = 0;
   out_5032970381228147467[195] = 0;
   out_5032970381228147467[196] = 0;
   out_5032970381228147467[197] = 0;
   out_5032970381228147467[198] = 0;
   out_5032970381228147467[199] = 0;
   out_5032970381228147467[200] = 0;
   out_5032970381228147467[201] = 0;
   out_5032970381228147467[202] = 0;
   out_5032970381228147467[203] = 0;
   out_5032970381228147467[204] = 0;
   out_5032970381228147467[205] = 0;
   out_5032970381228147467[206] = 0;
   out_5032970381228147467[207] = 0;
   out_5032970381228147467[208] = 0;
   out_5032970381228147467[209] = 1;
   out_5032970381228147467[210] = 0;
   out_5032970381228147467[211] = 0;
   out_5032970381228147467[212] = 0;
   out_5032970381228147467[213] = 0;
   out_5032970381228147467[214] = 0;
   out_5032970381228147467[215] = 0;
   out_5032970381228147467[216] = 0;
   out_5032970381228147467[217] = 0;
   out_5032970381228147467[218] = 0;
   out_5032970381228147467[219] = 0;
   out_5032970381228147467[220] = 0;
   out_5032970381228147467[221] = 0;
   out_5032970381228147467[222] = 0;
   out_5032970381228147467[223] = 0;
   out_5032970381228147467[224] = 0;
   out_5032970381228147467[225] = 0;
   out_5032970381228147467[226] = 0;
   out_5032970381228147467[227] = 0;
   out_5032970381228147467[228] = 1;
   out_5032970381228147467[229] = 0;
   out_5032970381228147467[230] = 0;
   out_5032970381228147467[231] = 0;
   out_5032970381228147467[232] = 0;
   out_5032970381228147467[233] = 0;
   out_5032970381228147467[234] = 0;
   out_5032970381228147467[235] = 0;
   out_5032970381228147467[236] = 0;
   out_5032970381228147467[237] = 0;
   out_5032970381228147467[238] = 0;
   out_5032970381228147467[239] = 0;
   out_5032970381228147467[240] = 0;
   out_5032970381228147467[241] = 0;
   out_5032970381228147467[242] = 0;
   out_5032970381228147467[243] = 0;
   out_5032970381228147467[244] = 0;
   out_5032970381228147467[245] = 0;
   out_5032970381228147467[246] = 0;
   out_5032970381228147467[247] = 1;
   out_5032970381228147467[248] = 0;
   out_5032970381228147467[249] = 0;
   out_5032970381228147467[250] = 0;
   out_5032970381228147467[251] = 0;
   out_5032970381228147467[252] = 0;
   out_5032970381228147467[253] = 0;
   out_5032970381228147467[254] = 0;
   out_5032970381228147467[255] = 0;
   out_5032970381228147467[256] = 0;
   out_5032970381228147467[257] = 0;
   out_5032970381228147467[258] = 0;
   out_5032970381228147467[259] = 0;
   out_5032970381228147467[260] = 0;
   out_5032970381228147467[261] = 0;
   out_5032970381228147467[262] = 0;
   out_5032970381228147467[263] = 0;
   out_5032970381228147467[264] = 0;
   out_5032970381228147467[265] = 0;
   out_5032970381228147467[266] = 1;
   out_5032970381228147467[267] = 0;
   out_5032970381228147467[268] = 0;
   out_5032970381228147467[269] = 0;
   out_5032970381228147467[270] = 0;
   out_5032970381228147467[271] = 0;
   out_5032970381228147467[272] = 0;
   out_5032970381228147467[273] = 0;
   out_5032970381228147467[274] = 0;
   out_5032970381228147467[275] = 0;
   out_5032970381228147467[276] = 0;
   out_5032970381228147467[277] = 0;
   out_5032970381228147467[278] = 0;
   out_5032970381228147467[279] = 0;
   out_5032970381228147467[280] = 0;
   out_5032970381228147467[281] = 0;
   out_5032970381228147467[282] = 0;
   out_5032970381228147467[283] = 0;
   out_5032970381228147467[284] = 0;
   out_5032970381228147467[285] = 1;
   out_5032970381228147467[286] = 0;
   out_5032970381228147467[287] = 0;
   out_5032970381228147467[288] = 0;
   out_5032970381228147467[289] = 0;
   out_5032970381228147467[290] = 0;
   out_5032970381228147467[291] = 0;
   out_5032970381228147467[292] = 0;
   out_5032970381228147467[293] = 0;
   out_5032970381228147467[294] = 0;
   out_5032970381228147467[295] = 0;
   out_5032970381228147467[296] = 0;
   out_5032970381228147467[297] = 0;
   out_5032970381228147467[298] = 0;
   out_5032970381228147467[299] = 0;
   out_5032970381228147467[300] = 0;
   out_5032970381228147467[301] = 0;
   out_5032970381228147467[302] = 0;
   out_5032970381228147467[303] = 0;
   out_5032970381228147467[304] = 1;
   out_5032970381228147467[305] = 0;
   out_5032970381228147467[306] = 0;
   out_5032970381228147467[307] = 0;
   out_5032970381228147467[308] = 0;
   out_5032970381228147467[309] = 0;
   out_5032970381228147467[310] = 0;
   out_5032970381228147467[311] = 0;
   out_5032970381228147467[312] = 0;
   out_5032970381228147467[313] = 0;
   out_5032970381228147467[314] = 0;
   out_5032970381228147467[315] = 0;
   out_5032970381228147467[316] = 0;
   out_5032970381228147467[317] = 0;
   out_5032970381228147467[318] = 0;
   out_5032970381228147467[319] = 0;
   out_5032970381228147467[320] = 0;
   out_5032970381228147467[321] = 0;
   out_5032970381228147467[322] = 0;
   out_5032970381228147467[323] = 1;
}
void h_4(double *state, double *unused, double *out_7618278180547779421) {
   out_7618278180547779421[0] = state[6] + state[9];
   out_7618278180547779421[1] = state[7] + state[10];
   out_7618278180547779421[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_6588138960762335933) {
   out_6588138960762335933[0] = 0;
   out_6588138960762335933[1] = 0;
   out_6588138960762335933[2] = 0;
   out_6588138960762335933[3] = 0;
   out_6588138960762335933[4] = 0;
   out_6588138960762335933[5] = 0;
   out_6588138960762335933[6] = 1;
   out_6588138960762335933[7] = 0;
   out_6588138960762335933[8] = 0;
   out_6588138960762335933[9] = 1;
   out_6588138960762335933[10] = 0;
   out_6588138960762335933[11] = 0;
   out_6588138960762335933[12] = 0;
   out_6588138960762335933[13] = 0;
   out_6588138960762335933[14] = 0;
   out_6588138960762335933[15] = 0;
   out_6588138960762335933[16] = 0;
   out_6588138960762335933[17] = 0;
   out_6588138960762335933[18] = 0;
   out_6588138960762335933[19] = 0;
   out_6588138960762335933[20] = 0;
   out_6588138960762335933[21] = 0;
   out_6588138960762335933[22] = 0;
   out_6588138960762335933[23] = 0;
   out_6588138960762335933[24] = 0;
   out_6588138960762335933[25] = 1;
   out_6588138960762335933[26] = 0;
   out_6588138960762335933[27] = 0;
   out_6588138960762335933[28] = 1;
   out_6588138960762335933[29] = 0;
   out_6588138960762335933[30] = 0;
   out_6588138960762335933[31] = 0;
   out_6588138960762335933[32] = 0;
   out_6588138960762335933[33] = 0;
   out_6588138960762335933[34] = 0;
   out_6588138960762335933[35] = 0;
   out_6588138960762335933[36] = 0;
   out_6588138960762335933[37] = 0;
   out_6588138960762335933[38] = 0;
   out_6588138960762335933[39] = 0;
   out_6588138960762335933[40] = 0;
   out_6588138960762335933[41] = 0;
   out_6588138960762335933[42] = 0;
   out_6588138960762335933[43] = 0;
   out_6588138960762335933[44] = 1;
   out_6588138960762335933[45] = 0;
   out_6588138960762335933[46] = 0;
   out_6588138960762335933[47] = 1;
   out_6588138960762335933[48] = 0;
   out_6588138960762335933[49] = 0;
   out_6588138960762335933[50] = 0;
   out_6588138960762335933[51] = 0;
   out_6588138960762335933[52] = 0;
   out_6588138960762335933[53] = 0;
}
void h_10(double *state, double *unused, double *out_5956604434663063125) {
   out_5956604434663063125[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_5956604434663063125[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_5956604434663063125[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_6715409743454385022) {
   out_6715409743454385022[0] = 0;
   out_6715409743454385022[1] = 9.8100000000000005*cos(state[1]);
   out_6715409743454385022[2] = 0;
   out_6715409743454385022[3] = 0;
   out_6715409743454385022[4] = -state[8];
   out_6715409743454385022[5] = state[7];
   out_6715409743454385022[6] = 0;
   out_6715409743454385022[7] = state[5];
   out_6715409743454385022[8] = -state[4];
   out_6715409743454385022[9] = 0;
   out_6715409743454385022[10] = 0;
   out_6715409743454385022[11] = 0;
   out_6715409743454385022[12] = 1;
   out_6715409743454385022[13] = 0;
   out_6715409743454385022[14] = 0;
   out_6715409743454385022[15] = 1;
   out_6715409743454385022[16] = 0;
   out_6715409743454385022[17] = 0;
   out_6715409743454385022[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_6715409743454385022[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_6715409743454385022[20] = 0;
   out_6715409743454385022[21] = state[8];
   out_6715409743454385022[22] = 0;
   out_6715409743454385022[23] = -state[6];
   out_6715409743454385022[24] = -state[5];
   out_6715409743454385022[25] = 0;
   out_6715409743454385022[26] = state[3];
   out_6715409743454385022[27] = 0;
   out_6715409743454385022[28] = 0;
   out_6715409743454385022[29] = 0;
   out_6715409743454385022[30] = 0;
   out_6715409743454385022[31] = 1;
   out_6715409743454385022[32] = 0;
   out_6715409743454385022[33] = 0;
   out_6715409743454385022[34] = 1;
   out_6715409743454385022[35] = 0;
   out_6715409743454385022[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_6715409743454385022[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_6715409743454385022[38] = 0;
   out_6715409743454385022[39] = -state[7];
   out_6715409743454385022[40] = state[6];
   out_6715409743454385022[41] = 0;
   out_6715409743454385022[42] = state[4];
   out_6715409743454385022[43] = -state[3];
   out_6715409743454385022[44] = 0;
   out_6715409743454385022[45] = 0;
   out_6715409743454385022[46] = 0;
   out_6715409743454385022[47] = 0;
   out_6715409743454385022[48] = 0;
   out_6715409743454385022[49] = 0;
   out_6715409743454385022[50] = 1;
   out_6715409743454385022[51] = 0;
   out_6715409743454385022[52] = 0;
   out_6715409743454385022[53] = 1;
}
void h_13(double *state, double *unused, double *out_1194330709066251479) {
   out_1194330709066251479[0] = state[3];
   out_1194330709066251479[1] = state[4];
   out_1194330709066251479[2] = state[5];
}
void H_13(double *state, double *unused, double *out_3375865135430003132) {
   out_3375865135430003132[0] = 0;
   out_3375865135430003132[1] = 0;
   out_3375865135430003132[2] = 0;
   out_3375865135430003132[3] = 1;
   out_3375865135430003132[4] = 0;
   out_3375865135430003132[5] = 0;
   out_3375865135430003132[6] = 0;
   out_3375865135430003132[7] = 0;
   out_3375865135430003132[8] = 0;
   out_3375865135430003132[9] = 0;
   out_3375865135430003132[10] = 0;
   out_3375865135430003132[11] = 0;
   out_3375865135430003132[12] = 0;
   out_3375865135430003132[13] = 0;
   out_3375865135430003132[14] = 0;
   out_3375865135430003132[15] = 0;
   out_3375865135430003132[16] = 0;
   out_3375865135430003132[17] = 0;
   out_3375865135430003132[18] = 0;
   out_3375865135430003132[19] = 0;
   out_3375865135430003132[20] = 0;
   out_3375865135430003132[21] = 0;
   out_3375865135430003132[22] = 1;
   out_3375865135430003132[23] = 0;
   out_3375865135430003132[24] = 0;
   out_3375865135430003132[25] = 0;
   out_3375865135430003132[26] = 0;
   out_3375865135430003132[27] = 0;
   out_3375865135430003132[28] = 0;
   out_3375865135430003132[29] = 0;
   out_3375865135430003132[30] = 0;
   out_3375865135430003132[31] = 0;
   out_3375865135430003132[32] = 0;
   out_3375865135430003132[33] = 0;
   out_3375865135430003132[34] = 0;
   out_3375865135430003132[35] = 0;
   out_3375865135430003132[36] = 0;
   out_3375865135430003132[37] = 0;
   out_3375865135430003132[38] = 0;
   out_3375865135430003132[39] = 0;
   out_3375865135430003132[40] = 0;
   out_3375865135430003132[41] = 1;
   out_3375865135430003132[42] = 0;
   out_3375865135430003132[43] = 0;
   out_3375865135430003132[44] = 0;
   out_3375865135430003132[45] = 0;
   out_3375865135430003132[46] = 0;
   out_3375865135430003132[47] = 0;
   out_3375865135430003132[48] = 0;
   out_3375865135430003132[49] = 0;
   out_3375865135430003132[50] = 0;
   out_3375865135430003132[51] = 0;
   out_3375865135430003132[52] = 0;
   out_3375865135430003132[53] = 0;
}
void h_14(double *state, double *unused, double *out_8419123727036403433) {
   out_8419123727036403433[0] = state[6];
   out_8419123727036403433[1] = state[7];
   out_8419123727036403433[2] = state[8];
}
void H_14(double *state, double *unused, double *out_2624898104422851404) {
   out_2624898104422851404[0] = 0;
   out_2624898104422851404[1] = 0;
   out_2624898104422851404[2] = 0;
   out_2624898104422851404[3] = 0;
   out_2624898104422851404[4] = 0;
   out_2624898104422851404[5] = 0;
   out_2624898104422851404[6] = 1;
   out_2624898104422851404[7] = 0;
   out_2624898104422851404[8] = 0;
   out_2624898104422851404[9] = 0;
   out_2624898104422851404[10] = 0;
   out_2624898104422851404[11] = 0;
   out_2624898104422851404[12] = 0;
   out_2624898104422851404[13] = 0;
   out_2624898104422851404[14] = 0;
   out_2624898104422851404[15] = 0;
   out_2624898104422851404[16] = 0;
   out_2624898104422851404[17] = 0;
   out_2624898104422851404[18] = 0;
   out_2624898104422851404[19] = 0;
   out_2624898104422851404[20] = 0;
   out_2624898104422851404[21] = 0;
   out_2624898104422851404[22] = 0;
   out_2624898104422851404[23] = 0;
   out_2624898104422851404[24] = 0;
   out_2624898104422851404[25] = 1;
   out_2624898104422851404[26] = 0;
   out_2624898104422851404[27] = 0;
   out_2624898104422851404[28] = 0;
   out_2624898104422851404[29] = 0;
   out_2624898104422851404[30] = 0;
   out_2624898104422851404[31] = 0;
   out_2624898104422851404[32] = 0;
   out_2624898104422851404[33] = 0;
   out_2624898104422851404[34] = 0;
   out_2624898104422851404[35] = 0;
   out_2624898104422851404[36] = 0;
   out_2624898104422851404[37] = 0;
   out_2624898104422851404[38] = 0;
   out_2624898104422851404[39] = 0;
   out_2624898104422851404[40] = 0;
   out_2624898104422851404[41] = 0;
   out_2624898104422851404[42] = 0;
   out_2624898104422851404[43] = 0;
   out_2624898104422851404[44] = 1;
   out_2624898104422851404[45] = 0;
   out_2624898104422851404[46] = 0;
   out_2624898104422851404[47] = 0;
   out_2624898104422851404[48] = 0;
   out_2624898104422851404[49] = 0;
   out_2624898104422851404[50] = 0;
   out_2624898104422851404[51] = 0;
   out_2624898104422851404[52] = 0;
   out_2624898104422851404[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_8560828130109272763) {
  err_fun(nom_x, delta_x, out_8560828130109272763);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_157920368283105894) {
  inv_err_fun(nom_x, true_x, out_157920368283105894);
}
void pose_H_mod_fun(double *state, double *out_3313916881971386680) {
  H_mod_fun(state, out_3313916881971386680);
}
void pose_f_fun(double *state, double dt, double *out_5589389168188525463) {
  f_fun(state,  dt, out_5589389168188525463);
}
void pose_F_fun(double *state, double dt, double *out_5032970381228147467) {
  F_fun(state,  dt, out_5032970381228147467);
}
void pose_h_4(double *state, double *unused, double *out_7618278180547779421) {
  h_4(state, unused, out_7618278180547779421);
}
void pose_H_4(double *state, double *unused, double *out_6588138960762335933) {
  H_4(state, unused, out_6588138960762335933);
}
void pose_h_10(double *state, double *unused, double *out_5956604434663063125) {
  h_10(state, unused, out_5956604434663063125);
}
void pose_H_10(double *state, double *unused, double *out_6715409743454385022) {
  H_10(state, unused, out_6715409743454385022);
}
void pose_h_13(double *state, double *unused, double *out_1194330709066251479) {
  h_13(state, unused, out_1194330709066251479);
}
void pose_H_13(double *state, double *unused, double *out_3375865135430003132) {
  H_13(state, unused, out_3375865135430003132);
}
void pose_h_14(double *state, double *unused, double *out_8419123727036403433) {
  h_14(state, unused, out_8419123727036403433);
}
void pose_H_14(double *state, double *unused, double *out_2624898104422851404) {
  H_14(state, unused, out_2624898104422851404);
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

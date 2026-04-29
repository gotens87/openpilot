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
void err_fun(double *nom_x, double *delta_x, double *out_8885455317417923135) {
   out_8885455317417923135[0] = delta_x[0] + nom_x[0];
   out_8885455317417923135[1] = delta_x[1] + nom_x[1];
   out_8885455317417923135[2] = delta_x[2] + nom_x[2];
   out_8885455317417923135[3] = delta_x[3] + nom_x[3];
   out_8885455317417923135[4] = delta_x[4] + nom_x[4];
   out_8885455317417923135[5] = delta_x[5] + nom_x[5];
   out_8885455317417923135[6] = delta_x[6] + nom_x[6];
   out_8885455317417923135[7] = delta_x[7] + nom_x[7];
   out_8885455317417923135[8] = delta_x[8] + nom_x[8];
   out_8885455317417923135[9] = delta_x[9] + nom_x[9];
   out_8885455317417923135[10] = delta_x[10] + nom_x[10];
   out_8885455317417923135[11] = delta_x[11] + nom_x[11];
   out_8885455317417923135[12] = delta_x[12] + nom_x[12];
   out_8885455317417923135[13] = delta_x[13] + nom_x[13];
   out_8885455317417923135[14] = delta_x[14] + nom_x[14];
   out_8885455317417923135[15] = delta_x[15] + nom_x[15];
   out_8885455317417923135[16] = delta_x[16] + nom_x[16];
   out_8885455317417923135[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7818671957054169652) {
   out_7818671957054169652[0] = -nom_x[0] + true_x[0];
   out_7818671957054169652[1] = -nom_x[1] + true_x[1];
   out_7818671957054169652[2] = -nom_x[2] + true_x[2];
   out_7818671957054169652[3] = -nom_x[3] + true_x[3];
   out_7818671957054169652[4] = -nom_x[4] + true_x[4];
   out_7818671957054169652[5] = -nom_x[5] + true_x[5];
   out_7818671957054169652[6] = -nom_x[6] + true_x[6];
   out_7818671957054169652[7] = -nom_x[7] + true_x[7];
   out_7818671957054169652[8] = -nom_x[8] + true_x[8];
   out_7818671957054169652[9] = -nom_x[9] + true_x[9];
   out_7818671957054169652[10] = -nom_x[10] + true_x[10];
   out_7818671957054169652[11] = -nom_x[11] + true_x[11];
   out_7818671957054169652[12] = -nom_x[12] + true_x[12];
   out_7818671957054169652[13] = -nom_x[13] + true_x[13];
   out_7818671957054169652[14] = -nom_x[14] + true_x[14];
   out_7818671957054169652[15] = -nom_x[15] + true_x[15];
   out_7818671957054169652[16] = -nom_x[16] + true_x[16];
   out_7818671957054169652[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_6824361727259051753) {
   out_6824361727259051753[0] = 1.0;
   out_6824361727259051753[1] = 0.0;
   out_6824361727259051753[2] = 0.0;
   out_6824361727259051753[3] = 0.0;
   out_6824361727259051753[4] = 0.0;
   out_6824361727259051753[5] = 0.0;
   out_6824361727259051753[6] = 0.0;
   out_6824361727259051753[7] = 0.0;
   out_6824361727259051753[8] = 0.0;
   out_6824361727259051753[9] = 0.0;
   out_6824361727259051753[10] = 0.0;
   out_6824361727259051753[11] = 0.0;
   out_6824361727259051753[12] = 0.0;
   out_6824361727259051753[13] = 0.0;
   out_6824361727259051753[14] = 0.0;
   out_6824361727259051753[15] = 0.0;
   out_6824361727259051753[16] = 0.0;
   out_6824361727259051753[17] = 0.0;
   out_6824361727259051753[18] = 0.0;
   out_6824361727259051753[19] = 1.0;
   out_6824361727259051753[20] = 0.0;
   out_6824361727259051753[21] = 0.0;
   out_6824361727259051753[22] = 0.0;
   out_6824361727259051753[23] = 0.0;
   out_6824361727259051753[24] = 0.0;
   out_6824361727259051753[25] = 0.0;
   out_6824361727259051753[26] = 0.0;
   out_6824361727259051753[27] = 0.0;
   out_6824361727259051753[28] = 0.0;
   out_6824361727259051753[29] = 0.0;
   out_6824361727259051753[30] = 0.0;
   out_6824361727259051753[31] = 0.0;
   out_6824361727259051753[32] = 0.0;
   out_6824361727259051753[33] = 0.0;
   out_6824361727259051753[34] = 0.0;
   out_6824361727259051753[35] = 0.0;
   out_6824361727259051753[36] = 0.0;
   out_6824361727259051753[37] = 0.0;
   out_6824361727259051753[38] = 1.0;
   out_6824361727259051753[39] = 0.0;
   out_6824361727259051753[40] = 0.0;
   out_6824361727259051753[41] = 0.0;
   out_6824361727259051753[42] = 0.0;
   out_6824361727259051753[43] = 0.0;
   out_6824361727259051753[44] = 0.0;
   out_6824361727259051753[45] = 0.0;
   out_6824361727259051753[46] = 0.0;
   out_6824361727259051753[47] = 0.0;
   out_6824361727259051753[48] = 0.0;
   out_6824361727259051753[49] = 0.0;
   out_6824361727259051753[50] = 0.0;
   out_6824361727259051753[51] = 0.0;
   out_6824361727259051753[52] = 0.0;
   out_6824361727259051753[53] = 0.0;
   out_6824361727259051753[54] = 0.0;
   out_6824361727259051753[55] = 0.0;
   out_6824361727259051753[56] = 0.0;
   out_6824361727259051753[57] = 1.0;
   out_6824361727259051753[58] = 0.0;
   out_6824361727259051753[59] = 0.0;
   out_6824361727259051753[60] = 0.0;
   out_6824361727259051753[61] = 0.0;
   out_6824361727259051753[62] = 0.0;
   out_6824361727259051753[63] = 0.0;
   out_6824361727259051753[64] = 0.0;
   out_6824361727259051753[65] = 0.0;
   out_6824361727259051753[66] = 0.0;
   out_6824361727259051753[67] = 0.0;
   out_6824361727259051753[68] = 0.0;
   out_6824361727259051753[69] = 0.0;
   out_6824361727259051753[70] = 0.0;
   out_6824361727259051753[71] = 0.0;
   out_6824361727259051753[72] = 0.0;
   out_6824361727259051753[73] = 0.0;
   out_6824361727259051753[74] = 0.0;
   out_6824361727259051753[75] = 0.0;
   out_6824361727259051753[76] = 1.0;
   out_6824361727259051753[77] = 0.0;
   out_6824361727259051753[78] = 0.0;
   out_6824361727259051753[79] = 0.0;
   out_6824361727259051753[80] = 0.0;
   out_6824361727259051753[81] = 0.0;
   out_6824361727259051753[82] = 0.0;
   out_6824361727259051753[83] = 0.0;
   out_6824361727259051753[84] = 0.0;
   out_6824361727259051753[85] = 0.0;
   out_6824361727259051753[86] = 0.0;
   out_6824361727259051753[87] = 0.0;
   out_6824361727259051753[88] = 0.0;
   out_6824361727259051753[89] = 0.0;
   out_6824361727259051753[90] = 0.0;
   out_6824361727259051753[91] = 0.0;
   out_6824361727259051753[92] = 0.0;
   out_6824361727259051753[93] = 0.0;
   out_6824361727259051753[94] = 0.0;
   out_6824361727259051753[95] = 1.0;
   out_6824361727259051753[96] = 0.0;
   out_6824361727259051753[97] = 0.0;
   out_6824361727259051753[98] = 0.0;
   out_6824361727259051753[99] = 0.0;
   out_6824361727259051753[100] = 0.0;
   out_6824361727259051753[101] = 0.0;
   out_6824361727259051753[102] = 0.0;
   out_6824361727259051753[103] = 0.0;
   out_6824361727259051753[104] = 0.0;
   out_6824361727259051753[105] = 0.0;
   out_6824361727259051753[106] = 0.0;
   out_6824361727259051753[107] = 0.0;
   out_6824361727259051753[108] = 0.0;
   out_6824361727259051753[109] = 0.0;
   out_6824361727259051753[110] = 0.0;
   out_6824361727259051753[111] = 0.0;
   out_6824361727259051753[112] = 0.0;
   out_6824361727259051753[113] = 0.0;
   out_6824361727259051753[114] = 1.0;
   out_6824361727259051753[115] = 0.0;
   out_6824361727259051753[116] = 0.0;
   out_6824361727259051753[117] = 0.0;
   out_6824361727259051753[118] = 0.0;
   out_6824361727259051753[119] = 0.0;
   out_6824361727259051753[120] = 0.0;
   out_6824361727259051753[121] = 0.0;
   out_6824361727259051753[122] = 0.0;
   out_6824361727259051753[123] = 0.0;
   out_6824361727259051753[124] = 0.0;
   out_6824361727259051753[125] = 0.0;
   out_6824361727259051753[126] = 0.0;
   out_6824361727259051753[127] = 0.0;
   out_6824361727259051753[128] = 0.0;
   out_6824361727259051753[129] = 0.0;
   out_6824361727259051753[130] = 0.0;
   out_6824361727259051753[131] = 0.0;
   out_6824361727259051753[132] = 0.0;
   out_6824361727259051753[133] = 1.0;
   out_6824361727259051753[134] = 0.0;
   out_6824361727259051753[135] = 0.0;
   out_6824361727259051753[136] = 0.0;
   out_6824361727259051753[137] = 0.0;
   out_6824361727259051753[138] = 0.0;
   out_6824361727259051753[139] = 0.0;
   out_6824361727259051753[140] = 0.0;
   out_6824361727259051753[141] = 0.0;
   out_6824361727259051753[142] = 0.0;
   out_6824361727259051753[143] = 0.0;
   out_6824361727259051753[144] = 0.0;
   out_6824361727259051753[145] = 0.0;
   out_6824361727259051753[146] = 0.0;
   out_6824361727259051753[147] = 0.0;
   out_6824361727259051753[148] = 0.0;
   out_6824361727259051753[149] = 0.0;
   out_6824361727259051753[150] = 0.0;
   out_6824361727259051753[151] = 0.0;
   out_6824361727259051753[152] = 1.0;
   out_6824361727259051753[153] = 0.0;
   out_6824361727259051753[154] = 0.0;
   out_6824361727259051753[155] = 0.0;
   out_6824361727259051753[156] = 0.0;
   out_6824361727259051753[157] = 0.0;
   out_6824361727259051753[158] = 0.0;
   out_6824361727259051753[159] = 0.0;
   out_6824361727259051753[160] = 0.0;
   out_6824361727259051753[161] = 0.0;
   out_6824361727259051753[162] = 0.0;
   out_6824361727259051753[163] = 0.0;
   out_6824361727259051753[164] = 0.0;
   out_6824361727259051753[165] = 0.0;
   out_6824361727259051753[166] = 0.0;
   out_6824361727259051753[167] = 0.0;
   out_6824361727259051753[168] = 0.0;
   out_6824361727259051753[169] = 0.0;
   out_6824361727259051753[170] = 0.0;
   out_6824361727259051753[171] = 1.0;
   out_6824361727259051753[172] = 0.0;
   out_6824361727259051753[173] = 0.0;
   out_6824361727259051753[174] = 0.0;
   out_6824361727259051753[175] = 0.0;
   out_6824361727259051753[176] = 0.0;
   out_6824361727259051753[177] = 0.0;
   out_6824361727259051753[178] = 0.0;
   out_6824361727259051753[179] = 0.0;
   out_6824361727259051753[180] = 0.0;
   out_6824361727259051753[181] = 0.0;
   out_6824361727259051753[182] = 0.0;
   out_6824361727259051753[183] = 0.0;
   out_6824361727259051753[184] = 0.0;
   out_6824361727259051753[185] = 0.0;
   out_6824361727259051753[186] = 0.0;
   out_6824361727259051753[187] = 0.0;
   out_6824361727259051753[188] = 0.0;
   out_6824361727259051753[189] = 0.0;
   out_6824361727259051753[190] = 1.0;
   out_6824361727259051753[191] = 0.0;
   out_6824361727259051753[192] = 0.0;
   out_6824361727259051753[193] = 0.0;
   out_6824361727259051753[194] = 0.0;
   out_6824361727259051753[195] = 0.0;
   out_6824361727259051753[196] = 0.0;
   out_6824361727259051753[197] = 0.0;
   out_6824361727259051753[198] = 0.0;
   out_6824361727259051753[199] = 0.0;
   out_6824361727259051753[200] = 0.0;
   out_6824361727259051753[201] = 0.0;
   out_6824361727259051753[202] = 0.0;
   out_6824361727259051753[203] = 0.0;
   out_6824361727259051753[204] = 0.0;
   out_6824361727259051753[205] = 0.0;
   out_6824361727259051753[206] = 0.0;
   out_6824361727259051753[207] = 0.0;
   out_6824361727259051753[208] = 0.0;
   out_6824361727259051753[209] = 1.0;
   out_6824361727259051753[210] = 0.0;
   out_6824361727259051753[211] = 0.0;
   out_6824361727259051753[212] = 0.0;
   out_6824361727259051753[213] = 0.0;
   out_6824361727259051753[214] = 0.0;
   out_6824361727259051753[215] = 0.0;
   out_6824361727259051753[216] = 0.0;
   out_6824361727259051753[217] = 0.0;
   out_6824361727259051753[218] = 0.0;
   out_6824361727259051753[219] = 0.0;
   out_6824361727259051753[220] = 0.0;
   out_6824361727259051753[221] = 0.0;
   out_6824361727259051753[222] = 0.0;
   out_6824361727259051753[223] = 0.0;
   out_6824361727259051753[224] = 0.0;
   out_6824361727259051753[225] = 0.0;
   out_6824361727259051753[226] = 0.0;
   out_6824361727259051753[227] = 0.0;
   out_6824361727259051753[228] = 1.0;
   out_6824361727259051753[229] = 0.0;
   out_6824361727259051753[230] = 0.0;
   out_6824361727259051753[231] = 0.0;
   out_6824361727259051753[232] = 0.0;
   out_6824361727259051753[233] = 0.0;
   out_6824361727259051753[234] = 0.0;
   out_6824361727259051753[235] = 0.0;
   out_6824361727259051753[236] = 0.0;
   out_6824361727259051753[237] = 0.0;
   out_6824361727259051753[238] = 0.0;
   out_6824361727259051753[239] = 0.0;
   out_6824361727259051753[240] = 0.0;
   out_6824361727259051753[241] = 0.0;
   out_6824361727259051753[242] = 0.0;
   out_6824361727259051753[243] = 0.0;
   out_6824361727259051753[244] = 0.0;
   out_6824361727259051753[245] = 0.0;
   out_6824361727259051753[246] = 0.0;
   out_6824361727259051753[247] = 1.0;
   out_6824361727259051753[248] = 0.0;
   out_6824361727259051753[249] = 0.0;
   out_6824361727259051753[250] = 0.0;
   out_6824361727259051753[251] = 0.0;
   out_6824361727259051753[252] = 0.0;
   out_6824361727259051753[253] = 0.0;
   out_6824361727259051753[254] = 0.0;
   out_6824361727259051753[255] = 0.0;
   out_6824361727259051753[256] = 0.0;
   out_6824361727259051753[257] = 0.0;
   out_6824361727259051753[258] = 0.0;
   out_6824361727259051753[259] = 0.0;
   out_6824361727259051753[260] = 0.0;
   out_6824361727259051753[261] = 0.0;
   out_6824361727259051753[262] = 0.0;
   out_6824361727259051753[263] = 0.0;
   out_6824361727259051753[264] = 0.0;
   out_6824361727259051753[265] = 0.0;
   out_6824361727259051753[266] = 1.0;
   out_6824361727259051753[267] = 0.0;
   out_6824361727259051753[268] = 0.0;
   out_6824361727259051753[269] = 0.0;
   out_6824361727259051753[270] = 0.0;
   out_6824361727259051753[271] = 0.0;
   out_6824361727259051753[272] = 0.0;
   out_6824361727259051753[273] = 0.0;
   out_6824361727259051753[274] = 0.0;
   out_6824361727259051753[275] = 0.0;
   out_6824361727259051753[276] = 0.0;
   out_6824361727259051753[277] = 0.0;
   out_6824361727259051753[278] = 0.0;
   out_6824361727259051753[279] = 0.0;
   out_6824361727259051753[280] = 0.0;
   out_6824361727259051753[281] = 0.0;
   out_6824361727259051753[282] = 0.0;
   out_6824361727259051753[283] = 0.0;
   out_6824361727259051753[284] = 0.0;
   out_6824361727259051753[285] = 1.0;
   out_6824361727259051753[286] = 0.0;
   out_6824361727259051753[287] = 0.0;
   out_6824361727259051753[288] = 0.0;
   out_6824361727259051753[289] = 0.0;
   out_6824361727259051753[290] = 0.0;
   out_6824361727259051753[291] = 0.0;
   out_6824361727259051753[292] = 0.0;
   out_6824361727259051753[293] = 0.0;
   out_6824361727259051753[294] = 0.0;
   out_6824361727259051753[295] = 0.0;
   out_6824361727259051753[296] = 0.0;
   out_6824361727259051753[297] = 0.0;
   out_6824361727259051753[298] = 0.0;
   out_6824361727259051753[299] = 0.0;
   out_6824361727259051753[300] = 0.0;
   out_6824361727259051753[301] = 0.0;
   out_6824361727259051753[302] = 0.0;
   out_6824361727259051753[303] = 0.0;
   out_6824361727259051753[304] = 1.0;
   out_6824361727259051753[305] = 0.0;
   out_6824361727259051753[306] = 0.0;
   out_6824361727259051753[307] = 0.0;
   out_6824361727259051753[308] = 0.0;
   out_6824361727259051753[309] = 0.0;
   out_6824361727259051753[310] = 0.0;
   out_6824361727259051753[311] = 0.0;
   out_6824361727259051753[312] = 0.0;
   out_6824361727259051753[313] = 0.0;
   out_6824361727259051753[314] = 0.0;
   out_6824361727259051753[315] = 0.0;
   out_6824361727259051753[316] = 0.0;
   out_6824361727259051753[317] = 0.0;
   out_6824361727259051753[318] = 0.0;
   out_6824361727259051753[319] = 0.0;
   out_6824361727259051753[320] = 0.0;
   out_6824361727259051753[321] = 0.0;
   out_6824361727259051753[322] = 0.0;
   out_6824361727259051753[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_1755259086508631527) {
   out_1755259086508631527[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_1755259086508631527[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_1755259086508631527[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_1755259086508631527[3] = dt*state[12] + state[3];
   out_1755259086508631527[4] = dt*state[13] + state[4];
   out_1755259086508631527[5] = dt*state[14] + state[5];
   out_1755259086508631527[6] = state[6];
   out_1755259086508631527[7] = state[7];
   out_1755259086508631527[8] = state[8];
   out_1755259086508631527[9] = state[9];
   out_1755259086508631527[10] = state[10];
   out_1755259086508631527[11] = state[11];
   out_1755259086508631527[12] = state[12];
   out_1755259086508631527[13] = state[13];
   out_1755259086508631527[14] = state[14];
   out_1755259086508631527[15] = state[15];
   out_1755259086508631527[16] = state[16];
   out_1755259086508631527[17] = state[17];
}
void F_fun(double *state, double dt, double *out_4099616348303124892) {
   out_4099616348303124892[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4099616348303124892[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4099616348303124892[2] = 0;
   out_4099616348303124892[3] = 0;
   out_4099616348303124892[4] = 0;
   out_4099616348303124892[5] = 0;
   out_4099616348303124892[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4099616348303124892[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4099616348303124892[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4099616348303124892[9] = 0;
   out_4099616348303124892[10] = 0;
   out_4099616348303124892[11] = 0;
   out_4099616348303124892[12] = 0;
   out_4099616348303124892[13] = 0;
   out_4099616348303124892[14] = 0;
   out_4099616348303124892[15] = 0;
   out_4099616348303124892[16] = 0;
   out_4099616348303124892[17] = 0;
   out_4099616348303124892[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4099616348303124892[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4099616348303124892[20] = 0;
   out_4099616348303124892[21] = 0;
   out_4099616348303124892[22] = 0;
   out_4099616348303124892[23] = 0;
   out_4099616348303124892[24] = 0;
   out_4099616348303124892[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4099616348303124892[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4099616348303124892[27] = 0;
   out_4099616348303124892[28] = 0;
   out_4099616348303124892[29] = 0;
   out_4099616348303124892[30] = 0;
   out_4099616348303124892[31] = 0;
   out_4099616348303124892[32] = 0;
   out_4099616348303124892[33] = 0;
   out_4099616348303124892[34] = 0;
   out_4099616348303124892[35] = 0;
   out_4099616348303124892[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4099616348303124892[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4099616348303124892[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4099616348303124892[39] = 0;
   out_4099616348303124892[40] = 0;
   out_4099616348303124892[41] = 0;
   out_4099616348303124892[42] = 0;
   out_4099616348303124892[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4099616348303124892[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4099616348303124892[45] = 0;
   out_4099616348303124892[46] = 0;
   out_4099616348303124892[47] = 0;
   out_4099616348303124892[48] = 0;
   out_4099616348303124892[49] = 0;
   out_4099616348303124892[50] = 0;
   out_4099616348303124892[51] = 0;
   out_4099616348303124892[52] = 0;
   out_4099616348303124892[53] = 0;
   out_4099616348303124892[54] = 0;
   out_4099616348303124892[55] = 0;
   out_4099616348303124892[56] = 0;
   out_4099616348303124892[57] = 1;
   out_4099616348303124892[58] = 0;
   out_4099616348303124892[59] = 0;
   out_4099616348303124892[60] = 0;
   out_4099616348303124892[61] = 0;
   out_4099616348303124892[62] = 0;
   out_4099616348303124892[63] = 0;
   out_4099616348303124892[64] = 0;
   out_4099616348303124892[65] = 0;
   out_4099616348303124892[66] = dt;
   out_4099616348303124892[67] = 0;
   out_4099616348303124892[68] = 0;
   out_4099616348303124892[69] = 0;
   out_4099616348303124892[70] = 0;
   out_4099616348303124892[71] = 0;
   out_4099616348303124892[72] = 0;
   out_4099616348303124892[73] = 0;
   out_4099616348303124892[74] = 0;
   out_4099616348303124892[75] = 0;
   out_4099616348303124892[76] = 1;
   out_4099616348303124892[77] = 0;
   out_4099616348303124892[78] = 0;
   out_4099616348303124892[79] = 0;
   out_4099616348303124892[80] = 0;
   out_4099616348303124892[81] = 0;
   out_4099616348303124892[82] = 0;
   out_4099616348303124892[83] = 0;
   out_4099616348303124892[84] = 0;
   out_4099616348303124892[85] = dt;
   out_4099616348303124892[86] = 0;
   out_4099616348303124892[87] = 0;
   out_4099616348303124892[88] = 0;
   out_4099616348303124892[89] = 0;
   out_4099616348303124892[90] = 0;
   out_4099616348303124892[91] = 0;
   out_4099616348303124892[92] = 0;
   out_4099616348303124892[93] = 0;
   out_4099616348303124892[94] = 0;
   out_4099616348303124892[95] = 1;
   out_4099616348303124892[96] = 0;
   out_4099616348303124892[97] = 0;
   out_4099616348303124892[98] = 0;
   out_4099616348303124892[99] = 0;
   out_4099616348303124892[100] = 0;
   out_4099616348303124892[101] = 0;
   out_4099616348303124892[102] = 0;
   out_4099616348303124892[103] = 0;
   out_4099616348303124892[104] = dt;
   out_4099616348303124892[105] = 0;
   out_4099616348303124892[106] = 0;
   out_4099616348303124892[107] = 0;
   out_4099616348303124892[108] = 0;
   out_4099616348303124892[109] = 0;
   out_4099616348303124892[110] = 0;
   out_4099616348303124892[111] = 0;
   out_4099616348303124892[112] = 0;
   out_4099616348303124892[113] = 0;
   out_4099616348303124892[114] = 1;
   out_4099616348303124892[115] = 0;
   out_4099616348303124892[116] = 0;
   out_4099616348303124892[117] = 0;
   out_4099616348303124892[118] = 0;
   out_4099616348303124892[119] = 0;
   out_4099616348303124892[120] = 0;
   out_4099616348303124892[121] = 0;
   out_4099616348303124892[122] = 0;
   out_4099616348303124892[123] = 0;
   out_4099616348303124892[124] = 0;
   out_4099616348303124892[125] = 0;
   out_4099616348303124892[126] = 0;
   out_4099616348303124892[127] = 0;
   out_4099616348303124892[128] = 0;
   out_4099616348303124892[129] = 0;
   out_4099616348303124892[130] = 0;
   out_4099616348303124892[131] = 0;
   out_4099616348303124892[132] = 0;
   out_4099616348303124892[133] = 1;
   out_4099616348303124892[134] = 0;
   out_4099616348303124892[135] = 0;
   out_4099616348303124892[136] = 0;
   out_4099616348303124892[137] = 0;
   out_4099616348303124892[138] = 0;
   out_4099616348303124892[139] = 0;
   out_4099616348303124892[140] = 0;
   out_4099616348303124892[141] = 0;
   out_4099616348303124892[142] = 0;
   out_4099616348303124892[143] = 0;
   out_4099616348303124892[144] = 0;
   out_4099616348303124892[145] = 0;
   out_4099616348303124892[146] = 0;
   out_4099616348303124892[147] = 0;
   out_4099616348303124892[148] = 0;
   out_4099616348303124892[149] = 0;
   out_4099616348303124892[150] = 0;
   out_4099616348303124892[151] = 0;
   out_4099616348303124892[152] = 1;
   out_4099616348303124892[153] = 0;
   out_4099616348303124892[154] = 0;
   out_4099616348303124892[155] = 0;
   out_4099616348303124892[156] = 0;
   out_4099616348303124892[157] = 0;
   out_4099616348303124892[158] = 0;
   out_4099616348303124892[159] = 0;
   out_4099616348303124892[160] = 0;
   out_4099616348303124892[161] = 0;
   out_4099616348303124892[162] = 0;
   out_4099616348303124892[163] = 0;
   out_4099616348303124892[164] = 0;
   out_4099616348303124892[165] = 0;
   out_4099616348303124892[166] = 0;
   out_4099616348303124892[167] = 0;
   out_4099616348303124892[168] = 0;
   out_4099616348303124892[169] = 0;
   out_4099616348303124892[170] = 0;
   out_4099616348303124892[171] = 1;
   out_4099616348303124892[172] = 0;
   out_4099616348303124892[173] = 0;
   out_4099616348303124892[174] = 0;
   out_4099616348303124892[175] = 0;
   out_4099616348303124892[176] = 0;
   out_4099616348303124892[177] = 0;
   out_4099616348303124892[178] = 0;
   out_4099616348303124892[179] = 0;
   out_4099616348303124892[180] = 0;
   out_4099616348303124892[181] = 0;
   out_4099616348303124892[182] = 0;
   out_4099616348303124892[183] = 0;
   out_4099616348303124892[184] = 0;
   out_4099616348303124892[185] = 0;
   out_4099616348303124892[186] = 0;
   out_4099616348303124892[187] = 0;
   out_4099616348303124892[188] = 0;
   out_4099616348303124892[189] = 0;
   out_4099616348303124892[190] = 1;
   out_4099616348303124892[191] = 0;
   out_4099616348303124892[192] = 0;
   out_4099616348303124892[193] = 0;
   out_4099616348303124892[194] = 0;
   out_4099616348303124892[195] = 0;
   out_4099616348303124892[196] = 0;
   out_4099616348303124892[197] = 0;
   out_4099616348303124892[198] = 0;
   out_4099616348303124892[199] = 0;
   out_4099616348303124892[200] = 0;
   out_4099616348303124892[201] = 0;
   out_4099616348303124892[202] = 0;
   out_4099616348303124892[203] = 0;
   out_4099616348303124892[204] = 0;
   out_4099616348303124892[205] = 0;
   out_4099616348303124892[206] = 0;
   out_4099616348303124892[207] = 0;
   out_4099616348303124892[208] = 0;
   out_4099616348303124892[209] = 1;
   out_4099616348303124892[210] = 0;
   out_4099616348303124892[211] = 0;
   out_4099616348303124892[212] = 0;
   out_4099616348303124892[213] = 0;
   out_4099616348303124892[214] = 0;
   out_4099616348303124892[215] = 0;
   out_4099616348303124892[216] = 0;
   out_4099616348303124892[217] = 0;
   out_4099616348303124892[218] = 0;
   out_4099616348303124892[219] = 0;
   out_4099616348303124892[220] = 0;
   out_4099616348303124892[221] = 0;
   out_4099616348303124892[222] = 0;
   out_4099616348303124892[223] = 0;
   out_4099616348303124892[224] = 0;
   out_4099616348303124892[225] = 0;
   out_4099616348303124892[226] = 0;
   out_4099616348303124892[227] = 0;
   out_4099616348303124892[228] = 1;
   out_4099616348303124892[229] = 0;
   out_4099616348303124892[230] = 0;
   out_4099616348303124892[231] = 0;
   out_4099616348303124892[232] = 0;
   out_4099616348303124892[233] = 0;
   out_4099616348303124892[234] = 0;
   out_4099616348303124892[235] = 0;
   out_4099616348303124892[236] = 0;
   out_4099616348303124892[237] = 0;
   out_4099616348303124892[238] = 0;
   out_4099616348303124892[239] = 0;
   out_4099616348303124892[240] = 0;
   out_4099616348303124892[241] = 0;
   out_4099616348303124892[242] = 0;
   out_4099616348303124892[243] = 0;
   out_4099616348303124892[244] = 0;
   out_4099616348303124892[245] = 0;
   out_4099616348303124892[246] = 0;
   out_4099616348303124892[247] = 1;
   out_4099616348303124892[248] = 0;
   out_4099616348303124892[249] = 0;
   out_4099616348303124892[250] = 0;
   out_4099616348303124892[251] = 0;
   out_4099616348303124892[252] = 0;
   out_4099616348303124892[253] = 0;
   out_4099616348303124892[254] = 0;
   out_4099616348303124892[255] = 0;
   out_4099616348303124892[256] = 0;
   out_4099616348303124892[257] = 0;
   out_4099616348303124892[258] = 0;
   out_4099616348303124892[259] = 0;
   out_4099616348303124892[260] = 0;
   out_4099616348303124892[261] = 0;
   out_4099616348303124892[262] = 0;
   out_4099616348303124892[263] = 0;
   out_4099616348303124892[264] = 0;
   out_4099616348303124892[265] = 0;
   out_4099616348303124892[266] = 1;
   out_4099616348303124892[267] = 0;
   out_4099616348303124892[268] = 0;
   out_4099616348303124892[269] = 0;
   out_4099616348303124892[270] = 0;
   out_4099616348303124892[271] = 0;
   out_4099616348303124892[272] = 0;
   out_4099616348303124892[273] = 0;
   out_4099616348303124892[274] = 0;
   out_4099616348303124892[275] = 0;
   out_4099616348303124892[276] = 0;
   out_4099616348303124892[277] = 0;
   out_4099616348303124892[278] = 0;
   out_4099616348303124892[279] = 0;
   out_4099616348303124892[280] = 0;
   out_4099616348303124892[281] = 0;
   out_4099616348303124892[282] = 0;
   out_4099616348303124892[283] = 0;
   out_4099616348303124892[284] = 0;
   out_4099616348303124892[285] = 1;
   out_4099616348303124892[286] = 0;
   out_4099616348303124892[287] = 0;
   out_4099616348303124892[288] = 0;
   out_4099616348303124892[289] = 0;
   out_4099616348303124892[290] = 0;
   out_4099616348303124892[291] = 0;
   out_4099616348303124892[292] = 0;
   out_4099616348303124892[293] = 0;
   out_4099616348303124892[294] = 0;
   out_4099616348303124892[295] = 0;
   out_4099616348303124892[296] = 0;
   out_4099616348303124892[297] = 0;
   out_4099616348303124892[298] = 0;
   out_4099616348303124892[299] = 0;
   out_4099616348303124892[300] = 0;
   out_4099616348303124892[301] = 0;
   out_4099616348303124892[302] = 0;
   out_4099616348303124892[303] = 0;
   out_4099616348303124892[304] = 1;
   out_4099616348303124892[305] = 0;
   out_4099616348303124892[306] = 0;
   out_4099616348303124892[307] = 0;
   out_4099616348303124892[308] = 0;
   out_4099616348303124892[309] = 0;
   out_4099616348303124892[310] = 0;
   out_4099616348303124892[311] = 0;
   out_4099616348303124892[312] = 0;
   out_4099616348303124892[313] = 0;
   out_4099616348303124892[314] = 0;
   out_4099616348303124892[315] = 0;
   out_4099616348303124892[316] = 0;
   out_4099616348303124892[317] = 0;
   out_4099616348303124892[318] = 0;
   out_4099616348303124892[319] = 0;
   out_4099616348303124892[320] = 0;
   out_4099616348303124892[321] = 0;
   out_4099616348303124892[322] = 0;
   out_4099616348303124892[323] = 1;
}
void h_4(double *state, double *unused, double *out_2755321730925237079) {
   out_2755321730925237079[0] = state[6] + state[9];
   out_2755321730925237079[1] = state[7] + state[10];
   out_2755321730925237079[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_3052554517415144181) {
   out_3052554517415144181[0] = 0;
   out_3052554517415144181[1] = 0;
   out_3052554517415144181[2] = 0;
   out_3052554517415144181[3] = 0;
   out_3052554517415144181[4] = 0;
   out_3052554517415144181[5] = 0;
   out_3052554517415144181[6] = 1;
   out_3052554517415144181[7] = 0;
   out_3052554517415144181[8] = 0;
   out_3052554517415144181[9] = 1;
   out_3052554517415144181[10] = 0;
   out_3052554517415144181[11] = 0;
   out_3052554517415144181[12] = 0;
   out_3052554517415144181[13] = 0;
   out_3052554517415144181[14] = 0;
   out_3052554517415144181[15] = 0;
   out_3052554517415144181[16] = 0;
   out_3052554517415144181[17] = 0;
   out_3052554517415144181[18] = 0;
   out_3052554517415144181[19] = 0;
   out_3052554517415144181[20] = 0;
   out_3052554517415144181[21] = 0;
   out_3052554517415144181[22] = 0;
   out_3052554517415144181[23] = 0;
   out_3052554517415144181[24] = 0;
   out_3052554517415144181[25] = 1;
   out_3052554517415144181[26] = 0;
   out_3052554517415144181[27] = 0;
   out_3052554517415144181[28] = 1;
   out_3052554517415144181[29] = 0;
   out_3052554517415144181[30] = 0;
   out_3052554517415144181[31] = 0;
   out_3052554517415144181[32] = 0;
   out_3052554517415144181[33] = 0;
   out_3052554517415144181[34] = 0;
   out_3052554517415144181[35] = 0;
   out_3052554517415144181[36] = 0;
   out_3052554517415144181[37] = 0;
   out_3052554517415144181[38] = 0;
   out_3052554517415144181[39] = 0;
   out_3052554517415144181[40] = 0;
   out_3052554517415144181[41] = 0;
   out_3052554517415144181[42] = 0;
   out_3052554517415144181[43] = 0;
   out_3052554517415144181[44] = 1;
   out_3052554517415144181[45] = 0;
   out_3052554517415144181[46] = 0;
   out_3052554517415144181[47] = 1;
   out_3052554517415144181[48] = 0;
   out_3052554517415144181[49] = 0;
   out_3052554517415144181[50] = 0;
   out_3052554517415144181[51] = 0;
   out_3052554517415144181[52] = 0;
   out_3052554517415144181[53] = 0;
}
void h_10(double *state, double *unused, double *out_5342072843391651998) {
   out_5342072843391651998[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_5342072843391651998[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_5342072843391651998[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_7744556233743894871) {
   out_7744556233743894871[0] = 0;
   out_7744556233743894871[1] = 9.8100000000000005*cos(state[1]);
   out_7744556233743894871[2] = 0;
   out_7744556233743894871[3] = 0;
   out_7744556233743894871[4] = -state[8];
   out_7744556233743894871[5] = state[7];
   out_7744556233743894871[6] = 0;
   out_7744556233743894871[7] = state[5];
   out_7744556233743894871[8] = -state[4];
   out_7744556233743894871[9] = 0;
   out_7744556233743894871[10] = 0;
   out_7744556233743894871[11] = 0;
   out_7744556233743894871[12] = 1;
   out_7744556233743894871[13] = 0;
   out_7744556233743894871[14] = 0;
   out_7744556233743894871[15] = 1;
   out_7744556233743894871[16] = 0;
   out_7744556233743894871[17] = 0;
   out_7744556233743894871[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_7744556233743894871[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_7744556233743894871[20] = 0;
   out_7744556233743894871[21] = state[8];
   out_7744556233743894871[22] = 0;
   out_7744556233743894871[23] = -state[6];
   out_7744556233743894871[24] = -state[5];
   out_7744556233743894871[25] = 0;
   out_7744556233743894871[26] = state[3];
   out_7744556233743894871[27] = 0;
   out_7744556233743894871[28] = 0;
   out_7744556233743894871[29] = 0;
   out_7744556233743894871[30] = 0;
   out_7744556233743894871[31] = 1;
   out_7744556233743894871[32] = 0;
   out_7744556233743894871[33] = 0;
   out_7744556233743894871[34] = 1;
   out_7744556233743894871[35] = 0;
   out_7744556233743894871[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_7744556233743894871[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_7744556233743894871[38] = 0;
   out_7744556233743894871[39] = -state[7];
   out_7744556233743894871[40] = state[6];
   out_7744556233743894871[41] = 0;
   out_7744556233743894871[42] = state[4];
   out_7744556233743894871[43] = -state[3];
   out_7744556233743894871[44] = 0;
   out_7744556233743894871[45] = 0;
   out_7744556233743894871[46] = 0;
   out_7744556233743894871[47] = 0;
   out_7744556233743894871[48] = 0;
   out_7744556233743894871[49] = 0;
   out_7744556233743894871[50] = 1;
   out_7744556233743894871[51] = 0;
   out_7744556233743894871[52] = 0;
   out_7744556233743894871[53] = 1;
}
void h_13(double *state, double *unused, double *out_2174293281027337494) {
   out_2174293281027337494[0] = state[3];
   out_2174293281027337494[1] = state[4];
   out_2174293281027337494[2] = state[5];
}
void H_13(double *state, double *unused, double *out_6886309980717668205) {
   out_6886309980717668205[0] = 0;
   out_6886309980717668205[1] = 0;
   out_6886309980717668205[2] = 0;
   out_6886309980717668205[3] = 1;
   out_6886309980717668205[4] = 0;
   out_6886309980717668205[5] = 0;
   out_6886309980717668205[6] = 0;
   out_6886309980717668205[7] = 0;
   out_6886309980717668205[8] = 0;
   out_6886309980717668205[9] = 0;
   out_6886309980717668205[10] = 0;
   out_6886309980717668205[11] = 0;
   out_6886309980717668205[12] = 0;
   out_6886309980717668205[13] = 0;
   out_6886309980717668205[14] = 0;
   out_6886309980717668205[15] = 0;
   out_6886309980717668205[16] = 0;
   out_6886309980717668205[17] = 0;
   out_6886309980717668205[18] = 0;
   out_6886309980717668205[19] = 0;
   out_6886309980717668205[20] = 0;
   out_6886309980717668205[21] = 0;
   out_6886309980717668205[22] = 1;
   out_6886309980717668205[23] = 0;
   out_6886309980717668205[24] = 0;
   out_6886309980717668205[25] = 0;
   out_6886309980717668205[26] = 0;
   out_6886309980717668205[27] = 0;
   out_6886309980717668205[28] = 0;
   out_6886309980717668205[29] = 0;
   out_6886309980717668205[30] = 0;
   out_6886309980717668205[31] = 0;
   out_6886309980717668205[32] = 0;
   out_6886309980717668205[33] = 0;
   out_6886309980717668205[34] = 0;
   out_6886309980717668205[35] = 0;
   out_6886309980717668205[36] = 0;
   out_6886309980717668205[37] = 0;
   out_6886309980717668205[38] = 0;
   out_6886309980717668205[39] = 0;
   out_6886309980717668205[40] = 0;
   out_6886309980717668205[41] = 1;
   out_6886309980717668205[42] = 0;
   out_6886309980717668205[43] = 0;
   out_6886309980717668205[44] = 0;
   out_6886309980717668205[45] = 0;
   out_6886309980717668205[46] = 0;
   out_6886309980717668205[47] = 0;
   out_6886309980717668205[48] = 0;
   out_6886309980717668205[49] = 0;
   out_6886309980717668205[50] = 0;
   out_6886309980717668205[51] = 0;
   out_6886309980717668205[52] = 0;
   out_6886309980717668205[53] = 0;
}
void h_14(double *state, double *unused, double *out_7883954814799996171) {
   out_7883954814799996171[0] = state[6];
   out_7883954814799996171[1] = state[7];
   out_7883954814799996171[2] = state[8];
}
void H_14(double *state, double *unused, double *out_6135342949710516477) {
   out_6135342949710516477[0] = 0;
   out_6135342949710516477[1] = 0;
   out_6135342949710516477[2] = 0;
   out_6135342949710516477[3] = 0;
   out_6135342949710516477[4] = 0;
   out_6135342949710516477[5] = 0;
   out_6135342949710516477[6] = 1;
   out_6135342949710516477[7] = 0;
   out_6135342949710516477[8] = 0;
   out_6135342949710516477[9] = 0;
   out_6135342949710516477[10] = 0;
   out_6135342949710516477[11] = 0;
   out_6135342949710516477[12] = 0;
   out_6135342949710516477[13] = 0;
   out_6135342949710516477[14] = 0;
   out_6135342949710516477[15] = 0;
   out_6135342949710516477[16] = 0;
   out_6135342949710516477[17] = 0;
   out_6135342949710516477[18] = 0;
   out_6135342949710516477[19] = 0;
   out_6135342949710516477[20] = 0;
   out_6135342949710516477[21] = 0;
   out_6135342949710516477[22] = 0;
   out_6135342949710516477[23] = 0;
   out_6135342949710516477[24] = 0;
   out_6135342949710516477[25] = 1;
   out_6135342949710516477[26] = 0;
   out_6135342949710516477[27] = 0;
   out_6135342949710516477[28] = 0;
   out_6135342949710516477[29] = 0;
   out_6135342949710516477[30] = 0;
   out_6135342949710516477[31] = 0;
   out_6135342949710516477[32] = 0;
   out_6135342949710516477[33] = 0;
   out_6135342949710516477[34] = 0;
   out_6135342949710516477[35] = 0;
   out_6135342949710516477[36] = 0;
   out_6135342949710516477[37] = 0;
   out_6135342949710516477[38] = 0;
   out_6135342949710516477[39] = 0;
   out_6135342949710516477[40] = 0;
   out_6135342949710516477[41] = 0;
   out_6135342949710516477[42] = 0;
   out_6135342949710516477[43] = 0;
   out_6135342949710516477[44] = 1;
   out_6135342949710516477[45] = 0;
   out_6135342949710516477[46] = 0;
   out_6135342949710516477[47] = 0;
   out_6135342949710516477[48] = 0;
   out_6135342949710516477[49] = 0;
   out_6135342949710516477[50] = 0;
   out_6135342949710516477[51] = 0;
   out_6135342949710516477[52] = 0;
   out_6135342949710516477[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_8885455317417923135) {
  err_fun(nom_x, delta_x, out_8885455317417923135);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7818671957054169652) {
  inv_err_fun(nom_x, true_x, out_7818671957054169652);
}
void pose_H_mod_fun(double *state, double *out_6824361727259051753) {
  H_mod_fun(state, out_6824361727259051753);
}
void pose_f_fun(double *state, double dt, double *out_1755259086508631527) {
  f_fun(state,  dt, out_1755259086508631527);
}
void pose_F_fun(double *state, double dt, double *out_4099616348303124892) {
  F_fun(state,  dt, out_4099616348303124892);
}
void pose_h_4(double *state, double *unused, double *out_2755321730925237079) {
  h_4(state, unused, out_2755321730925237079);
}
void pose_H_4(double *state, double *unused, double *out_3052554517415144181) {
  H_4(state, unused, out_3052554517415144181);
}
void pose_h_10(double *state, double *unused, double *out_5342072843391651998) {
  h_10(state, unused, out_5342072843391651998);
}
void pose_H_10(double *state, double *unused, double *out_7744556233743894871) {
  H_10(state, unused, out_7744556233743894871);
}
void pose_h_13(double *state, double *unused, double *out_2174293281027337494) {
  h_13(state, unused, out_2174293281027337494);
}
void pose_H_13(double *state, double *unused, double *out_6886309980717668205) {
  H_13(state, unused, out_6886309980717668205);
}
void pose_h_14(double *state, double *unused, double *out_7883954814799996171) {
  h_14(state, unused, out_7883954814799996171);
}
void pose_H_14(double *state, double *unused, double *out_6135342949710516477) {
  H_14(state, unused, out_6135342949710516477);
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

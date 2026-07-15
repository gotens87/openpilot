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
void err_fun(double *nom_x, double *delta_x, double *out_1527400177482295711) {
   out_1527400177482295711[0] = delta_x[0] + nom_x[0];
   out_1527400177482295711[1] = delta_x[1] + nom_x[1];
   out_1527400177482295711[2] = delta_x[2] + nom_x[2];
   out_1527400177482295711[3] = delta_x[3] + nom_x[3];
   out_1527400177482295711[4] = delta_x[4] + nom_x[4];
   out_1527400177482295711[5] = delta_x[5] + nom_x[5];
   out_1527400177482295711[6] = delta_x[6] + nom_x[6];
   out_1527400177482295711[7] = delta_x[7] + nom_x[7];
   out_1527400177482295711[8] = delta_x[8] + nom_x[8];
   out_1527400177482295711[9] = delta_x[9] + nom_x[9];
   out_1527400177482295711[10] = delta_x[10] + nom_x[10];
   out_1527400177482295711[11] = delta_x[11] + nom_x[11];
   out_1527400177482295711[12] = delta_x[12] + nom_x[12];
   out_1527400177482295711[13] = delta_x[13] + nom_x[13];
   out_1527400177482295711[14] = delta_x[14] + nom_x[14];
   out_1527400177482295711[15] = delta_x[15] + nom_x[15];
   out_1527400177482295711[16] = delta_x[16] + nom_x[16];
   out_1527400177482295711[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7620805951199866590) {
   out_7620805951199866590[0] = -nom_x[0] + true_x[0];
   out_7620805951199866590[1] = -nom_x[1] + true_x[1];
   out_7620805951199866590[2] = -nom_x[2] + true_x[2];
   out_7620805951199866590[3] = -nom_x[3] + true_x[3];
   out_7620805951199866590[4] = -nom_x[4] + true_x[4];
   out_7620805951199866590[5] = -nom_x[5] + true_x[5];
   out_7620805951199866590[6] = -nom_x[6] + true_x[6];
   out_7620805951199866590[7] = -nom_x[7] + true_x[7];
   out_7620805951199866590[8] = -nom_x[8] + true_x[8];
   out_7620805951199866590[9] = -nom_x[9] + true_x[9];
   out_7620805951199866590[10] = -nom_x[10] + true_x[10];
   out_7620805951199866590[11] = -nom_x[11] + true_x[11];
   out_7620805951199866590[12] = -nom_x[12] + true_x[12];
   out_7620805951199866590[13] = -nom_x[13] + true_x[13];
   out_7620805951199866590[14] = -nom_x[14] + true_x[14];
   out_7620805951199866590[15] = -nom_x[15] + true_x[15];
   out_7620805951199866590[16] = -nom_x[16] + true_x[16];
   out_7620805951199866590[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_2041474721312435071) {
   out_2041474721312435071[0] = 1.0;
   out_2041474721312435071[1] = 0.0;
   out_2041474721312435071[2] = 0.0;
   out_2041474721312435071[3] = 0.0;
   out_2041474721312435071[4] = 0.0;
   out_2041474721312435071[5] = 0.0;
   out_2041474721312435071[6] = 0.0;
   out_2041474721312435071[7] = 0.0;
   out_2041474721312435071[8] = 0.0;
   out_2041474721312435071[9] = 0.0;
   out_2041474721312435071[10] = 0.0;
   out_2041474721312435071[11] = 0.0;
   out_2041474721312435071[12] = 0.0;
   out_2041474721312435071[13] = 0.0;
   out_2041474721312435071[14] = 0.0;
   out_2041474721312435071[15] = 0.0;
   out_2041474721312435071[16] = 0.0;
   out_2041474721312435071[17] = 0.0;
   out_2041474721312435071[18] = 0.0;
   out_2041474721312435071[19] = 1.0;
   out_2041474721312435071[20] = 0.0;
   out_2041474721312435071[21] = 0.0;
   out_2041474721312435071[22] = 0.0;
   out_2041474721312435071[23] = 0.0;
   out_2041474721312435071[24] = 0.0;
   out_2041474721312435071[25] = 0.0;
   out_2041474721312435071[26] = 0.0;
   out_2041474721312435071[27] = 0.0;
   out_2041474721312435071[28] = 0.0;
   out_2041474721312435071[29] = 0.0;
   out_2041474721312435071[30] = 0.0;
   out_2041474721312435071[31] = 0.0;
   out_2041474721312435071[32] = 0.0;
   out_2041474721312435071[33] = 0.0;
   out_2041474721312435071[34] = 0.0;
   out_2041474721312435071[35] = 0.0;
   out_2041474721312435071[36] = 0.0;
   out_2041474721312435071[37] = 0.0;
   out_2041474721312435071[38] = 1.0;
   out_2041474721312435071[39] = 0.0;
   out_2041474721312435071[40] = 0.0;
   out_2041474721312435071[41] = 0.0;
   out_2041474721312435071[42] = 0.0;
   out_2041474721312435071[43] = 0.0;
   out_2041474721312435071[44] = 0.0;
   out_2041474721312435071[45] = 0.0;
   out_2041474721312435071[46] = 0.0;
   out_2041474721312435071[47] = 0.0;
   out_2041474721312435071[48] = 0.0;
   out_2041474721312435071[49] = 0.0;
   out_2041474721312435071[50] = 0.0;
   out_2041474721312435071[51] = 0.0;
   out_2041474721312435071[52] = 0.0;
   out_2041474721312435071[53] = 0.0;
   out_2041474721312435071[54] = 0.0;
   out_2041474721312435071[55] = 0.0;
   out_2041474721312435071[56] = 0.0;
   out_2041474721312435071[57] = 1.0;
   out_2041474721312435071[58] = 0.0;
   out_2041474721312435071[59] = 0.0;
   out_2041474721312435071[60] = 0.0;
   out_2041474721312435071[61] = 0.0;
   out_2041474721312435071[62] = 0.0;
   out_2041474721312435071[63] = 0.0;
   out_2041474721312435071[64] = 0.0;
   out_2041474721312435071[65] = 0.0;
   out_2041474721312435071[66] = 0.0;
   out_2041474721312435071[67] = 0.0;
   out_2041474721312435071[68] = 0.0;
   out_2041474721312435071[69] = 0.0;
   out_2041474721312435071[70] = 0.0;
   out_2041474721312435071[71] = 0.0;
   out_2041474721312435071[72] = 0.0;
   out_2041474721312435071[73] = 0.0;
   out_2041474721312435071[74] = 0.0;
   out_2041474721312435071[75] = 0.0;
   out_2041474721312435071[76] = 1.0;
   out_2041474721312435071[77] = 0.0;
   out_2041474721312435071[78] = 0.0;
   out_2041474721312435071[79] = 0.0;
   out_2041474721312435071[80] = 0.0;
   out_2041474721312435071[81] = 0.0;
   out_2041474721312435071[82] = 0.0;
   out_2041474721312435071[83] = 0.0;
   out_2041474721312435071[84] = 0.0;
   out_2041474721312435071[85] = 0.0;
   out_2041474721312435071[86] = 0.0;
   out_2041474721312435071[87] = 0.0;
   out_2041474721312435071[88] = 0.0;
   out_2041474721312435071[89] = 0.0;
   out_2041474721312435071[90] = 0.0;
   out_2041474721312435071[91] = 0.0;
   out_2041474721312435071[92] = 0.0;
   out_2041474721312435071[93] = 0.0;
   out_2041474721312435071[94] = 0.0;
   out_2041474721312435071[95] = 1.0;
   out_2041474721312435071[96] = 0.0;
   out_2041474721312435071[97] = 0.0;
   out_2041474721312435071[98] = 0.0;
   out_2041474721312435071[99] = 0.0;
   out_2041474721312435071[100] = 0.0;
   out_2041474721312435071[101] = 0.0;
   out_2041474721312435071[102] = 0.0;
   out_2041474721312435071[103] = 0.0;
   out_2041474721312435071[104] = 0.0;
   out_2041474721312435071[105] = 0.0;
   out_2041474721312435071[106] = 0.0;
   out_2041474721312435071[107] = 0.0;
   out_2041474721312435071[108] = 0.0;
   out_2041474721312435071[109] = 0.0;
   out_2041474721312435071[110] = 0.0;
   out_2041474721312435071[111] = 0.0;
   out_2041474721312435071[112] = 0.0;
   out_2041474721312435071[113] = 0.0;
   out_2041474721312435071[114] = 1.0;
   out_2041474721312435071[115] = 0.0;
   out_2041474721312435071[116] = 0.0;
   out_2041474721312435071[117] = 0.0;
   out_2041474721312435071[118] = 0.0;
   out_2041474721312435071[119] = 0.0;
   out_2041474721312435071[120] = 0.0;
   out_2041474721312435071[121] = 0.0;
   out_2041474721312435071[122] = 0.0;
   out_2041474721312435071[123] = 0.0;
   out_2041474721312435071[124] = 0.0;
   out_2041474721312435071[125] = 0.0;
   out_2041474721312435071[126] = 0.0;
   out_2041474721312435071[127] = 0.0;
   out_2041474721312435071[128] = 0.0;
   out_2041474721312435071[129] = 0.0;
   out_2041474721312435071[130] = 0.0;
   out_2041474721312435071[131] = 0.0;
   out_2041474721312435071[132] = 0.0;
   out_2041474721312435071[133] = 1.0;
   out_2041474721312435071[134] = 0.0;
   out_2041474721312435071[135] = 0.0;
   out_2041474721312435071[136] = 0.0;
   out_2041474721312435071[137] = 0.0;
   out_2041474721312435071[138] = 0.0;
   out_2041474721312435071[139] = 0.0;
   out_2041474721312435071[140] = 0.0;
   out_2041474721312435071[141] = 0.0;
   out_2041474721312435071[142] = 0.0;
   out_2041474721312435071[143] = 0.0;
   out_2041474721312435071[144] = 0.0;
   out_2041474721312435071[145] = 0.0;
   out_2041474721312435071[146] = 0.0;
   out_2041474721312435071[147] = 0.0;
   out_2041474721312435071[148] = 0.0;
   out_2041474721312435071[149] = 0.0;
   out_2041474721312435071[150] = 0.0;
   out_2041474721312435071[151] = 0.0;
   out_2041474721312435071[152] = 1.0;
   out_2041474721312435071[153] = 0.0;
   out_2041474721312435071[154] = 0.0;
   out_2041474721312435071[155] = 0.0;
   out_2041474721312435071[156] = 0.0;
   out_2041474721312435071[157] = 0.0;
   out_2041474721312435071[158] = 0.0;
   out_2041474721312435071[159] = 0.0;
   out_2041474721312435071[160] = 0.0;
   out_2041474721312435071[161] = 0.0;
   out_2041474721312435071[162] = 0.0;
   out_2041474721312435071[163] = 0.0;
   out_2041474721312435071[164] = 0.0;
   out_2041474721312435071[165] = 0.0;
   out_2041474721312435071[166] = 0.0;
   out_2041474721312435071[167] = 0.0;
   out_2041474721312435071[168] = 0.0;
   out_2041474721312435071[169] = 0.0;
   out_2041474721312435071[170] = 0.0;
   out_2041474721312435071[171] = 1.0;
   out_2041474721312435071[172] = 0.0;
   out_2041474721312435071[173] = 0.0;
   out_2041474721312435071[174] = 0.0;
   out_2041474721312435071[175] = 0.0;
   out_2041474721312435071[176] = 0.0;
   out_2041474721312435071[177] = 0.0;
   out_2041474721312435071[178] = 0.0;
   out_2041474721312435071[179] = 0.0;
   out_2041474721312435071[180] = 0.0;
   out_2041474721312435071[181] = 0.0;
   out_2041474721312435071[182] = 0.0;
   out_2041474721312435071[183] = 0.0;
   out_2041474721312435071[184] = 0.0;
   out_2041474721312435071[185] = 0.0;
   out_2041474721312435071[186] = 0.0;
   out_2041474721312435071[187] = 0.0;
   out_2041474721312435071[188] = 0.0;
   out_2041474721312435071[189] = 0.0;
   out_2041474721312435071[190] = 1.0;
   out_2041474721312435071[191] = 0.0;
   out_2041474721312435071[192] = 0.0;
   out_2041474721312435071[193] = 0.0;
   out_2041474721312435071[194] = 0.0;
   out_2041474721312435071[195] = 0.0;
   out_2041474721312435071[196] = 0.0;
   out_2041474721312435071[197] = 0.0;
   out_2041474721312435071[198] = 0.0;
   out_2041474721312435071[199] = 0.0;
   out_2041474721312435071[200] = 0.0;
   out_2041474721312435071[201] = 0.0;
   out_2041474721312435071[202] = 0.0;
   out_2041474721312435071[203] = 0.0;
   out_2041474721312435071[204] = 0.0;
   out_2041474721312435071[205] = 0.0;
   out_2041474721312435071[206] = 0.0;
   out_2041474721312435071[207] = 0.0;
   out_2041474721312435071[208] = 0.0;
   out_2041474721312435071[209] = 1.0;
   out_2041474721312435071[210] = 0.0;
   out_2041474721312435071[211] = 0.0;
   out_2041474721312435071[212] = 0.0;
   out_2041474721312435071[213] = 0.0;
   out_2041474721312435071[214] = 0.0;
   out_2041474721312435071[215] = 0.0;
   out_2041474721312435071[216] = 0.0;
   out_2041474721312435071[217] = 0.0;
   out_2041474721312435071[218] = 0.0;
   out_2041474721312435071[219] = 0.0;
   out_2041474721312435071[220] = 0.0;
   out_2041474721312435071[221] = 0.0;
   out_2041474721312435071[222] = 0.0;
   out_2041474721312435071[223] = 0.0;
   out_2041474721312435071[224] = 0.0;
   out_2041474721312435071[225] = 0.0;
   out_2041474721312435071[226] = 0.0;
   out_2041474721312435071[227] = 0.0;
   out_2041474721312435071[228] = 1.0;
   out_2041474721312435071[229] = 0.0;
   out_2041474721312435071[230] = 0.0;
   out_2041474721312435071[231] = 0.0;
   out_2041474721312435071[232] = 0.0;
   out_2041474721312435071[233] = 0.0;
   out_2041474721312435071[234] = 0.0;
   out_2041474721312435071[235] = 0.0;
   out_2041474721312435071[236] = 0.0;
   out_2041474721312435071[237] = 0.0;
   out_2041474721312435071[238] = 0.0;
   out_2041474721312435071[239] = 0.0;
   out_2041474721312435071[240] = 0.0;
   out_2041474721312435071[241] = 0.0;
   out_2041474721312435071[242] = 0.0;
   out_2041474721312435071[243] = 0.0;
   out_2041474721312435071[244] = 0.0;
   out_2041474721312435071[245] = 0.0;
   out_2041474721312435071[246] = 0.0;
   out_2041474721312435071[247] = 1.0;
   out_2041474721312435071[248] = 0.0;
   out_2041474721312435071[249] = 0.0;
   out_2041474721312435071[250] = 0.0;
   out_2041474721312435071[251] = 0.0;
   out_2041474721312435071[252] = 0.0;
   out_2041474721312435071[253] = 0.0;
   out_2041474721312435071[254] = 0.0;
   out_2041474721312435071[255] = 0.0;
   out_2041474721312435071[256] = 0.0;
   out_2041474721312435071[257] = 0.0;
   out_2041474721312435071[258] = 0.0;
   out_2041474721312435071[259] = 0.0;
   out_2041474721312435071[260] = 0.0;
   out_2041474721312435071[261] = 0.0;
   out_2041474721312435071[262] = 0.0;
   out_2041474721312435071[263] = 0.0;
   out_2041474721312435071[264] = 0.0;
   out_2041474721312435071[265] = 0.0;
   out_2041474721312435071[266] = 1.0;
   out_2041474721312435071[267] = 0.0;
   out_2041474721312435071[268] = 0.0;
   out_2041474721312435071[269] = 0.0;
   out_2041474721312435071[270] = 0.0;
   out_2041474721312435071[271] = 0.0;
   out_2041474721312435071[272] = 0.0;
   out_2041474721312435071[273] = 0.0;
   out_2041474721312435071[274] = 0.0;
   out_2041474721312435071[275] = 0.0;
   out_2041474721312435071[276] = 0.0;
   out_2041474721312435071[277] = 0.0;
   out_2041474721312435071[278] = 0.0;
   out_2041474721312435071[279] = 0.0;
   out_2041474721312435071[280] = 0.0;
   out_2041474721312435071[281] = 0.0;
   out_2041474721312435071[282] = 0.0;
   out_2041474721312435071[283] = 0.0;
   out_2041474721312435071[284] = 0.0;
   out_2041474721312435071[285] = 1.0;
   out_2041474721312435071[286] = 0.0;
   out_2041474721312435071[287] = 0.0;
   out_2041474721312435071[288] = 0.0;
   out_2041474721312435071[289] = 0.0;
   out_2041474721312435071[290] = 0.0;
   out_2041474721312435071[291] = 0.0;
   out_2041474721312435071[292] = 0.0;
   out_2041474721312435071[293] = 0.0;
   out_2041474721312435071[294] = 0.0;
   out_2041474721312435071[295] = 0.0;
   out_2041474721312435071[296] = 0.0;
   out_2041474721312435071[297] = 0.0;
   out_2041474721312435071[298] = 0.0;
   out_2041474721312435071[299] = 0.0;
   out_2041474721312435071[300] = 0.0;
   out_2041474721312435071[301] = 0.0;
   out_2041474721312435071[302] = 0.0;
   out_2041474721312435071[303] = 0.0;
   out_2041474721312435071[304] = 1.0;
   out_2041474721312435071[305] = 0.0;
   out_2041474721312435071[306] = 0.0;
   out_2041474721312435071[307] = 0.0;
   out_2041474721312435071[308] = 0.0;
   out_2041474721312435071[309] = 0.0;
   out_2041474721312435071[310] = 0.0;
   out_2041474721312435071[311] = 0.0;
   out_2041474721312435071[312] = 0.0;
   out_2041474721312435071[313] = 0.0;
   out_2041474721312435071[314] = 0.0;
   out_2041474721312435071[315] = 0.0;
   out_2041474721312435071[316] = 0.0;
   out_2041474721312435071[317] = 0.0;
   out_2041474721312435071[318] = 0.0;
   out_2041474721312435071[319] = 0.0;
   out_2041474721312435071[320] = 0.0;
   out_2041474721312435071[321] = 0.0;
   out_2041474721312435071[322] = 0.0;
   out_2041474721312435071[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_3319995548118437592) {
   out_3319995548118437592[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_3319995548118437592[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_3319995548118437592[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_3319995548118437592[3] = dt*state[12] + state[3];
   out_3319995548118437592[4] = dt*state[13] + state[4];
   out_3319995548118437592[5] = dt*state[14] + state[5];
   out_3319995548118437592[6] = state[6];
   out_3319995548118437592[7] = state[7];
   out_3319995548118437592[8] = state[8];
   out_3319995548118437592[9] = state[9];
   out_3319995548118437592[10] = state[10];
   out_3319995548118437592[11] = state[11];
   out_3319995548118437592[12] = state[12];
   out_3319995548118437592[13] = state[13];
   out_3319995548118437592[14] = state[14];
   out_3319995548118437592[15] = state[15];
   out_3319995548118437592[16] = state[16];
   out_3319995548118437592[17] = state[17];
}
void F_fun(double *state, double dt, double *out_1297884238983223646) {
   out_1297884238983223646[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1297884238983223646[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1297884238983223646[2] = 0;
   out_1297884238983223646[3] = 0;
   out_1297884238983223646[4] = 0;
   out_1297884238983223646[5] = 0;
   out_1297884238983223646[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1297884238983223646[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1297884238983223646[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1297884238983223646[9] = 0;
   out_1297884238983223646[10] = 0;
   out_1297884238983223646[11] = 0;
   out_1297884238983223646[12] = 0;
   out_1297884238983223646[13] = 0;
   out_1297884238983223646[14] = 0;
   out_1297884238983223646[15] = 0;
   out_1297884238983223646[16] = 0;
   out_1297884238983223646[17] = 0;
   out_1297884238983223646[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1297884238983223646[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1297884238983223646[20] = 0;
   out_1297884238983223646[21] = 0;
   out_1297884238983223646[22] = 0;
   out_1297884238983223646[23] = 0;
   out_1297884238983223646[24] = 0;
   out_1297884238983223646[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1297884238983223646[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1297884238983223646[27] = 0;
   out_1297884238983223646[28] = 0;
   out_1297884238983223646[29] = 0;
   out_1297884238983223646[30] = 0;
   out_1297884238983223646[31] = 0;
   out_1297884238983223646[32] = 0;
   out_1297884238983223646[33] = 0;
   out_1297884238983223646[34] = 0;
   out_1297884238983223646[35] = 0;
   out_1297884238983223646[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1297884238983223646[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1297884238983223646[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1297884238983223646[39] = 0;
   out_1297884238983223646[40] = 0;
   out_1297884238983223646[41] = 0;
   out_1297884238983223646[42] = 0;
   out_1297884238983223646[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1297884238983223646[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1297884238983223646[45] = 0;
   out_1297884238983223646[46] = 0;
   out_1297884238983223646[47] = 0;
   out_1297884238983223646[48] = 0;
   out_1297884238983223646[49] = 0;
   out_1297884238983223646[50] = 0;
   out_1297884238983223646[51] = 0;
   out_1297884238983223646[52] = 0;
   out_1297884238983223646[53] = 0;
   out_1297884238983223646[54] = 0;
   out_1297884238983223646[55] = 0;
   out_1297884238983223646[56] = 0;
   out_1297884238983223646[57] = 1;
   out_1297884238983223646[58] = 0;
   out_1297884238983223646[59] = 0;
   out_1297884238983223646[60] = 0;
   out_1297884238983223646[61] = 0;
   out_1297884238983223646[62] = 0;
   out_1297884238983223646[63] = 0;
   out_1297884238983223646[64] = 0;
   out_1297884238983223646[65] = 0;
   out_1297884238983223646[66] = dt;
   out_1297884238983223646[67] = 0;
   out_1297884238983223646[68] = 0;
   out_1297884238983223646[69] = 0;
   out_1297884238983223646[70] = 0;
   out_1297884238983223646[71] = 0;
   out_1297884238983223646[72] = 0;
   out_1297884238983223646[73] = 0;
   out_1297884238983223646[74] = 0;
   out_1297884238983223646[75] = 0;
   out_1297884238983223646[76] = 1;
   out_1297884238983223646[77] = 0;
   out_1297884238983223646[78] = 0;
   out_1297884238983223646[79] = 0;
   out_1297884238983223646[80] = 0;
   out_1297884238983223646[81] = 0;
   out_1297884238983223646[82] = 0;
   out_1297884238983223646[83] = 0;
   out_1297884238983223646[84] = 0;
   out_1297884238983223646[85] = dt;
   out_1297884238983223646[86] = 0;
   out_1297884238983223646[87] = 0;
   out_1297884238983223646[88] = 0;
   out_1297884238983223646[89] = 0;
   out_1297884238983223646[90] = 0;
   out_1297884238983223646[91] = 0;
   out_1297884238983223646[92] = 0;
   out_1297884238983223646[93] = 0;
   out_1297884238983223646[94] = 0;
   out_1297884238983223646[95] = 1;
   out_1297884238983223646[96] = 0;
   out_1297884238983223646[97] = 0;
   out_1297884238983223646[98] = 0;
   out_1297884238983223646[99] = 0;
   out_1297884238983223646[100] = 0;
   out_1297884238983223646[101] = 0;
   out_1297884238983223646[102] = 0;
   out_1297884238983223646[103] = 0;
   out_1297884238983223646[104] = dt;
   out_1297884238983223646[105] = 0;
   out_1297884238983223646[106] = 0;
   out_1297884238983223646[107] = 0;
   out_1297884238983223646[108] = 0;
   out_1297884238983223646[109] = 0;
   out_1297884238983223646[110] = 0;
   out_1297884238983223646[111] = 0;
   out_1297884238983223646[112] = 0;
   out_1297884238983223646[113] = 0;
   out_1297884238983223646[114] = 1;
   out_1297884238983223646[115] = 0;
   out_1297884238983223646[116] = 0;
   out_1297884238983223646[117] = 0;
   out_1297884238983223646[118] = 0;
   out_1297884238983223646[119] = 0;
   out_1297884238983223646[120] = 0;
   out_1297884238983223646[121] = 0;
   out_1297884238983223646[122] = 0;
   out_1297884238983223646[123] = 0;
   out_1297884238983223646[124] = 0;
   out_1297884238983223646[125] = 0;
   out_1297884238983223646[126] = 0;
   out_1297884238983223646[127] = 0;
   out_1297884238983223646[128] = 0;
   out_1297884238983223646[129] = 0;
   out_1297884238983223646[130] = 0;
   out_1297884238983223646[131] = 0;
   out_1297884238983223646[132] = 0;
   out_1297884238983223646[133] = 1;
   out_1297884238983223646[134] = 0;
   out_1297884238983223646[135] = 0;
   out_1297884238983223646[136] = 0;
   out_1297884238983223646[137] = 0;
   out_1297884238983223646[138] = 0;
   out_1297884238983223646[139] = 0;
   out_1297884238983223646[140] = 0;
   out_1297884238983223646[141] = 0;
   out_1297884238983223646[142] = 0;
   out_1297884238983223646[143] = 0;
   out_1297884238983223646[144] = 0;
   out_1297884238983223646[145] = 0;
   out_1297884238983223646[146] = 0;
   out_1297884238983223646[147] = 0;
   out_1297884238983223646[148] = 0;
   out_1297884238983223646[149] = 0;
   out_1297884238983223646[150] = 0;
   out_1297884238983223646[151] = 0;
   out_1297884238983223646[152] = 1;
   out_1297884238983223646[153] = 0;
   out_1297884238983223646[154] = 0;
   out_1297884238983223646[155] = 0;
   out_1297884238983223646[156] = 0;
   out_1297884238983223646[157] = 0;
   out_1297884238983223646[158] = 0;
   out_1297884238983223646[159] = 0;
   out_1297884238983223646[160] = 0;
   out_1297884238983223646[161] = 0;
   out_1297884238983223646[162] = 0;
   out_1297884238983223646[163] = 0;
   out_1297884238983223646[164] = 0;
   out_1297884238983223646[165] = 0;
   out_1297884238983223646[166] = 0;
   out_1297884238983223646[167] = 0;
   out_1297884238983223646[168] = 0;
   out_1297884238983223646[169] = 0;
   out_1297884238983223646[170] = 0;
   out_1297884238983223646[171] = 1;
   out_1297884238983223646[172] = 0;
   out_1297884238983223646[173] = 0;
   out_1297884238983223646[174] = 0;
   out_1297884238983223646[175] = 0;
   out_1297884238983223646[176] = 0;
   out_1297884238983223646[177] = 0;
   out_1297884238983223646[178] = 0;
   out_1297884238983223646[179] = 0;
   out_1297884238983223646[180] = 0;
   out_1297884238983223646[181] = 0;
   out_1297884238983223646[182] = 0;
   out_1297884238983223646[183] = 0;
   out_1297884238983223646[184] = 0;
   out_1297884238983223646[185] = 0;
   out_1297884238983223646[186] = 0;
   out_1297884238983223646[187] = 0;
   out_1297884238983223646[188] = 0;
   out_1297884238983223646[189] = 0;
   out_1297884238983223646[190] = 1;
   out_1297884238983223646[191] = 0;
   out_1297884238983223646[192] = 0;
   out_1297884238983223646[193] = 0;
   out_1297884238983223646[194] = 0;
   out_1297884238983223646[195] = 0;
   out_1297884238983223646[196] = 0;
   out_1297884238983223646[197] = 0;
   out_1297884238983223646[198] = 0;
   out_1297884238983223646[199] = 0;
   out_1297884238983223646[200] = 0;
   out_1297884238983223646[201] = 0;
   out_1297884238983223646[202] = 0;
   out_1297884238983223646[203] = 0;
   out_1297884238983223646[204] = 0;
   out_1297884238983223646[205] = 0;
   out_1297884238983223646[206] = 0;
   out_1297884238983223646[207] = 0;
   out_1297884238983223646[208] = 0;
   out_1297884238983223646[209] = 1;
   out_1297884238983223646[210] = 0;
   out_1297884238983223646[211] = 0;
   out_1297884238983223646[212] = 0;
   out_1297884238983223646[213] = 0;
   out_1297884238983223646[214] = 0;
   out_1297884238983223646[215] = 0;
   out_1297884238983223646[216] = 0;
   out_1297884238983223646[217] = 0;
   out_1297884238983223646[218] = 0;
   out_1297884238983223646[219] = 0;
   out_1297884238983223646[220] = 0;
   out_1297884238983223646[221] = 0;
   out_1297884238983223646[222] = 0;
   out_1297884238983223646[223] = 0;
   out_1297884238983223646[224] = 0;
   out_1297884238983223646[225] = 0;
   out_1297884238983223646[226] = 0;
   out_1297884238983223646[227] = 0;
   out_1297884238983223646[228] = 1;
   out_1297884238983223646[229] = 0;
   out_1297884238983223646[230] = 0;
   out_1297884238983223646[231] = 0;
   out_1297884238983223646[232] = 0;
   out_1297884238983223646[233] = 0;
   out_1297884238983223646[234] = 0;
   out_1297884238983223646[235] = 0;
   out_1297884238983223646[236] = 0;
   out_1297884238983223646[237] = 0;
   out_1297884238983223646[238] = 0;
   out_1297884238983223646[239] = 0;
   out_1297884238983223646[240] = 0;
   out_1297884238983223646[241] = 0;
   out_1297884238983223646[242] = 0;
   out_1297884238983223646[243] = 0;
   out_1297884238983223646[244] = 0;
   out_1297884238983223646[245] = 0;
   out_1297884238983223646[246] = 0;
   out_1297884238983223646[247] = 1;
   out_1297884238983223646[248] = 0;
   out_1297884238983223646[249] = 0;
   out_1297884238983223646[250] = 0;
   out_1297884238983223646[251] = 0;
   out_1297884238983223646[252] = 0;
   out_1297884238983223646[253] = 0;
   out_1297884238983223646[254] = 0;
   out_1297884238983223646[255] = 0;
   out_1297884238983223646[256] = 0;
   out_1297884238983223646[257] = 0;
   out_1297884238983223646[258] = 0;
   out_1297884238983223646[259] = 0;
   out_1297884238983223646[260] = 0;
   out_1297884238983223646[261] = 0;
   out_1297884238983223646[262] = 0;
   out_1297884238983223646[263] = 0;
   out_1297884238983223646[264] = 0;
   out_1297884238983223646[265] = 0;
   out_1297884238983223646[266] = 1;
   out_1297884238983223646[267] = 0;
   out_1297884238983223646[268] = 0;
   out_1297884238983223646[269] = 0;
   out_1297884238983223646[270] = 0;
   out_1297884238983223646[271] = 0;
   out_1297884238983223646[272] = 0;
   out_1297884238983223646[273] = 0;
   out_1297884238983223646[274] = 0;
   out_1297884238983223646[275] = 0;
   out_1297884238983223646[276] = 0;
   out_1297884238983223646[277] = 0;
   out_1297884238983223646[278] = 0;
   out_1297884238983223646[279] = 0;
   out_1297884238983223646[280] = 0;
   out_1297884238983223646[281] = 0;
   out_1297884238983223646[282] = 0;
   out_1297884238983223646[283] = 0;
   out_1297884238983223646[284] = 0;
   out_1297884238983223646[285] = 1;
   out_1297884238983223646[286] = 0;
   out_1297884238983223646[287] = 0;
   out_1297884238983223646[288] = 0;
   out_1297884238983223646[289] = 0;
   out_1297884238983223646[290] = 0;
   out_1297884238983223646[291] = 0;
   out_1297884238983223646[292] = 0;
   out_1297884238983223646[293] = 0;
   out_1297884238983223646[294] = 0;
   out_1297884238983223646[295] = 0;
   out_1297884238983223646[296] = 0;
   out_1297884238983223646[297] = 0;
   out_1297884238983223646[298] = 0;
   out_1297884238983223646[299] = 0;
   out_1297884238983223646[300] = 0;
   out_1297884238983223646[301] = 0;
   out_1297884238983223646[302] = 0;
   out_1297884238983223646[303] = 0;
   out_1297884238983223646[304] = 1;
   out_1297884238983223646[305] = 0;
   out_1297884238983223646[306] = 0;
   out_1297884238983223646[307] = 0;
   out_1297884238983223646[308] = 0;
   out_1297884238983223646[309] = 0;
   out_1297884238983223646[310] = 0;
   out_1297884238983223646[311] = 0;
   out_1297884238983223646[312] = 0;
   out_1297884238983223646[313] = 0;
   out_1297884238983223646[314] = 0;
   out_1297884238983223646[315] = 0;
   out_1297884238983223646[316] = 0;
   out_1297884238983223646[317] = 0;
   out_1297884238983223646[318] = 0;
   out_1297884238983223646[319] = 0;
   out_1297884238983223646[320] = 0;
   out_1297884238983223646[321] = 0;
   out_1297884238983223646[322] = 0;
   out_1297884238983223646[323] = 1;
}
void h_4(double *state, double *unused, double *out_8728651807226544018) {
   out_8728651807226544018[0] = state[6] + state[9];
   out_8728651807226544018[1] = state[7] + state[10];
   out_8728651807226544018[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_5315696800103384324) {
   out_5315696800103384324[0] = 0;
   out_5315696800103384324[1] = 0;
   out_5315696800103384324[2] = 0;
   out_5315696800103384324[3] = 0;
   out_5315696800103384324[4] = 0;
   out_5315696800103384324[5] = 0;
   out_5315696800103384324[6] = 1;
   out_5315696800103384324[7] = 0;
   out_5315696800103384324[8] = 0;
   out_5315696800103384324[9] = 1;
   out_5315696800103384324[10] = 0;
   out_5315696800103384324[11] = 0;
   out_5315696800103384324[12] = 0;
   out_5315696800103384324[13] = 0;
   out_5315696800103384324[14] = 0;
   out_5315696800103384324[15] = 0;
   out_5315696800103384324[16] = 0;
   out_5315696800103384324[17] = 0;
   out_5315696800103384324[18] = 0;
   out_5315696800103384324[19] = 0;
   out_5315696800103384324[20] = 0;
   out_5315696800103384324[21] = 0;
   out_5315696800103384324[22] = 0;
   out_5315696800103384324[23] = 0;
   out_5315696800103384324[24] = 0;
   out_5315696800103384324[25] = 1;
   out_5315696800103384324[26] = 0;
   out_5315696800103384324[27] = 0;
   out_5315696800103384324[28] = 1;
   out_5315696800103384324[29] = 0;
   out_5315696800103384324[30] = 0;
   out_5315696800103384324[31] = 0;
   out_5315696800103384324[32] = 0;
   out_5315696800103384324[33] = 0;
   out_5315696800103384324[34] = 0;
   out_5315696800103384324[35] = 0;
   out_5315696800103384324[36] = 0;
   out_5315696800103384324[37] = 0;
   out_5315696800103384324[38] = 0;
   out_5315696800103384324[39] = 0;
   out_5315696800103384324[40] = 0;
   out_5315696800103384324[41] = 0;
   out_5315696800103384324[42] = 0;
   out_5315696800103384324[43] = 0;
   out_5315696800103384324[44] = 1;
   out_5315696800103384324[45] = 0;
   out_5315696800103384324[46] = 0;
   out_5315696800103384324[47] = 1;
   out_5315696800103384324[48] = 0;
   out_5315696800103384324[49] = 0;
   out_5315696800103384324[50] = 0;
   out_5315696800103384324[51] = 0;
   out_5315696800103384324[52] = 0;
   out_5315696800103384324[53] = 0;
}
void h_10(double *state, double *unused, double *out_3679201747496490736) {
   out_3679201747496490736[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_3679201747496490736[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_3679201747496490736[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_2420796569664890088) {
   out_2420796569664890088[0] = 0;
   out_2420796569664890088[1] = 9.8100000000000005*cos(state[1]);
   out_2420796569664890088[2] = 0;
   out_2420796569664890088[3] = 0;
   out_2420796569664890088[4] = -state[8];
   out_2420796569664890088[5] = state[7];
   out_2420796569664890088[6] = 0;
   out_2420796569664890088[7] = state[5];
   out_2420796569664890088[8] = -state[4];
   out_2420796569664890088[9] = 0;
   out_2420796569664890088[10] = 0;
   out_2420796569664890088[11] = 0;
   out_2420796569664890088[12] = 1;
   out_2420796569664890088[13] = 0;
   out_2420796569664890088[14] = 0;
   out_2420796569664890088[15] = 1;
   out_2420796569664890088[16] = 0;
   out_2420796569664890088[17] = 0;
   out_2420796569664890088[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_2420796569664890088[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_2420796569664890088[20] = 0;
   out_2420796569664890088[21] = state[8];
   out_2420796569664890088[22] = 0;
   out_2420796569664890088[23] = -state[6];
   out_2420796569664890088[24] = -state[5];
   out_2420796569664890088[25] = 0;
   out_2420796569664890088[26] = state[3];
   out_2420796569664890088[27] = 0;
   out_2420796569664890088[28] = 0;
   out_2420796569664890088[29] = 0;
   out_2420796569664890088[30] = 0;
   out_2420796569664890088[31] = 1;
   out_2420796569664890088[32] = 0;
   out_2420796569664890088[33] = 0;
   out_2420796569664890088[34] = 1;
   out_2420796569664890088[35] = 0;
   out_2420796569664890088[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_2420796569664890088[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_2420796569664890088[38] = 0;
   out_2420796569664890088[39] = -state[7];
   out_2420796569664890088[40] = state[6];
   out_2420796569664890088[41] = 0;
   out_2420796569664890088[42] = state[4];
   out_2420796569664890088[43] = -state[3];
   out_2420796569664890088[44] = 0;
   out_2420796569664890088[45] = 0;
   out_2420796569664890088[46] = 0;
   out_2420796569664890088[47] = 0;
   out_2420796569664890088[48] = 0;
   out_2420796569664890088[49] = 0;
   out_2420796569664890088[50] = 1;
   out_2420796569664890088[51] = 0;
   out_2420796569664890088[52] = 0;
   out_2420796569664890088[53] = 1;
}
void h_13(double *state, double *unused, double *out_4772104686591378688) {
   out_4772104686591378688[0] = state[3];
   out_4772104686591378688[1] = state[4];
   out_4772104686591378688[2] = state[5];
}
void H_13(double *state, double *unused, double *out_2103422974771051523) {
   out_2103422974771051523[0] = 0;
   out_2103422974771051523[1] = 0;
   out_2103422974771051523[2] = 0;
   out_2103422974771051523[3] = 1;
   out_2103422974771051523[4] = 0;
   out_2103422974771051523[5] = 0;
   out_2103422974771051523[6] = 0;
   out_2103422974771051523[7] = 0;
   out_2103422974771051523[8] = 0;
   out_2103422974771051523[9] = 0;
   out_2103422974771051523[10] = 0;
   out_2103422974771051523[11] = 0;
   out_2103422974771051523[12] = 0;
   out_2103422974771051523[13] = 0;
   out_2103422974771051523[14] = 0;
   out_2103422974771051523[15] = 0;
   out_2103422974771051523[16] = 0;
   out_2103422974771051523[17] = 0;
   out_2103422974771051523[18] = 0;
   out_2103422974771051523[19] = 0;
   out_2103422974771051523[20] = 0;
   out_2103422974771051523[21] = 0;
   out_2103422974771051523[22] = 1;
   out_2103422974771051523[23] = 0;
   out_2103422974771051523[24] = 0;
   out_2103422974771051523[25] = 0;
   out_2103422974771051523[26] = 0;
   out_2103422974771051523[27] = 0;
   out_2103422974771051523[28] = 0;
   out_2103422974771051523[29] = 0;
   out_2103422974771051523[30] = 0;
   out_2103422974771051523[31] = 0;
   out_2103422974771051523[32] = 0;
   out_2103422974771051523[33] = 0;
   out_2103422974771051523[34] = 0;
   out_2103422974771051523[35] = 0;
   out_2103422974771051523[36] = 0;
   out_2103422974771051523[37] = 0;
   out_2103422974771051523[38] = 0;
   out_2103422974771051523[39] = 0;
   out_2103422974771051523[40] = 0;
   out_2103422974771051523[41] = 1;
   out_2103422974771051523[42] = 0;
   out_2103422974771051523[43] = 0;
   out_2103422974771051523[44] = 0;
   out_2103422974771051523[45] = 0;
   out_2103422974771051523[46] = 0;
   out_2103422974771051523[47] = 0;
   out_2103422974771051523[48] = 0;
   out_2103422974771051523[49] = 0;
   out_2103422974771051523[50] = 0;
   out_2103422974771051523[51] = 0;
   out_2103422974771051523[52] = 0;
   out_2103422974771051523[53] = 0;
}
void h_14(double *state, double *unused, double *out_3663326241180753804) {
   out_3663326241180753804[0] = state[6];
   out_3663326241180753804[1] = state[7];
   out_3663326241180753804[2] = state[8];
}
void H_14(double *state, double *unused, double *out_1352455943763899795) {
   out_1352455943763899795[0] = 0;
   out_1352455943763899795[1] = 0;
   out_1352455943763899795[2] = 0;
   out_1352455943763899795[3] = 0;
   out_1352455943763899795[4] = 0;
   out_1352455943763899795[5] = 0;
   out_1352455943763899795[6] = 1;
   out_1352455943763899795[7] = 0;
   out_1352455943763899795[8] = 0;
   out_1352455943763899795[9] = 0;
   out_1352455943763899795[10] = 0;
   out_1352455943763899795[11] = 0;
   out_1352455943763899795[12] = 0;
   out_1352455943763899795[13] = 0;
   out_1352455943763899795[14] = 0;
   out_1352455943763899795[15] = 0;
   out_1352455943763899795[16] = 0;
   out_1352455943763899795[17] = 0;
   out_1352455943763899795[18] = 0;
   out_1352455943763899795[19] = 0;
   out_1352455943763899795[20] = 0;
   out_1352455943763899795[21] = 0;
   out_1352455943763899795[22] = 0;
   out_1352455943763899795[23] = 0;
   out_1352455943763899795[24] = 0;
   out_1352455943763899795[25] = 1;
   out_1352455943763899795[26] = 0;
   out_1352455943763899795[27] = 0;
   out_1352455943763899795[28] = 0;
   out_1352455943763899795[29] = 0;
   out_1352455943763899795[30] = 0;
   out_1352455943763899795[31] = 0;
   out_1352455943763899795[32] = 0;
   out_1352455943763899795[33] = 0;
   out_1352455943763899795[34] = 0;
   out_1352455943763899795[35] = 0;
   out_1352455943763899795[36] = 0;
   out_1352455943763899795[37] = 0;
   out_1352455943763899795[38] = 0;
   out_1352455943763899795[39] = 0;
   out_1352455943763899795[40] = 0;
   out_1352455943763899795[41] = 0;
   out_1352455943763899795[42] = 0;
   out_1352455943763899795[43] = 0;
   out_1352455943763899795[44] = 1;
   out_1352455943763899795[45] = 0;
   out_1352455943763899795[46] = 0;
   out_1352455943763899795[47] = 0;
   out_1352455943763899795[48] = 0;
   out_1352455943763899795[49] = 0;
   out_1352455943763899795[50] = 0;
   out_1352455943763899795[51] = 0;
   out_1352455943763899795[52] = 0;
   out_1352455943763899795[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_1527400177482295711) {
  err_fun(nom_x, delta_x, out_1527400177482295711);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7620805951199866590) {
  inv_err_fun(nom_x, true_x, out_7620805951199866590);
}
void pose_H_mod_fun(double *state, double *out_2041474721312435071) {
  H_mod_fun(state, out_2041474721312435071);
}
void pose_f_fun(double *state, double dt, double *out_3319995548118437592) {
  f_fun(state,  dt, out_3319995548118437592);
}
void pose_F_fun(double *state, double dt, double *out_1297884238983223646) {
  F_fun(state,  dt, out_1297884238983223646);
}
void pose_h_4(double *state, double *unused, double *out_8728651807226544018) {
  h_4(state, unused, out_8728651807226544018);
}
void pose_H_4(double *state, double *unused, double *out_5315696800103384324) {
  H_4(state, unused, out_5315696800103384324);
}
void pose_h_10(double *state, double *unused, double *out_3679201747496490736) {
  h_10(state, unused, out_3679201747496490736);
}
void pose_H_10(double *state, double *unused, double *out_2420796569664890088) {
  H_10(state, unused, out_2420796569664890088);
}
void pose_h_13(double *state, double *unused, double *out_4772104686591378688) {
  h_13(state, unused, out_4772104686591378688);
}
void pose_H_13(double *state, double *unused, double *out_2103422974771051523) {
  H_13(state, unused, out_2103422974771051523);
}
void pose_h_14(double *state, double *unused, double *out_3663326241180753804) {
  h_14(state, unused, out_3663326241180753804);
}
void pose_H_14(double *state, double *unused, double *out_1352455943763899795) {
  H_14(state, unused, out_1352455943763899795);
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

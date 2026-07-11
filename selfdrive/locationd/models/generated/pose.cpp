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
void err_fun(double *nom_x, double *delta_x, double *out_6464539647254934838) {
   out_6464539647254934838[0] = delta_x[0] + nom_x[0];
   out_6464539647254934838[1] = delta_x[1] + nom_x[1];
   out_6464539647254934838[2] = delta_x[2] + nom_x[2];
   out_6464539647254934838[3] = delta_x[3] + nom_x[3];
   out_6464539647254934838[4] = delta_x[4] + nom_x[4];
   out_6464539647254934838[5] = delta_x[5] + nom_x[5];
   out_6464539647254934838[6] = delta_x[6] + nom_x[6];
   out_6464539647254934838[7] = delta_x[7] + nom_x[7];
   out_6464539647254934838[8] = delta_x[8] + nom_x[8];
   out_6464539647254934838[9] = delta_x[9] + nom_x[9];
   out_6464539647254934838[10] = delta_x[10] + nom_x[10];
   out_6464539647254934838[11] = delta_x[11] + nom_x[11];
   out_6464539647254934838[12] = delta_x[12] + nom_x[12];
   out_6464539647254934838[13] = delta_x[13] + nom_x[13];
   out_6464539647254934838[14] = delta_x[14] + nom_x[14];
   out_6464539647254934838[15] = delta_x[15] + nom_x[15];
   out_6464539647254934838[16] = delta_x[16] + nom_x[16];
   out_6464539647254934838[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6486962015996802201) {
   out_6486962015996802201[0] = -nom_x[0] + true_x[0];
   out_6486962015996802201[1] = -nom_x[1] + true_x[1];
   out_6486962015996802201[2] = -nom_x[2] + true_x[2];
   out_6486962015996802201[3] = -nom_x[3] + true_x[3];
   out_6486962015996802201[4] = -nom_x[4] + true_x[4];
   out_6486962015996802201[5] = -nom_x[5] + true_x[5];
   out_6486962015996802201[6] = -nom_x[6] + true_x[6];
   out_6486962015996802201[7] = -nom_x[7] + true_x[7];
   out_6486962015996802201[8] = -nom_x[8] + true_x[8];
   out_6486962015996802201[9] = -nom_x[9] + true_x[9];
   out_6486962015996802201[10] = -nom_x[10] + true_x[10];
   out_6486962015996802201[11] = -nom_x[11] + true_x[11];
   out_6486962015996802201[12] = -nom_x[12] + true_x[12];
   out_6486962015996802201[13] = -nom_x[13] + true_x[13];
   out_6486962015996802201[14] = -nom_x[14] + true_x[14];
   out_6486962015996802201[15] = -nom_x[15] + true_x[15];
   out_6486962015996802201[16] = -nom_x[16] + true_x[16];
   out_6486962015996802201[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_6072805508798864108) {
   out_6072805508798864108[0] = 1.0;
   out_6072805508798864108[1] = 0.0;
   out_6072805508798864108[2] = 0.0;
   out_6072805508798864108[3] = 0.0;
   out_6072805508798864108[4] = 0.0;
   out_6072805508798864108[5] = 0.0;
   out_6072805508798864108[6] = 0.0;
   out_6072805508798864108[7] = 0.0;
   out_6072805508798864108[8] = 0.0;
   out_6072805508798864108[9] = 0.0;
   out_6072805508798864108[10] = 0.0;
   out_6072805508798864108[11] = 0.0;
   out_6072805508798864108[12] = 0.0;
   out_6072805508798864108[13] = 0.0;
   out_6072805508798864108[14] = 0.0;
   out_6072805508798864108[15] = 0.0;
   out_6072805508798864108[16] = 0.0;
   out_6072805508798864108[17] = 0.0;
   out_6072805508798864108[18] = 0.0;
   out_6072805508798864108[19] = 1.0;
   out_6072805508798864108[20] = 0.0;
   out_6072805508798864108[21] = 0.0;
   out_6072805508798864108[22] = 0.0;
   out_6072805508798864108[23] = 0.0;
   out_6072805508798864108[24] = 0.0;
   out_6072805508798864108[25] = 0.0;
   out_6072805508798864108[26] = 0.0;
   out_6072805508798864108[27] = 0.0;
   out_6072805508798864108[28] = 0.0;
   out_6072805508798864108[29] = 0.0;
   out_6072805508798864108[30] = 0.0;
   out_6072805508798864108[31] = 0.0;
   out_6072805508798864108[32] = 0.0;
   out_6072805508798864108[33] = 0.0;
   out_6072805508798864108[34] = 0.0;
   out_6072805508798864108[35] = 0.0;
   out_6072805508798864108[36] = 0.0;
   out_6072805508798864108[37] = 0.0;
   out_6072805508798864108[38] = 1.0;
   out_6072805508798864108[39] = 0.0;
   out_6072805508798864108[40] = 0.0;
   out_6072805508798864108[41] = 0.0;
   out_6072805508798864108[42] = 0.0;
   out_6072805508798864108[43] = 0.0;
   out_6072805508798864108[44] = 0.0;
   out_6072805508798864108[45] = 0.0;
   out_6072805508798864108[46] = 0.0;
   out_6072805508798864108[47] = 0.0;
   out_6072805508798864108[48] = 0.0;
   out_6072805508798864108[49] = 0.0;
   out_6072805508798864108[50] = 0.0;
   out_6072805508798864108[51] = 0.0;
   out_6072805508798864108[52] = 0.0;
   out_6072805508798864108[53] = 0.0;
   out_6072805508798864108[54] = 0.0;
   out_6072805508798864108[55] = 0.0;
   out_6072805508798864108[56] = 0.0;
   out_6072805508798864108[57] = 1.0;
   out_6072805508798864108[58] = 0.0;
   out_6072805508798864108[59] = 0.0;
   out_6072805508798864108[60] = 0.0;
   out_6072805508798864108[61] = 0.0;
   out_6072805508798864108[62] = 0.0;
   out_6072805508798864108[63] = 0.0;
   out_6072805508798864108[64] = 0.0;
   out_6072805508798864108[65] = 0.0;
   out_6072805508798864108[66] = 0.0;
   out_6072805508798864108[67] = 0.0;
   out_6072805508798864108[68] = 0.0;
   out_6072805508798864108[69] = 0.0;
   out_6072805508798864108[70] = 0.0;
   out_6072805508798864108[71] = 0.0;
   out_6072805508798864108[72] = 0.0;
   out_6072805508798864108[73] = 0.0;
   out_6072805508798864108[74] = 0.0;
   out_6072805508798864108[75] = 0.0;
   out_6072805508798864108[76] = 1.0;
   out_6072805508798864108[77] = 0.0;
   out_6072805508798864108[78] = 0.0;
   out_6072805508798864108[79] = 0.0;
   out_6072805508798864108[80] = 0.0;
   out_6072805508798864108[81] = 0.0;
   out_6072805508798864108[82] = 0.0;
   out_6072805508798864108[83] = 0.0;
   out_6072805508798864108[84] = 0.0;
   out_6072805508798864108[85] = 0.0;
   out_6072805508798864108[86] = 0.0;
   out_6072805508798864108[87] = 0.0;
   out_6072805508798864108[88] = 0.0;
   out_6072805508798864108[89] = 0.0;
   out_6072805508798864108[90] = 0.0;
   out_6072805508798864108[91] = 0.0;
   out_6072805508798864108[92] = 0.0;
   out_6072805508798864108[93] = 0.0;
   out_6072805508798864108[94] = 0.0;
   out_6072805508798864108[95] = 1.0;
   out_6072805508798864108[96] = 0.0;
   out_6072805508798864108[97] = 0.0;
   out_6072805508798864108[98] = 0.0;
   out_6072805508798864108[99] = 0.0;
   out_6072805508798864108[100] = 0.0;
   out_6072805508798864108[101] = 0.0;
   out_6072805508798864108[102] = 0.0;
   out_6072805508798864108[103] = 0.0;
   out_6072805508798864108[104] = 0.0;
   out_6072805508798864108[105] = 0.0;
   out_6072805508798864108[106] = 0.0;
   out_6072805508798864108[107] = 0.0;
   out_6072805508798864108[108] = 0.0;
   out_6072805508798864108[109] = 0.0;
   out_6072805508798864108[110] = 0.0;
   out_6072805508798864108[111] = 0.0;
   out_6072805508798864108[112] = 0.0;
   out_6072805508798864108[113] = 0.0;
   out_6072805508798864108[114] = 1.0;
   out_6072805508798864108[115] = 0.0;
   out_6072805508798864108[116] = 0.0;
   out_6072805508798864108[117] = 0.0;
   out_6072805508798864108[118] = 0.0;
   out_6072805508798864108[119] = 0.0;
   out_6072805508798864108[120] = 0.0;
   out_6072805508798864108[121] = 0.0;
   out_6072805508798864108[122] = 0.0;
   out_6072805508798864108[123] = 0.0;
   out_6072805508798864108[124] = 0.0;
   out_6072805508798864108[125] = 0.0;
   out_6072805508798864108[126] = 0.0;
   out_6072805508798864108[127] = 0.0;
   out_6072805508798864108[128] = 0.0;
   out_6072805508798864108[129] = 0.0;
   out_6072805508798864108[130] = 0.0;
   out_6072805508798864108[131] = 0.0;
   out_6072805508798864108[132] = 0.0;
   out_6072805508798864108[133] = 1.0;
   out_6072805508798864108[134] = 0.0;
   out_6072805508798864108[135] = 0.0;
   out_6072805508798864108[136] = 0.0;
   out_6072805508798864108[137] = 0.0;
   out_6072805508798864108[138] = 0.0;
   out_6072805508798864108[139] = 0.0;
   out_6072805508798864108[140] = 0.0;
   out_6072805508798864108[141] = 0.0;
   out_6072805508798864108[142] = 0.0;
   out_6072805508798864108[143] = 0.0;
   out_6072805508798864108[144] = 0.0;
   out_6072805508798864108[145] = 0.0;
   out_6072805508798864108[146] = 0.0;
   out_6072805508798864108[147] = 0.0;
   out_6072805508798864108[148] = 0.0;
   out_6072805508798864108[149] = 0.0;
   out_6072805508798864108[150] = 0.0;
   out_6072805508798864108[151] = 0.0;
   out_6072805508798864108[152] = 1.0;
   out_6072805508798864108[153] = 0.0;
   out_6072805508798864108[154] = 0.0;
   out_6072805508798864108[155] = 0.0;
   out_6072805508798864108[156] = 0.0;
   out_6072805508798864108[157] = 0.0;
   out_6072805508798864108[158] = 0.0;
   out_6072805508798864108[159] = 0.0;
   out_6072805508798864108[160] = 0.0;
   out_6072805508798864108[161] = 0.0;
   out_6072805508798864108[162] = 0.0;
   out_6072805508798864108[163] = 0.0;
   out_6072805508798864108[164] = 0.0;
   out_6072805508798864108[165] = 0.0;
   out_6072805508798864108[166] = 0.0;
   out_6072805508798864108[167] = 0.0;
   out_6072805508798864108[168] = 0.0;
   out_6072805508798864108[169] = 0.0;
   out_6072805508798864108[170] = 0.0;
   out_6072805508798864108[171] = 1.0;
   out_6072805508798864108[172] = 0.0;
   out_6072805508798864108[173] = 0.0;
   out_6072805508798864108[174] = 0.0;
   out_6072805508798864108[175] = 0.0;
   out_6072805508798864108[176] = 0.0;
   out_6072805508798864108[177] = 0.0;
   out_6072805508798864108[178] = 0.0;
   out_6072805508798864108[179] = 0.0;
   out_6072805508798864108[180] = 0.0;
   out_6072805508798864108[181] = 0.0;
   out_6072805508798864108[182] = 0.0;
   out_6072805508798864108[183] = 0.0;
   out_6072805508798864108[184] = 0.0;
   out_6072805508798864108[185] = 0.0;
   out_6072805508798864108[186] = 0.0;
   out_6072805508798864108[187] = 0.0;
   out_6072805508798864108[188] = 0.0;
   out_6072805508798864108[189] = 0.0;
   out_6072805508798864108[190] = 1.0;
   out_6072805508798864108[191] = 0.0;
   out_6072805508798864108[192] = 0.0;
   out_6072805508798864108[193] = 0.0;
   out_6072805508798864108[194] = 0.0;
   out_6072805508798864108[195] = 0.0;
   out_6072805508798864108[196] = 0.0;
   out_6072805508798864108[197] = 0.0;
   out_6072805508798864108[198] = 0.0;
   out_6072805508798864108[199] = 0.0;
   out_6072805508798864108[200] = 0.0;
   out_6072805508798864108[201] = 0.0;
   out_6072805508798864108[202] = 0.0;
   out_6072805508798864108[203] = 0.0;
   out_6072805508798864108[204] = 0.0;
   out_6072805508798864108[205] = 0.0;
   out_6072805508798864108[206] = 0.0;
   out_6072805508798864108[207] = 0.0;
   out_6072805508798864108[208] = 0.0;
   out_6072805508798864108[209] = 1.0;
   out_6072805508798864108[210] = 0.0;
   out_6072805508798864108[211] = 0.0;
   out_6072805508798864108[212] = 0.0;
   out_6072805508798864108[213] = 0.0;
   out_6072805508798864108[214] = 0.0;
   out_6072805508798864108[215] = 0.0;
   out_6072805508798864108[216] = 0.0;
   out_6072805508798864108[217] = 0.0;
   out_6072805508798864108[218] = 0.0;
   out_6072805508798864108[219] = 0.0;
   out_6072805508798864108[220] = 0.0;
   out_6072805508798864108[221] = 0.0;
   out_6072805508798864108[222] = 0.0;
   out_6072805508798864108[223] = 0.0;
   out_6072805508798864108[224] = 0.0;
   out_6072805508798864108[225] = 0.0;
   out_6072805508798864108[226] = 0.0;
   out_6072805508798864108[227] = 0.0;
   out_6072805508798864108[228] = 1.0;
   out_6072805508798864108[229] = 0.0;
   out_6072805508798864108[230] = 0.0;
   out_6072805508798864108[231] = 0.0;
   out_6072805508798864108[232] = 0.0;
   out_6072805508798864108[233] = 0.0;
   out_6072805508798864108[234] = 0.0;
   out_6072805508798864108[235] = 0.0;
   out_6072805508798864108[236] = 0.0;
   out_6072805508798864108[237] = 0.0;
   out_6072805508798864108[238] = 0.0;
   out_6072805508798864108[239] = 0.0;
   out_6072805508798864108[240] = 0.0;
   out_6072805508798864108[241] = 0.0;
   out_6072805508798864108[242] = 0.0;
   out_6072805508798864108[243] = 0.0;
   out_6072805508798864108[244] = 0.0;
   out_6072805508798864108[245] = 0.0;
   out_6072805508798864108[246] = 0.0;
   out_6072805508798864108[247] = 1.0;
   out_6072805508798864108[248] = 0.0;
   out_6072805508798864108[249] = 0.0;
   out_6072805508798864108[250] = 0.0;
   out_6072805508798864108[251] = 0.0;
   out_6072805508798864108[252] = 0.0;
   out_6072805508798864108[253] = 0.0;
   out_6072805508798864108[254] = 0.0;
   out_6072805508798864108[255] = 0.0;
   out_6072805508798864108[256] = 0.0;
   out_6072805508798864108[257] = 0.0;
   out_6072805508798864108[258] = 0.0;
   out_6072805508798864108[259] = 0.0;
   out_6072805508798864108[260] = 0.0;
   out_6072805508798864108[261] = 0.0;
   out_6072805508798864108[262] = 0.0;
   out_6072805508798864108[263] = 0.0;
   out_6072805508798864108[264] = 0.0;
   out_6072805508798864108[265] = 0.0;
   out_6072805508798864108[266] = 1.0;
   out_6072805508798864108[267] = 0.0;
   out_6072805508798864108[268] = 0.0;
   out_6072805508798864108[269] = 0.0;
   out_6072805508798864108[270] = 0.0;
   out_6072805508798864108[271] = 0.0;
   out_6072805508798864108[272] = 0.0;
   out_6072805508798864108[273] = 0.0;
   out_6072805508798864108[274] = 0.0;
   out_6072805508798864108[275] = 0.0;
   out_6072805508798864108[276] = 0.0;
   out_6072805508798864108[277] = 0.0;
   out_6072805508798864108[278] = 0.0;
   out_6072805508798864108[279] = 0.0;
   out_6072805508798864108[280] = 0.0;
   out_6072805508798864108[281] = 0.0;
   out_6072805508798864108[282] = 0.0;
   out_6072805508798864108[283] = 0.0;
   out_6072805508798864108[284] = 0.0;
   out_6072805508798864108[285] = 1.0;
   out_6072805508798864108[286] = 0.0;
   out_6072805508798864108[287] = 0.0;
   out_6072805508798864108[288] = 0.0;
   out_6072805508798864108[289] = 0.0;
   out_6072805508798864108[290] = 0.0;
   out_6072805508798864108[291] = 0.0;
   out_6072805508798864108[292] = 0.0;
   out_6072805508798864108[293] = 0.0;
   out_6072805508798864108[294] = 0.0;
   out_6072805508798864108[295] = 0.0;
   out_6072805508798864108[296] = 0.0;
   out_6072805508798864108[297] = 0.0;
   out_6072805508798864108[298] = 0.0;
   out_6072805508798864108[299] = 0.0;
   out_6072805508798864108[300] = 0.0;
   out_6072805508798864108[301] = 0.0;
   out_6072805508798864108[302] = 0.0;
   out_6072805508798864108[303] = 0.0;
   out_6072805508798864108[304] = 1.0;
   out_6072805508798864108[305] = 0.0;
   out_6072805508798864108[306] = 0.0;
   out_6072805508798864108[307] = 0.0;
   out_6072805508798864108[308] = 0.0;
   out_6072805508798864108[309] = 0.0;
   out_6072805508798864108[310] = 0.0;
   out_6072805508798864108[311] = 0.0;
   out_6072805508798864108[312] = 0.0;
   out_6072805508798864108[313] = 0.0;
   out_6072805508798864108[314] = 0.0;
   out_6072805508798864108[315] = 0.0;
   out_6072805508798864108[316] = 0.0;
   out_6072805508798864108[317] = 0.0;
   out_6072805508798864108[318] = 0.0;
   out_6072805508798864108[319] = 0.0;
   out_6072805508798864108[320] = 0.0;
   out_6072805508798864108[321] = 0.0;
   out_6072805508798864108[322] = 0.0;
   out_6072805508798864108[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_5954902381465487783) {
   out_5954902381465487783[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_5954902381465487783[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_5954902381465487783[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_5954902381465487783[3] = dt*state[12] + state[3];
   out_5954902381465487783[4] = dt*state[13] + state[4];
   out_5954902381465487783[5] = dt*state[14] + state[5];
   out_5954902381465487783[6] = state[6];
   out_5954902381465487783[7] = state[7];
   out_5954902381465487783[8] = state[8];
   out_5954902381465487783[9] = state[9];
   out_5954902381465487783[10] = state[10];
   out_5954902381465487783[11] = state[11];
   out_5954902381465487783[12] = state[12];
   out_5954902381465487783[13] = state[13];
   out_5954902381465487783[14] = state[14];
   out_5954902381465487783[15] = state[15];
   out_5954902381465487783[16] = state[16];
   out_5954902381465487783[17] = state[17];
}
void F_fun(double *state, double dt, double *out_3606916503598787696) {
   out_3606916503598787696[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3606916503598787696[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3606916503598787696[2] = 0;
   out_3606916503598787696[3] = 0;
   out_3606916503598787696[4] = 0;
   out_3606916503598787696[5] = 0;
   out_3606916503598787696[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3606916503598787696[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3606916503598787696[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3606916503598787696[9] = 0;
   out_3606916503598787696[10] = 0;
   out_3606916503598787696[11] = 0;
   out_3606916503598787696[12] = 0;
   out_3606916503598787696[13] = 0;
   out_3606916503598787696[14] = 0;
   out_3606916503598787696[15] = 0;
   out_3606916503598787696[16] = 0;
   out_3606916503598787696[17] = 0;
   out_3606916503598787696[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3606916503598787696[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3606916503598787696[20] = 0;
   out_3606916503598787696[21] = 0;
   out_3606916503598787696[22] = 0;
   out_3606916503598787696[23] = 0;
   out_3606916503598787696[24] = 0;
   out_3606916503598787696[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3606916503598787696[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3606916503598787696[27] = 0;
   out_3606916503598787696[28] = 0;
   out_3606916503598787696[29] = 0;
   out_3606916503598787696[30] = 0;
   out_3606916503598787696[31] = 0;
   out_3606916503598787696[32] = 0;
   out_3606916503598787696[33] = 0;
   out_3606916503598787696[34] = 0;
   out_3606916503598787696[35] = 0;
   out_3606916503598787696[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3606916503598787696[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3606916503598787696[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3606916503598787696[39] = 0;
   out_3606916503598787696[40] = 0;
   out_3606916503598787696[41] = 0;
   out_3606916503598787696[42] = 0;
   out_3606916503598787696[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3606916503598787696[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3606916503598787696[45] = 0;
   out_3606916503598787696[46] = 0;
   out_3606916503598787696[47] = 0;
   out_3606916503598787696[48] = 0;
   out_3606916503598787696[49] = 0;
   out_3606916503598787696[50] = 0;
   out_3606916503598787696[51] = 0;
   out_3606916503598787696[52] = 0;
   out_3606916503598787696[53] = 0;
   out_3606916503598787696[54] = 0;
   out_3606916503598787696[55] = 0;
   out_3606916503598787696[56] = 0;
   out_3606916503598787696[57] = 1;
   out_3606916503598787696[58] = 0;
   out_3606916503598787696[59] = 0;
   out_3606916503598787696[60] = 0;
   out_3606916503598787696[61] = 0;
   out_3606916503598787696[62] = 0;
   out_3606916503598787696[63] = 0;
   out_3606916503598787696[64] = 0;
   out_3606916503598787696[65] = 0;
   out_3606916503598787696[66] = dt;
   out_3606916503598787696[67] = 0;
   out_3606916503598787696[68] = 0;
   out_3606916503598787696[69] = 0;
   out_3606916503598787696[70] = 0;
   out_3606916503598787696[71] = 0;
   out_3606916503598787696[72] = 0;
   out_3606916503598787696[73] = 0;
   out_3606916503598787696[74] = 0;
   out_3606916503598787696[75] = 0;
   out_3606916503598787696[76] = 1;
   out_3606916503598787696[77] = 0;
   out_3606916503598787696[78] = 0;
   out_3606916503598787696[79] = 0;
   out_3606916503598787696[80] = 0;
   out_3606916503598787696[81] = 0;
   out_3606916503598787696[82] = 0;
   out_3606916503598787696[83] = 0;
   out_3606916503598787696[84] = 0;
   out_3606916503598787696[85] = dt;
   out_3606916503598787696[86] = 0;
   out_3606916503598787696[87] = 0;
   out_3606916503598787696[88] = 0;
   out_3606916503598787696[89] = 0;
   out_3606916503598787696[90] = 0;
   out_3606916503598787696[91] = 0;
   out_3606916503598787696[92] = 0;
   out_3606916503598787696[93] = 0;
   out_3606916503598787696[94] = 0;
   out_3606916503598787696[95] = 1;
   out_3606916503598787696[96] = 0;
   out_3606916503598787696[97] = 0;
   out_3606916503598787696[98] = 0;
   out_3606916503598787696[99] = 0;
   out_3606916503598787696[100] = 0;
   out_3606916503598787696[101] = 0;
   out_3606916503598787696[102] = 0;
   out_3606916503598787696[103] = 0;
   out_3606916503598787696[104] = dt;
   out_3606916503598787696[105] = 0;
   out_3606916503598787696[106] = 0;
   out_3606916503598787696[107] = 0;
   out_3606916503598787696[108] = 0;
   out_3606916503598787696[109] = 0;
   out_3606916503598787696[110] = 0;
   out_3606916503598787696[111] = 0;
   out_3606916503598787696[112] = 0;
   out_3606916503598787696[113] = 0;
   out_3606916503598787696[114] = 1;
   out_3606916503598787696[115] = 0;
   out_3606916503598787696[116] = 0;
   out_3606916503598787696[117] = 0;
   out_3606916503598787696[118] = 0;
   out_3606916503598787696[119] = 0;
   out_3606916503598787696[120] = 0;
   out_3606916503598787696[121] = 0;
   out_3606916503598787696[122] = 0;
   out_3606916503598787696[123] = 0;
   out_3606916503598787696[124] = 0;
   out_3606916503598787696[125] = 0;
   out_3606916503598787696[126] = 0;
   out_3606916503598787696[127] = 0;
   out_3606916503598787696[128] = 0;
   out_3606916503598787696[129] = 0;
   out_3606916503598787696[130] = 0;
   out_3606916503598787696[131] = 0;
   out_3606916503598787696[132] = 0;
   out_3606916503598787696[133] = 1;
   out_3606916503598787696[134] = 0;
   out_3606916503598787696[135] = 0;
   out_3606916503598787696[136] = 0;
   out_3606916503598787696[137] = 0;
   out_3606916503598787696[138] = 0;
   out_3606916503598787696[139] = 0;
   out_3606916503598787696[140] = 0;
   out_3606916503598787696[141] = 0;
   out_3606916503598787696[142] = 0;
   out_3606916503598787696[143] = 0;
   out_3606916503598787696[144] = 0;
   out_3606916503598787696[145] = 0;
   out_3606916503598787696[146] = 0;
   out_3606916503598787696[147] = 0;
   out_3606916503598787696[148] = 0;
   out_3606916503598787696[149] = 0;
   out_3606916503598787696[150] = 0;
   out_3606916503598787696[151] = 0;
   out_3606916503598787696[152] = 1;
   out_3606916503598787696[153] = 0;
   out_3606916503598787696[154] = 0;
   out_3606916503598787696[155] = 0;
   out_3606916503598787696[156] = 0;
   out_3606916503598787696[157] = 0;
   out_3606916503598787696[158] = 0;
   out_3606916503598787696[159] = 0;
   out_3606916503598787696[160] = 0;
   out_3606916503598787696[161] = 0;
   out_3606916503598787696[162] = 0;
   out_3606916503598787696[163] = 0;
   out_3606916503598787696[164] = 0;
   out_3606916503598787696[165] = 0;
   out_3606916503598787696[166] = 0;
   out_3606916503598787696[167] = 0;
   out_3606916503598787696[168] = 0;
   out_3606916503598787696[169] = 0;
   out_3606916503598787696[170] = 0;
   out_3606916503598787696[171] = 1;
   out_3606916503598787696[172] = 0;
   out_3606916503598787696[173] = 0;
   out_3606916503598787696[174] = 0;
   out_3606916503598787696[175] = 0;
   out_3606916503598787696[176] = 0;
   out_3606916503598787696[177] = 0;
   out_3606916503598787696[178] = 0;
   out_3606916503598787696[179] = 0;
   out_3606916503598787696[180] = 0;
   out_3606916503598787696[181] = 0;
   out_3606916503598787696[182] = 0;
   out_3606916503598787696[183] = 0;
   out_3606916503598787696[184] = 0;
   out_3606916503598787696[185] = 0;
   out_3606916503598787696[186] = 0;
   out_3606916503598787696[187] = 0;
   out_3606916503598787696[188] = 0;
   out_3606916503598787696[189] = 0;
   out_3606916503598787696[190] = 1;
   out_3606916503598787696[191] = 0;
   out_3606916503598787696[192] = 0;
   out_3606916503598787696[193] = 0;
   out_3606916503598787696[194] = 0;
   out_3606916503598787696[195] = 0;
   out_3606916503598787696[196] = 0;
   out_3606916503598787696[197] = 0;
   out_3606916503598787696[198] = 0;
   out_3606916503598787696[199] = 0;
   out_3606916503598787696[200] = 0;
   out_3606916503598787696[201] = 0;
   out_3606916503598787696[202] = 0;
   out_3606916503598787696[203] = 0;
   out_3606916503598787696[204] = 0;
   out_3606916503598787696[205] = 0;
   out_3606916503598787696[206] = 0;
   out_3606916503598787696[207] = 0;
   out_3606916503598787696[208] = 0;
   out_3606916503598787696[209] = 1;
   out_3606916503598787696[210] = 0;
   out_3606916503598787696[211] = 0;
   out_3606916503598787696[212] = 0;
   out_3606916503598787696[213] = 0;
   out_3606916503598787696[214] = 0;
   out_3606916503598787696[215] = 0;
   out_3606916503598787696[216] = 0;
   out_3606916503598787696[217] = 0;
   out_3606916503598787696[218] = 0;
   out_3606916503598787696[219] = 0;
   out_3606916503598787696[220] = 0;
   out_3606916503598787696[221] = 0;
   out_3606916503598787696[222] = 0;
   out_3606916503598787696[223] = 0;
   out_3606916503598787696[224] = 0;
   out_3606916503598787696[225] = 0;
   out_3606916503598787696[226] = 0;
   out_3606916503598787696[227] = 0;
   out_3606916503598787696[228] = 1;
   out_3606916503598787696[229] = 0;
   out_3606916503598787696[230] = 0;
   out_3606916503598787696[231] = 0;
   out_3606916503598787696[232] = 0;
   out_3606916503598787696[233] = 0;
   out_3606916503598787696[234] = 0;
   out_3606916503598787696[235] = 0;
   out_3606916503598787696[236] = 0;
   out_3606916503598787696[237] = 0;
   out_3606916503598787696[238] = 0;
   out_3606916503598787696[239] = 0;
   out_3606916503598787696[240] = 0;
   out_3606916503598787696[241] = 0;
   out_3606916503598787696[242] = 0;
   out_3606916503598787696[243] = 0;
   out_3606916503598787696[244] = 0;
   out_3606916503598787696[245] = 0;
   out_3606916503598787696[246] = 0;
   out_3606916503598787696[247] = 1;
   out_3606916503598787696[248] = 0;
   out_3606916503598787696[249] = 0;
   out_3606916503598787696[250] = 0;
   out_3606916503598787696[251] = 0;
   out_3606916503598787696[252] = 0;
   out_3606916503598787696[253] = 0;
   out_3606916503598787696[254] = 0;
   out_3606916503598787696[255] = 0;
   out_3606916503598787696[256] = 0;
   out_3606916503598787696[257] = 0;
   out_3606916503598787696[258] = 0;
   out_3606916503598787696[259] = 0;
   out_3606916503598787696[260] = 0;
   out_3606916503598787696[261] = 0;
   out_3606916503598787696[262] = 0;
   out_3606916503598787696[263] = 0;
   out_3606916503598787696[264] = 0;
   out_3606916503598787696[265] = 0;
   out_3606916503598787696[266] = 1;
   out_3606916503598787696[267] = 0;
   out_3606916503598787696[268] = 0;
   out_3606916503598787696[269] = 0;
   out_3606916503598787696[270] = 0;
   out_3606916503598787696[271] = 0;
   out_3606916503598787696[272] = 0;
   out_3606916503598787696[273] = 0;
   out_3606916503598787696[274] = 0;
   out_3606916503598787696[275] = 0;
   out_3606916503598787696[276] = 0;
   out_3606916503598787696[277] = 0;
   out_3606916503598787696[278] = 0;
   out_3606916503598787696[279] = 0;
   out_3606916503598787696[280] = 0;
   out_3606916503598787696[281] = 0;
   out_3606916503598787696[282] = 0;
   out_3606916503598787696[283] = 0;
   out_3606916503598787696[284] = 0;
   out_3606916503598787696[285] = 1;
   out_3606916503598787696[286] = 0;
   out_3606916503598787696[287] = 0;
   out_3606916503598787696[288] = 0;
   out_3606916503598787696[289] = 0;
   out_3606916503598787696[290] = 0;
   out_3606916503598787696[291] = 0;
   out_3606916503598787696[292] = 0;
   out_3606916503598787696[293] = 0;
   out_3606916503598787696[294] = 0;
   out_3606916503598787696[295] = 0;
   out_3606916503598787696[296] = 0;
   out_3606916503598787696[297] = 0;
   out_3606916503598787696[298] = 0;
   out_3606916503598787696[299] = 0;
   out_3606916503598787696[300] = 0;
   out_3606916503598787696[301] = 0;
   out_3606916503598787696[302] = 0;
   out_3606916503598787696[303] = 0;
   out_3606916503598787696[304] = 1;
   out_3606916503598787696[305] = 0;
   out_3606916503598787696[306] = 0;
   out_3606916503598787696[307] = 0;
   out_3606916503598787696[308] = 0;
   out_3606916503598787696[309] = 0;
   out_3606916503598787696[310] = 0;
   out_3606916503598787696[311] = 0;
   out_3606916503598787696[312] = 0;
   out_3606916503598787696[313] = 0;
   out_3606916503598787696[314] = 0;
   out_3606916503598787696[315] = 0;
   out_3606916503598787696[316] = 0;
   out_3606916503598787696[317] = 0;
   out_3606916503598787696[318] = 0;
   out_3606916503598787696[319] = 0;
   out_3606916503598787696[320] = 0;
   out_3606916503598787696[321] = 0;
   out_3606916503598787696[322] = 0;
   out_3606916503598787696[323] = 1;
}
void h_4(double *state, double *unused, double *out_5987237266806342619) {
   out_5987237266806342619[0] = state[6] + state[9];
   out_5987237266806342619[1] = state[7] + state[10];
   out_5987237266806342619[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_2798583430007914855) {
   out_2798583430007914855[0] = 0;
   out_2798583430007914855[1] = 0;
   out_2798583430007914855[2] = 0;
   out_2798583430007914855[3] = 0;
   out_2798583430007914855[4] = 0;
   out_2798583430007914855[5] = 0;
   out_2798583430007914855[6] = 1;
   out_2798583430007914855[7] = 0;
   out_2798583430007914855[8] = 0;
   out_2798583430007914855[9] = 1;
   out_2798583430007914855[10] = 0;
   out_2798583430007914855[11] = 0;
   out_2798583430007914855[12] = 0;
   out_2798583430007914855[13] = 0;
   out_2798583430007914855[14] = 0;
   out_2798583430007914855[15] = 0;
   out_2798583430007914855[16] = 0;
   out_2798583430007914855[17] = 0;
   out_2798583430007914855[18] = 0;
   out_2798583430007914855[19] = 0;
   out_2798583430007914855[20] = 0;
   out_2798583430007914855[21] = 0;
   out_2798583430007914855[22] = 0;
   out_2798583430007914855[23] = 0;
   out_2798583430007914855[24] = 0;
   out_2798583430007914855[25] = 1;
   out_2798583430007914855[26] = 0;
   out_2798583430007914855[27] = 0;
   out_2798583430007914855[28] = 1;
   out_2798583430007914855[29] = 0;
   out_2798583430007914855[30] = 0;
   out_2798583430007914855[31] = 0;
   out_2798583430007914855[32] = 0;
   out_2798583430007914855[33] = 0;
   out_2798583430007914855[34] = 0;
   out_2798583430007914855[35] = 0;
   out_2798583430007914855[36] = 0;
   out_2798583430007914855[37] = 0;
   out_2798583430007914855[38] = 0;
   out_2798583430007914855[39] = 0;
   out_2798583430007914855[40] = 0;
   out_2798583430007914855[41] = 0;
   out_2798583430007914855[42] = 0;
   out_2798583430007914855[43] = 0;
   out_2798583430007914855[44] = 1;
   out_2798583430007914855[45] = 0;
   out_2798583430007914855[46] = 0;
   out_2798583430007914855[47] = 1;
   out_2798583430007914855[48] = 0;
   out_2798583430007914855[49] = 0;
   out_2798583430007914855[50] = 0;
   out_2798583430007914855[51] = 0;
   out_2798583430007914855[52] = 0;
   out_2798583430007914855[53] = 0;
}
void h_10(double *state, double *unused, double *out_3025597584677695987) {
   out_3025597584677695987[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_3025597584677695987[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_3025597584677695987[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_1745173860280422879) {
   out_1745173860280422879[0] = 0;
   out_1745173860280422879[1] = 9.8100000000000005*cos(state[1]);
   out_1745173860280422879[2] = 0;
   out_1745173860280422879[3] = 0;
   out_1745173860280422879[4] = -state[8];
   out_1745173860280422879[5] = state[7];
   out_1745173860280422879[6] = 0;
   out_1745173860280422879[7] = state[5];
   out_1745173860280422879[8] = -state[4];
   out_1745173860280422879[9] = 0;
   out_1745173860280422879[10] = 0;
   out_1745173860280422879[11] = 0;
   out_1745173860280422879[12] = 1;
   out_1745173860280422879[13] = 0;
   out_1745173860280422879[14] = 0;
   out_1745173860280422879[15] = 1;
   out_1745173860280422879[16] = 0;
   out_1745173860280422879[17] = 0;
   out_1745173860280422879[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_1745173860280422879[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_1745173860280422879[20] = 0;
   out_1745173860280422879[21] = state[8];
   out_1745173860280422879[22] = 0;
   out_1745173860280422879[23] = -state[6];
   out_1745173860280422879[24] = -state[5];
   out_1745173860280422879[25] = 0;
   out_1745173860280422879[26] = state[3];
   out_1745173860280422879[27] = 0;
   out_1745173860280422879[28] = 0;
   out_1745173860280422879[29] = 0;
   out_1745173860280422879[30] = 0;
   out_1745173860280422879[31] = 1;
   out_1745173860280422879[32] = 0;
   out_1745173860280422879[33] = 0;
   out_1745173860280422879[34] = 1;
   out_1745173860280422879[35] = 0;
   out_1745173860280422879[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_1745173860280422879[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_1745173860280422879[38] = 0;
   out_1745173860280422879[39] = -state[7];
   out_1745173860280422879[40] = state[6];
   out_1745173860280422879[41] = 0;
   out_1745173860280422879[42] = state[4];
   out_1745173860280422879[43] = -state[3];
   out_1745173860280422879[44] = 0;
   out_1745173860280422879[45] = 0;
   out_1745173860280422879[46] = 0;
   out_1745173860280422879[47] = 0;
   out_1745173860280422879[48] = 0;
   out_1745173860280422879[49] = 0;
   out_1745173860280422879[50] = 1;
   out_1745173860280422879[51] = 0;
   out_1745173860280422879[52] = 0;
   out_1745173860280422879[53] = 1;
}
void h_13(double *state, double *unused, double *out_8599707273883010977) {
   out_8599707273883010977[0] = state[3];
   out_8599707273883010977[1] = state[4];
   out_8599707273883010977[2] = state[5];
}
void H_13(double *state, double *unused, double *out_8037529435384935832) {
   out_8037529435384935832[0] = 0;
   out_8037529435384935832[1] = 0;
   out_8037529435384935832[2] = 0;
   out_8037529435384935832[3] = 1;
   out_8037529435384935832[4] = 0;
   out_8037529435384935832[5] = 0;
   out_8037529435384935832[6] = 0;
   out_8037529435384935832[7] = 0;
   out_8037529435384935832[8] = 0;
   out_8037529435384935832[9] = 0;
   out_8037529435384935832[10] = 0;
   out_8037529435384935832[11] = 0;
   out_8037529435384935832[12] = 0;
   out_8037529435384935832[13] = 0;
   out_8037529435384935832[14] = 0;
   out_8037529435384935832[15] = 0;
   out_8037529435384935832[16] = 0;
   out_8037529435384935832[17] = 0;
   out_8037529435384935832[18] = 0;
   out_8037529435384935832[19] = 0;
   out_8037529435384935832[20] = 0;
   out_8037529435384935832[21] = 0;
   out_8037529435384935832[22] = 1;
   out_8037529435384935832[23] = 0;
   out_8037529435384935832[24] = 0;
   out_8037529435384935832[25] = 0;
   out_8037529435384935832[26] = 0;
   out_8037529435384935832[27] = 0;
   out_8037529435384935832[28] = 0;
   out_8037529435384935832[29] = 0;
   out_8037529435384935832[30] = 0;
   out_8037529435384935832[31] = 0;
   out_8037529435384935832[32] = 0;
   out_8037529435384935832[33] = 0;
   out_8037529435384935832[34] = 0;
   out_8037529435384935832[35] = 0;
   out_8037529435384935832[36] = 0;
   out_8037529435384935832[37] = 0;
   out_8037529435384935832[38] = 0;
   out_8037529435384935832[39] = 0;
   out_8037529435384935832[40] = 0;
   out_8037529435384935832[41] = 1;
   out_8037529435384935832[42] = 0;
   out_8037529435384935832[43] = 0;
   out_8037529435384935832[44] = 0;
   out_8037529435384935832[45] = 0;
   out_8037529435384935832[46] = 0;
   out_8037529435384935832[47] = 0;
   out_8037529435384935832[48] = 0;
   out_8037529435384935832[49] = 0;
   out_8037529435384935832[50] = 0;
   out_8037529435384935832[51] = 0;
   out_8037529435384935832[52] = 0;
   out_8037529435384935832[53] = 0;
}
void h_14(double *state, double *unused, double *out_3899158584107184147) {
   out_3899158584107184147[0] = state[6];
   out_3899158584107184147[1] = state[7];
   out_3899158584107184147[2] = state[8];
}
void H_14(double *state, double *unused, double *out_6761824286347399384) {
   out_6761824286347399384[0] = 0;
   out_6761824286347399384[1] = 0;
   out_6761824286347399384[2] = 0;
   out_6761824286347399384[3] = 0;
   out_6761824286347399384[4] = 0;
   out_6761824286347399384[5] = 0;
   out_6761824286347399384[6] = 1;
   out_6761824286347399384[7] = 0;
   out_6761824286347399384[8] = 0;
   out_6761824286347399384[9] = 0;
   out_6761824286347399384[10] = 0;
   out_6761824286347399384[11] = 0;
   out_6761824286347399384[12] = 0;
   out_6761824286347399384[13] = 0;
   out_6761824286347399384[14] = 0;
   out_6761824286347399384[15] = 0;
   out_6761824286347399384[16] = 0;
   out_6761824286347399384[17] = 0;
   out_6761824286347399384[18] = 0;
   out_6761824286347399384[19] = 0;
   out_6761824286347399384[20] = 0;
   out_6761824286347399384[21] = 0;
   out_6761824286347399384[22] = 0;
   out_6761824286347399384[23] = 0;
   out_6761824286347399384[24] = 0;
   out_6761824286347399384[25] = 1;
   out_6761824286347399384[26] = 0;
   out_6761824286347399384[27] = 0;
   out_6761824286347399384[28] = 0;
   out_6761824286347399384[29] = 0;
   out_6761824286347399384[30] = 0;
   out_6761824286347399384[31] = 0;
   out_6761824286347399384[32] = 0;
   out_6761824286347399384[33] = 0;
   out_6761824286347399384[34] = 0;
   out_6761824286347399384[35] = 0;
   out_6761824286347399384[36] = 0;
   out_6761824286347399384[37] = 0;
   out_6761824286347399384[38] = 0;
   out_6761824286347399384[39] = 0;
   out_6761824286347399384[40] = 0;
   out_6761824286347399384[41] = 0;
   out_6761824286347399384[42] = 0;
   out_6761824286347399384[43] = 0;
   out_6761824286347399384[44] = 1;
   out_6761824286347399384[45] = 0;
   out_6761824286347399384[46] = 0;
   out_6761824286347399384[47] = 0;
   out_6761824286347399384[48] = 0;
   out_6761824286347399384[49] = 0;
   out_6761824286347399384[50] = 0;
   out_6761824286347399384[51] = 0;
   out_6761824286347399384[52] = 0;
   out_6761824286347399384[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_6464539647254934838) {
  err_fun(nom_x, delta_x, out_6464539647254934838);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6486962015996802201) {
  inv_err_fun(nom_x, true_x, out_6486962015996802201);
}
void pose_H_mod_fun(double *state, double *out_6072805508798864108) {
  H_mod_fun(state, out_6072805508798864108);
}
void pose_f_fun(double *state, double dt, double *out_5954902381465487783) {
  f_fun(state,  dt, out_5954902381465487783);
}
void pose_F_fun(double *state, double dt, double *out_3606916503598787696) {
  F_fun(state,  dt, out_3606916503598787696);
}
void pose_h_4(double *state, double *unused, double *out_5987237266806342619) {
  h_4(state, unused, out_5987237266806342619);
}
void pose_H_4(double *state, double *unused, double *out_2798583430007914855) {
  H_4(state, unused, out_2798583430007914855);
}
void pose_h_10(double *state, double *unused, double *out_3025597584677695987) {
  h_10(state, unused, out_3025597584677695987);
}
void pose_H_10(double *state, double *unused, double *out_1745173860280422879) {
  H_10(state, unused, out_1745173860280422879);
}
void pose_h_13(double *state, double *unused, double *out_8599707273883010977) {
  h_13(state, unused, out_8599707273883010977);
}
void pose_H_13(double *state, double *unused, double *out_8037529435384935832) {
  H_13(state, unused, out_8037529435384935832);
}
void pose_h_14(double *state, double *unused, double *out_3899158584107184147) {
  h_14(state, unused, out_3899158584107184147);
}
void pose_H_14(double *state, double *unused, double *out_6761824286347399384) {
  H_14(state, unused, out_6761824286347399384);
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

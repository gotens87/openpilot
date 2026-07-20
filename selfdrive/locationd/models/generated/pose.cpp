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
void err_fun(double *nom_x, double *delta_x, double *out_8879515811318903877) {
   out_8879515811318903877[0] = delta_x[0] + nom_x[0];
   out_8879515811318903877[1] = delta_x[1] + nom_x[1];
   out_8879515811318903877[2] = delta_x[2] + nom_x[2];
   out_8879515811318903877[3] = delta_x[3] + nom_x[3];
   out_8879515811318903877[4] = delta_x[4] + nom_x[4];
   out_8879515811318903877[5] = delta_x[5] + nom_x[5];
   out_8879515811318903877[6] = delta_x[6] + nom_x[6];
   out_8879515811318903877[7] = delta_x[7] + nom_x[7];
   out_8879515811318903877[8] = delta_x[8] + nom_x[8];
   out_8879515811318903877[9] = delta_x[9] + nom_x[9];
   out_8879515811318903877[10] = delta_x[10] + nom_x[10];
   out_8879515811318903877[11] = delta_x[11] + nom_x[11];
   out_8879515811318903877[12] = delta_x[12] + nom_x[12];
   out_8879515811318903877[13] = delta_x[13] + nom_x[13];
   out_8879515811318903877[14] = delta_x[14] + nom_x[14];
   out_8879515811318903877[15] = delta_x[15] + nom_x[15];
   out_8879515811318903877[16] = delta_x[16] + nom_x[16];
   out_8879515811318903877[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5299248816140732451) {
   out_5299248816140732451[0] = -nom_x[0] + true_x[0];
   out_5299248816140732451[1] = -nom_x[1] + true_x[1];
   out_5299248816140732451[2] = -nom_x[2] + true_x[2];
   out_5299248816140732451[3] = -nom_x[3] + true_x[3];
   out_5299248816140732451[4] = -nom_x[4] + true_x[4];
   out_5299248816140732451[5] = -nom_x[5] + true_x[5];
   out_5299248816140732451[6] = -nom_x[6] + true_x[6];
   out_5299248816140732451[7] = -nom_x[7] + true_x[7];
   out_5299248816140732451[8] = -nom_x[8] + true_x[8];
   out_5299248816140732451[9] = -nom_x[9] + true_x[9];
   out_5299248816140732451[10] = -nom_x[10] + true_x[10];
   out_5299248816140732451[11] = -nom_x[11] + true_x[11];
   out_5299248816140732451[12] = -nom_x[12] + true_x[12];
   out_5299248816140732451[13] = -nom_x[13] + true_x[13];
   out_5299248816140732451[14] = -nom_x[14] + true_x[14];
   out_5299248816140732451[15] = -nom_x[15] + true_x[15];
   out_5299248816140732451[16] = -nom_x[16] + true_x[16];
   out_5299248816140732451[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4677544152144015683) {
   out_4677544152144015683[0] = 1.0;
   out_4677544152144015683[1] = 0.0;
   out_4677544152144015683[2] = 0.0;
   out_4677544152144015683[3] = 0.0;
   out_4677544152144015683[4] = 0.0;
   out_4677544152144015683[5] = 0.0;
   out_4677544152144015683[6] = 0.0;
   out_4677544152144015683[7] = 0.0;
   out_4677544152144015683[8] = 0.0;
   out_4677544152144015683[9] = 0.0;
   out_4677544152144015683[10] = 0.0;
   out_4677544152144015683[11] = 0.0;
   out_4677544152144015683[12] = 0.0;
   out_4677544152144015683[13] = 0.0;
   out_4677544152144015683[14] = 0.0;
   out_4677544152144015683[15] = 0.0;
   out_4677544152144015683[16] = 0.0;
   out_4677544152144015683[17] = 0.0;
   out_4677544152144015683[18] = 0.0;
   out_4677544152144015683[19] = 1.0;
   out_4677544152144015683[20] = 0.0;
   out_4677544152144015683[21] = 0.0;
   out_4677544152144015683[22] = 0.0;
   out_4677544152144015683[23] = 0.0;
   out_4677544152144015683[24] = 0.0;
   out_4677544152144015683[25] = 0.0;
   out_4677544152144015683[26] = 0.0;
   out_4677544152144015683[27] = 0.0;
   out_4677544152144015683[28] = 0.0;
   out_4677544152144015683[29] = 0.0;
   out_4677544152144015683[30] = 0.0;
   out_4677544152144015683[31] = 0.0;
   out_4677544152144015683[32] = 0.0;
   out_4677544152144015683[33] = 0.0;
   out_4677544152144015683[34] = 0.0;
   out_4677544152144015683[35] = 0.0;
   out_4677544152144015683[36] = 0.0;
   out_4677544152144015683[37] = 0.0;
   out_4677544152144015683[38] = 1.0;
   out_4677544152144015683[39] = 0.0;
   out_4677544152144015683[40] = 0.0;
   out_4677544152144015683[41] = 0.0;
   out_4677544152144015683[42] = 0.0;
   out_4677544152144015683[43] = 0.0;
   out_4677544152144015683[44] = 0.0;
   out_4677544152144015683[45] = 0.0;
   out_4677544152144015683[46] = 0.0;
   out_4677544152144015683[47] = 0.0;
   out_4677544152144015683[48] = 0.0;
   out_4677544152144015683[49] = 0.0;
   out_4677544152144015683[50] = 0.0;
   out_4677544152144015683[51] = 0.0;
   out_4677544152144015683[52] = 0.0;
   out_4677544152144015683[53] = 0.0;
   out_4677544152144015683[54] = 0.0;
   out_4677544152144015683[55] = 0.0;
   out_4677544152144015683[56] = 0.0;
   out_4677544152144015683[57] = 1.0;
   out_4677544152144015683[58] = 0.0;
   out_4677544152144015683[59] = 0.0;
   out_4677544152144015683[60] = 0.0;
   out_4677544152144015683[61] = 0.0;
   out_4677544152144015683[62] = 0.0;
   out_4677544152144015683[63] = 0.0;
   out_4677544152144015683[64] = 0.0;
   out_4677544152144015683[65] = 0.0;
   out_4677544152144015683[66] = 0.0;
   out_4677544152144015683[67] = 0.0;
   out_4677544152144015683[68] = 0.0;
   out_4677544152144015683[69] = 0.0;
   out_4677544152144015683[70] = 0.0;
   out_4677544152144015683[71] = 0.0;
   out_4677544152144015683[72] = 0.0;
   out_4677544152144015683[73] = 0.0;
   out_4677544152144015683[74] = 0.0;
   out_4677544152144015683[75] = 0.0;
   out_4677544152144015683[76] = 1.0;
   out_4677544152144015683[77] = 0.0;
   out_4677544152144015683[78] = 0.0;
   out_4677544152144015683[79] = 0.0;
   out_4677544152144015683[80] = 0.0;
   out_4677544152144015683[81] = 0.0;
   out_4677544152144015683[82] = 0.0;
   out_4677544152144015683[83] = 0.0;
   out_4677544152144015683[84] = 0.0;
   out_4677544152144015683[85] = 0.0;
   out_4677544152144015683[86] = 0.0;
   out_4677544152144015683[87] = 0.0;
   out_4677544152144015683[88] = 0.0;
   out_4677544152144015683[89] = 0.0;
   out_4677544152144015683[90] = 0.0;
   out_4677544152144015683[91] = 0.0;
   out_4677544152144015683[92] = 0.0;
   out_4677544152144015683[93] = 0.0;
   out_4677544152144015683[94] = 0.0;
   out_4677544152144015683[95] = 1.0;
   out_4677544152144015683[96] = 0.0;
   out_4677544152144015683[97] = 0.0;
   out_4677544152144015683[98] = 0.0;
   out_4677544152144015683[99] = 0.0;
   out_4677544152144015683[100] = 0.0;
   out_4677544152144015683[101] = 0.0;
   out_4677544152144015683[102] = 0.0;
   out_4677544152144015683[103] = 0.0;
   out_4677544152144015683[104] = 0.0;
   out_4677544152144015683[105] = 0.0;
   out_4677544152144015683[106] = 0.0;
   out_4677544152144015683[107] = 0.0;
   out_4677544152144015683[108] = 0.0;
   out_4677544152144015683[109] = 0.0;
   out_4677544152144015683[110] = 0.0;
   out_4677544152144015683[111] = 0.0;
   out_4677544152144015683[112] = 0.0;
   out_4677544152144015683[113] = 0.0;
   out_4677544152144015683[114] = 1.0;
   out_4677544152144015683[115] = 0.0;
   out_4677544152144015683[116] = 0.0;
   out_4677544152144015683[117] = 0.0;
   out_4677544152144015683[118] = 0.0;
   out_4677544152144015683[119] = 0.0;
   out_4677544152144015683[120] = 0.0;
   out_4677544152144015683[121] = 0.0;
   out_4677544152144015683[122] = 0.0;
   out_4677544152144015683[123] = 0.0;
   out_4677544152144015683[124] = 0.0;
   out_4677544152144015683[125] = 0.0;
   out_4677544152144015683[126] = 0.0;
   out_4677544152144015683[127] = 0.0;
   out_4677544152144015683[128] = 0.0;
   out_4677544152144015683[129] = 0.0;
   out_4677544152144015683[130] = 0.0;
   out_4677544152144015683[131] = 0.0;
   out_4677544152144015683[132] = 0.0;
   out_4677544152144015683[133] = 1.0;
   out_4677544152144015683[134] = 0.0;
   out_4677544152144015683[135] = 0.0;
   out_4677544152144015683[136] = 0.0;
   out_4677544152144015683[137] = 0.0;
   out_4677544152144015683[138] = 0.0;
   out_4677544152144015683[139] = 0.0;
   out_4677544152144015683[140] = 0.0;
   out_4677544152144015683[141] = 0.0;
   out_4677544152144015683[142] = 0.0;
   out_4677544152144015683[143] = 0.0;
   out_4677544152144015683[144] = 0.0;
   out_4677544152144015683[145] = 0.0;
   out_4677544152144015683[146] = 0.0;
   out_4677544152144015683[147] = 0.0;
   out_4677544152144015683[148] = 0.0;
   out_4677544152144015683[149] = 0.0;
   out_4677544152144015683[150] = 0.0;
   out_4677544152144015683[151] = 0.0;
   out_4677544152144015683[152] = 1.0;
   out_4677544152144015683[153] = 0.0;
   out_4677544152144015683[154] = 0.0;
   out_4677544152144015683[155] = 0.0;
   out_4677544152144015683[156] = 0.0;
   out_4677544152144015683[157] = 0.0;
   out_4677544152144015683[158] = 0.0;
   out_4677544152144015683[159] = 0.0;
   out_4677544152144015683[160] = 0.0;
   out_4677544152144015683[161] = 0.0;
   out_4677544152144015683[162] = 0.0;
   out_4677544152144015683[163] = 0.0;
   out_4677544152144015683[164] = 0.0;
   out_4677544152144015683[165] = 0.0;
   out_4677544152144015683[166] = 0.0;
   out_4677544152144015683[167] = 0.0;
   out_4677544152144015683[168] = 0.0;
   out_4677544152144015683[169] = 0.0;
   out_4677544152144015683[170] = 0.0;
   out_4677544152144015683[171] = 1.0;
   out_4677544152144015683[172] = 0.0;
   out_4677544152144015683[173] = 0.0;
   out_4677544152144015683[174] = 0.0;
   out_4677544152144015683[175] = 0.0;
   out_4677544152144015683[176] = 0.0;
   out_4677544152144015683[177] = 0.0;
   out_4677544152144015683[178] = 0.0;
   out_4677544152144015683[179] = 0.0;
   out_4677544152144015683[180] = 0.0;
   out_4677544152144015683[181] = 0.0;
   out_4677544152144015683[182] = 0.0;
   out_4677544152144015683[183] = 0.0;
   out_4677544152144015683[184] = 0.0;
   out_4677544152144015683[185] = 0.0;
   out_4677544152144015683[186] = 0.0;
   out_4677544152144015683[187] = 0.0;
   out_4677544152144015683[188] = 0.0;
   out_4677544152144015683[189] = 0.0;
   out_4677544152144015683[190] = 1.0;
   out_4677544152144015683[191] = 0.0;
   out_4677544152144015683[192] = 0.0;
   out_4677544152144015683[193] = 0.0;
   out_4677544152144015683[194] = 0.0;
   out_4677544152144015683[195] = 0.0;
   out_4677544152144015683[196] = 0.0;
   out_4677544152144015683[197] = 0.0;
   out_4677544152144015683[198] = 0.0;
   out_4677544152144015683[199] = 0.0;
   out_4677544152144015683[200] = 0.0;
   out_4677544152144015683[201] = 0.0;
   out_4677544152144015683[202] = 0.0;
   out_4677544152144015683[203] = 0.0;
   out_4677544152144015683[204] = 0.0;
   out_4677544152144015683[205] = 0.0;
   out_4677544152144015683[206] = 0.0;
   out_4677544152144015683[207] = 0.0;
   out_4677544152144015683[208] = 0.0;
   out_4677544152144015683[209] = 1.0;
   out_4677544152144015683[210] = 0.0;
   out_4677544152144015683[211] = 0.0;
   out_4677544152144015683[212] = 0.0;
   out_4677544152144015683[213] = 0.0;
   out_4677544152144015683[214] = 0.0;
   out_4677544152144015683[215] = 0.0;
   out_4677544152144015683[216] = 0.0;
   out_4677544152144015683[217] = 0.0;
   out_4677544152144015683[218] = 0.0;
   out_4677544152144015683[219] = 0.0;
   out_4677544152144015683[220] = 0.0;
   out_4677544152144015683[221] = 0.0;
   out_4677544152144015683[222] = 0.0;
   out_4677544152144015683[223] = 0.0;
   out_4677544152144015683[224] = 0.0;
   out_4677544152144015683[225] = 0.0;
   out_4677544152144015683[226] = 0.0;
   out_4677544152144015683[227] = 0.0;
   out_4677544152144015683[228] = 1.0;
   out_4677544152144015683[229] = 0.0;
   out_4677544152144015683[230] = 0.0;
   out_4677544152144015683[231] = 0.0;
   out_4677544152144015683[232] = 0.0;
   out_4677544152144015683[233] = 0.0;
   out_4677544152144015683[234] = 0.0;
   out_4677544152144015683[235] = 0.0;
   out_4677544152144015683[236] = 0.0;
   out_4677544152144015683[237] = 0.0;
   out_4677544152144015683[238] = 0.0;
   out_4677544152144015683[239] = 0.0;
   out_4677544152144015683[240] = 0.0;
   out_4677544152144015683[241] = 0.0;
   out_4677544152144015683[242] = 0.0;
   out_4677544152144015683[243] = 0.0;
   out_4677544152144015683[244] = 0.0;
   out_4677544152144015683[245] = 0.0;
   out_4677544152144015683[246] = 0.0;
   out_4677544152144015683[247] = 1.0;
   out_4677544152144015683[248] = 0.0;
   out_4677544152144015683[249] = 0.0;
   out_4677544152144015683[250] = 0.0;
   out_4677544152144015683[251] = 0.0;
   out_4677544152144015683[252] = 0.0;
   out_4677544152144015683[253] = 0.0;
   out_4677544152144015683[254] = 0.0;
   out_4677544152144015683[255] = 0.0;
   out_4677544152144015683[256] = 0.0;
   out_4677544152144015683[257] = 0.0;
   out_4677544152144015683[258] = 0.0;
   out_4677544152144015683[259] = 0.0;
   out_4677544152144015683[260] = 0.0;
   out_4677544152144015683[261] = 0.0;
   out_4677544152144015683[262] = 0.0;
   out_4677544152144015683[263] = 0.0;
   out_4677544152144015683[264] = 0.0;
   out_4677544152144015683[265] = 0.0;
   out_4677544152144015683[266] = 1.0;
   out_4677544152144015683[267] = 0.0;
   out_4677544152144015683[268] = 0.0;
   out_4677544152144015683[269] = 0.0;
   out_4677544152144015683[270] = 0.0;
   out_4677544152144015683[271] = 0.0;
   out_4677544152144015683[272] = 0.0;
   out_4677544152144015683[273] = 0.0;
   out_4677544152144015683[274] = 0.0;
   out_4677544152144015683[275] = 0.0;
   out_4677544152144015683[276] = 0.0;
   out_4677544152144015683[277] = 0.0;
   out_4677544152144015683[278] = 0.0;
   out_4677544152144015683[279] = 0.0;
   out_4677544152144015683[280] = 0.0;
   out_4677544152144015683[281] = 0.0;
   out_4677544152144015683[282] = 0.0;
   out_4677544152144015683[283] = 0.0;
   out_4677544152144015683[284] = 0.0;
   out_4677544152144015683[285] = 1.0;
   out_4677544152144015683[286] = 0.0;
   out_4677544152144015683[287] = 0.0;
   out_4677544152144015683[288] = 0.0;
   out_4677544152144015683[289] = 0.0;
   out_4677544152144015683[290] = 0.0;
   out_4677544152144015683[291] = 0.0;
   out_4677544152144015683[292] = 0.0;
   out_4677544152144015683[293] = 0.0;
   out_4677544152144015683[294] = 0.0;
   out_4677544152144015683[295] = 0.0;
   out_4677544152144015683[296] = 0.0;
   out_4677544152144015683[297] = 0.0;
   out_4677544152144015683[298] = 0.0;
   out_4677544152144015683[299] = 0.0;
   out_4677544152144015683[300] = 0.0;
   out_4677544152144015683[301] = 0.0;
   out_4677544152144015683[302] = 0.0;
   out_4677544152144015683[303] = 0.0;
   out_4677544152144015683[304] = 1.0;
   out_4677544152144015683[305] = 0.0;
   out_4677544152144015683[306] = 0.0;
   out_4677544152144015683[307] = 0.0;
   out_4677544152144015683[308] = 0.0;
   out_4677544152144015683[309] = 0.0;
   out_4677544152144015683[310] = 0.0;
   out_4677544152144015683[311] = 0.0;
   out_4677544152144015683[312] = 0.0;
   out_4677544152144015683[313] = 0.0;
   out_4677544152144015683[314] = 0.0;
   out_4677544152144015683[315] = 0.0;
   out_4677544152144015683[316] = 0.0;
   out_4677544152144015683[317] = 0.0;
   out_4677544152144015683[318] = 0.0;
   out_4677544152144015683[319] = 0.0;
   out_4677544152144015683[320] = 0.0;
   out_4677544152144015683[321] = 0.0;
   out_4677544152144015683[322] = 0.0;
   out_4677544152144015683[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_7013494318329414190) {
   out_7013494318329414190[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_7013494318329414190[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_7013494318329414190[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_7013494318329414190[3] = dt*state[12] + state[3];
   out_7013494318329414190[4] = dt*state[13] + state[4];
   out_7013494318329414190[5] = dt*state[14] + state[5];
   out_7013494318329414190[6] = state[6];
   out_7013494318329414190[7] = state[7];
   out_7013494318329414190[8] = state[8];
   out_7013494318329414190[9] = state[9];
   out_7013494318329414190[10] = state[10];
   out_7013494318329414190[11] = state[11];
   out_7013494318329414190[12] = state[12];
   out_7013494318329414190[13] = state[13];
   out_7013494318329414190[14] = state[14];
   out_7013494318329414190[15] = state[15];
   out_7013494318329414190[16] = state[16];
   out_7013494318329414190[17] = state[17];
}
void F_fun(double *state, double dt, double *out_1354267763168931680) {
   out_1354267763168931680[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1354267763168931680[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1354267763168931680[2] = 0;
   out_1354267763168931680[3] = 0;
   out_1354267763168931680[4] = 0;
   out_1354267763168931680[5] = 0;
   out_1354267763168931680[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1354267763168931680[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1354267763168931680[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1354267763168931680[9] = 0;
   out_1354267763168931680[10] = 0;
   out_1354267763168931680[11] = 0;
   out_1354267763168931680[12] = 0;
   out_1354267763168931680[13] = 0;
   out_1354267763168931680[14] = 0;
   out_1354267763168931680[15] = 0;
   out_1354267763168931680[16] = 0;
   out_1354267763168931680[17] = 0;
   out_1354267763168931680[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1354267763168931680[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1354267763168931680[20] = 0;
   out_1354267763168931680[21] = 0;
   out_1354267763168931680[22] = 0;
   out_1354267763168931680[23] = 0;
   out_1354267763168931680[24] = 0;
   out_1354267763168931680[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1354267763168931680[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1354267763168931680[27] = 0;
   out_1354267763168931680[28] = 0;
   out_1354267763168931680[29] = 0;
   out_1354267763168931680[30] = 0;
   out_1354267763168931680[31] = 0;
   out_1354267763168931680[32] = 0;
   out_1354267763168931680[33] = 0;
   out_1354267763168931680[34] = 0;
   out_1354267763168931680[35] = 0;
   out_1354267763168931680[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1354267763168931680[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1354267763168931680[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1354267763168931680[39] = 0;
   out_1354267763168931680[40] = 0;
   out_1354267763168931680[41] = 0;
   out_1354267763168931680[42] = 0;
   out_1354267763168931680[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1354267763168931680[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1354267763168931680[45] = 0;
   out_1354267763168931680[46] = 0;
   out_1354267763168931680[47] = 0;
   out_1354267763168931680[48] = 0;
   out_1354267763168931680[49] = 0;
   out_1354267763168931680[50] = 0;
   out_1354267763168931680[51] = 0;
   out_1354267763168931680[52] = 0;
   out_1354267763168931680[53] = 0;
   out_1354267763168931680[54] = 0;
   out_1354267763168931680[55] = 0;
   out_1354267763168931680[56] = 0;
   out_1354267763168931680[57] = 1;
   out_1354267763168931680[58] = 0;
   out_1354267763168931680[59] = 0;
   out_1354267763168931680[60] = 0;
   out_1354267763168931680[61] = 0;
   out_1354267763168931680[62] = 0;
   out_1354267763168931680[63] = 0;
   out_1354267763168931680[64] = 0;
   out_1354267763168931680[65] = 0;
   out_1354267763168931680[66] = dt;
   out_1354267763168931680[67] = 0;
   out_1354267763168931680[68] = 0;
   out_1354267763168931680[69] = 0;
   out_1354267763168931680[70] = 0;
   out_1354267763168931680[71] = 0;
   out_1354267763168931680[72] = 0;
   out_1354267763168931680[73] = 0;
   out_1354267763168931680[74] = 0;
   out_1354267763168931680[75] = 0;
   out_1354267763168931680[76] = 1;
   out_1354267763168931680[77] = 0;
   out_1354267763168931680[78] = 0;
   out_1354267763168931680[79] = 0;
   out_1354267763168931680[80] = 0;
   out_1354267763168931680[81] = 0;
   out_1354267763168931680[82] = 0;
   out_1354267763168931680[83] = 0;
   out_1354267763168931680[84] = 0;
   out_1354267763168931680[85] = dt;
   out_1354267763168931680[86] = 0;
   out_1354267763168931680[87] = 0;
   out_1354267763168931680[88] = 0;
   out_1354267763168931680[89] = 0;
   out_1354267763168931680[90] = 0;
   out_1354267763168931680[91] = 0;
   out_1354267763168931680[92] = 0;
   out_1354267763168931680[93] = 0;
   out_1354267763168931680[94] = 0;
   out_1354267763168931680[95] = 1;
   out_1354267763168931680[96] = 0;
   out_1354267763168931680[97] = 0;
   out_1354267763168931680[98] = 0;
   out_1354267763168931680[99] = 0;
   out_1354267763168931680[100] = 0;
   out_1354267763168931680[101] = 0;
   out_1354267763168931680[102] = 0;
   out_1354267763168931680[103] = 0;
   out_1354267763168931680[104] = dt;
   out_1354267763168931680[105] = 0;
   out_1354267763168931680[106] = 0;
   out_1354267763168931680[107] = 0;
   out_1354267763168931680[108] = 0;
   out_1354267763168931680[109] = 0;
   out_1354267763168931680[110] = 0;
   out_1354267763168931680[111] = 0;
   out_1354267763168931680[112] = 0;
   out_1354267763168931680[113] = 0;
   out_1354267763168931680[114] = 1;
   out_1354267763168931680[115] = 0;
   out_1354267763168931680[116] = 0;
   out_1354267763168931680[117] = 0;
   out_1354267763168931680[118] = 0;
   out_1354267763168931680[119] = 0;
   out_1354267763168931680[120] = 0;
   out_1354267763168931680[121] = 0;
   out_1354267763168931680[122] = 0;
   out_1354267763168931680[123] = 0;
   out_1354267763168931680[124] = 0;
   out_1354267763168931680[125] = 0;
   out_1354267763168931680[126] = 0;
   out_1354267763168931680[127] = 0;
   out_1354267763168931680[128] = 0;
   out_1354267763168931680[129] = 0;
   out_1354267763168931680[130] = 0;
   out_1354267763168931680[131] = 0;
   out_1354267763168931680[132] = 0;
   out_1354267763168931680[133] = 1;
   out_1354267763168931680[134] = 0;
   out_1354267763168931680[135] = 0;
   out_1354267763168931680[136] = 0;
   out_1354267763168931680[137] = 0;
   out_1354267763168931680[138] = 0;
   out_1354267763168931680[139] = 0;
   out_1354267763168931680[140] = 0;
   out_1354267763168931680[141] = 0;
   out_1354267763168931680[142] = 0;
   out_1354267763168931680[143] = 0;
   out_1354267763168931680[144] = 0;
   out_1354267763168931680[145] = 0;
   out_1354267763168931680[146] = 0;
   out_1354267763168931680[147] = 0;
   out_1354267763168931680[148] = 0;
   out_1354267763168931680[149] = 0;
   out_1354267763168931680[150] = 0;
   out_1354267763168931680[151] = 0;
   out_1354267763168931680[152] = 1;
   out_1354267763168931680[153] = 0;
   out_1354267763168931680[154] = 0;
   out_1354267763168931680[155] = 0;
   out_1354267763168931680[156] = 0;
   out_1354267763168931680[157] = 0;
   out_1354267763168931680[158] = 0;
   out_1354267763168931680[159] = 0;
   out_1354267763168931680[160] = 0;
   out_1354267763168931680[161] = 0;
   out_1354267763168931680[162] = 0;
   out_1354267763168931680[163] = 0;
   out_1354267763168931680[164] = 0;
   out_1354267763168931680[165] = 0;
   out_1354267763168931680[166] = 0;
   out_1354267763168931680[167] = 0;
   out_1354267763168931680[168] = 0;
   out_1354267763168931680[169] = 0;
   out_1354267763168931680[170] = 0;
   out_1354267763168931680[171] = 1;
   out_1354267763168931680[172] = 0;
   out_1354267763168931680[173] = 0;
   out_1354267763168931680[174] = 0;
   out_1354267763168931680[175] = 0;
   out_1354267763168931680[176] = 0;
   out_1354267763168931680[177] = 0;
   out_1354267763168931680[178] = 0;
   out_1354267763168931680[179] = 0;
   out_1354267763168931680[180] = 0;
   out_1354267763168931680[181] = 0;
   out_1354267763168931680[182] = 0;
   out_1354267763168931680[183] = 0;
   out_1354267763168931680[184] = 0;
   out_1354267763168931680[185] = 0;
   out_1354267763168931680[186] = 0;
   out_1354267763168931680[187] = 0;
   out_1354267763168931680[188] = 0;
   out_1354267763168931680[189] = 0;
   out_1354267763168931680[190] = 1;
   out_1354267763168931680[191] = 0;
   out_1354267763168931680[192] = 0;
   out_1354267763168931680[193] = 0;
   out_1354267763168931680[194] = 0;
   out_1354267763168931680[195] = 0;
   out_1354267763168931680[196] = 0;
   out_1354267763168931680[197] = 0;
   out_1354267763168931680[198] = 0;
   out_1354267763168931680[199] = 0;
   out_1354267763168931680[200] = 0;
   out_1354267763168931680[201] = 0;
   out_1354267763168931680[202] = 0;
   out_1354267763168931680[203] = 0;
   out_1354267763168931680[204] = 0;
   out_1354267763168931680[205] = 0;
   out_1354267763168931680[206] = 0;
   out_1354267763168931680[207] = 0;
   out_1354267763168931680[208] = 0;
   out_1354267763168931680[209] = 1;
   out_1354267763168931680[210] = 0;
   out_1354267763168931680[211] = 0;
   out_1354267763168931680[212] = 0;
   out_1354267763168931680[213] = 0;
   out_1354267763168931680[214] = 0;
   out_1354267763168931680[215] = 0;
   out_1354267763168931680[216] = 0;
   out_1354267763168931680[217] = 0;
   out_1354267763168931680[218] = 0;
   out_1354267763168931680[219] = 0;
   out_1354267763168931680[220] = 0;
   out_1354267763168931680[221] = 0;
   out_1354267763168931680[222] = 0;
   out_1354267763168931680[223] = 0;
   out_1354267763168931680[224] = 0;
   out_1354267763168931680[225] = 0;
   out_1354267763168931680[226] = 0;
   out_1354267763168931680[227] = 0;
   out_1354267763168931680[228] = 1;
   out_1354267763168931680[229] = 0;
   out_1354267763168931680[230] = 0;
   out_1354267763168931680[231] = 0;
   out_1354267763168931680[232] = 0;
   out_1354267763168931680[233] = 0;
   out_1354267763168931680[234] = 0;
   out_1354267763168931680[235] = 0;
   out_1354267763168931680[236] = 0;
   out_1354267763168931680[237] = 0;
   out_1354267763168931680[238] = 0;
   out_1354267763168931680[239] = 0;
   out_1354267763168931680[240] = 0;
   out_1354267763168931680[241] = 0;
   out_1354267763168931680[242] = 0;
   out_1354267763168931680[243] = 0;
   out_1354267763168931680[244] = 0;
   out_1354267763168931680[245] = 0;
   out_1354267763168931680[246] = 0;
   out_1354267763168931680[247] = 1;
   out_1354267763168931680[248] = 0;
   out_1354267763168931680[249] = 0;
   out_1354267763168931680[250] = 0;
   out_1354267763168931680[251] = 0;
   out_1354267763168931680[252] = 0;
   out_1354267763168931680[253] = 0;
   out_1354267763168931680[254] = 0;
   out_1354267763168931680[255] = 0;
   out_1354267763168931680[256] = 0;
   out_1354267763168931680[257] = 0;
   out_1354267763168931680[258] = 0;
   out_1354267763168931680[259] = 0;
   out_1354267763168931680[260] = 0;
   out_1354267763168931680[261] = 0;
   out_1354267763168931680[262] = 0;
   out_1354267763168931680[263] = 0;
   out_1354267763168931680[264] = 0;
   out_1354267763168931680[265] = 0;
   out_1354267763168931680[266] = 1;
   out_1354267763168931680[267] = 0;
   out_1354267763168931680[268] = 0;
   out_1354267763168931680[269] = 0;
   out_1354267763168931680[270] = 0;
   out_1354267763168931680[271] = 0;
   out_1354267763168931680[272] = 0;
   out_1354267763168931680[273] = 0;
   out_1354267763168931680[274] = 0;
   out_1354267763168931680[275] = 0;
   out_1354267763168931680[276] = 0;
   out_1354267763168931680[277] = 0;
   out_1354267763168931680[278] = 0;
   out_1354267763168931680[279] = 0;
   out_1354267763168931680[280] = 0;
   out_1354267763168931680[281] = 0;
   out_1354267763168931680[282] = 0;
   out_1354267763168931680[283] = 0;
   out_1354267763168931680[284] = 0;
   out_1354267763168931680[285] = 1;
   out_1354267763168931680[286] = 0;
   out_1354267763168931680[287] = 0;
   out_1354267763168931680[288] = 0;
   out_1354267763168931680[289] = 0;
   out_1354267763168931680[290] = 0;
   out_1354267763168931680[291] = 0;
   out_1354267763168931680[292] = 0;
   out_1354267763168931680[293] = 0;
   out_1354267763168931680[294] = 0;
   out_1354267763168931680[295] = 0;
   out_1354267763168931680[296] = 0;
   out_1354267763168931680[297] = 0;
   out_1354267763168931680[298] = 0;
   out_1354267763168931680[299] = 0;
   out_1354267763168931680[300] = 0;
   out_1354267763168931680[301] = 0;
   out_1354267763168931680[302] = 0;
   out_1354267763168931680[303] = 0;
   out_1354267763168931680[304] = 1;
   out_1354267763168931680[305] = 0;
   out_1354267763168931680[306] = 0;
   out_1354267763168931680[307] = 0;
   out_1354267763168931680[308] = 0;
   out_1354267763168931680[309] = 0;
   out_1354267763168931680[310] = 0;
   out_1354267763168931680[311] = 0;
   out_1354267763168931680[312] = 0;
   out_1354267763168931680[313] = 0;
   out_1354267763168931680[314] = 0;
   out_1354267763168931680[315] = 0;
   out_1354267763168931680[316] = 0;
   out_1354267763168931680[317] = 0;
   out_1354267763168931680[318] = 0;
   out_1354267763168931680[319] = 0;
   out_1354267763168931680[320] = 0;
   out_1354267763168931680[321] = 0;
   out_1354267763168931680[322] = 0;
   out_1354267763168931680[323] = 1;
}
void h_4(double *state, double *unused, double *out_4996940222577203074) {
   out_4996940222577203074[0] = state[6] + state[9];
   out_4996940222577203074[1] = state[7] + state[10];
   out_4996940222577203074[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_1403322073353066430) {
   out_1403322073353066430[0] = 0;
   out_1403322073353066430[1] = 0;
   out_1403322073353066430[2] = 0;
   out_1403322073353066430[3] = 0;
   out_1403322073353066430[4] = 0;
   out_1403322073353066430[5] = 0;
   out_1403322073353066430[6] = 1;
   out_1403322073353066430[7] = 0;
   out_1403322073353066430[8] = 0;
   out_1403322073353066430[9] = 1;
   out_1403322073353066430[10] = 0;
   out_1403322073353066430[11] = 0;
   out_1403322073353066430[12] = 0;
   out_1403322073353066430[13] = 0;
   out_1403322073353066430[14] = 0;
   out_1403322073353066430[15] = 0;
   out_1403322073353066430[16] = 0;
   out_1403322073353066430[17] = 0;
   out_1403322073353066430[18] = 0;
   out_1403322073353066430[19] = 0;
   out_1403322073353066430[20] = 0;
   out_1403322073353066430[21] = 0;
   out_1403322073353066430[22] = 0;
   out_1403322073353066430[23] = 0;
   out_1403322073353066430[24] = 0;
   out_1403322073353066430[25] = 1;
   out_1403322073353066430[26] = 0;
   out_1403322073353066430[27] = 0;
   out_1403322073353066430[28] = 1;
   out_1403322073353066430[29] = 0;
   out_1403322073353066430[30] = 0;
   out_1403322073353066430[31] = 0;
   out_1403322073353066430[32] = 0;
   out_1403322073353066430[33] = 0;
   out_1403322073353066430[34] = 0;
   out_1403322073353066430[35] = 0;
   out_1403322073353066430[36] = 0;
   out_1403322073353066430[37] = 0;
   out_1403322073353066430[38] = 0;
   out_1403322073353066430[39] = 0;
   out_1403322073353066430[40] = 0;
   out_1403322073353066430[41] = 0;
   out_1403322073353066430[42] = 0;
   out_1403322073353066430[43] = 0;
   out_1403322073353066430[44] = 1;
   out_1403322073353066430[45] = 0;
   out_1403322073353066430[46] = 0;
   out_1403322073353066430[47] = 1;
   out_1403322073353066430[48] = 0;
   out_1403322073353066430[49] = 0;
   out_1403322073353066430[50] = 0;
   out_1403322073353066430[51] = 0;
   out_1403322073353066430[52] = 0;
   out_1403322073353066430[53] = 0;
}
void h_10(double *state, double *unused, double *out_4736471881593411299) {
   out_4736471881593411299[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_4736471881593411299[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_4736471881593411299[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_6592862647789865695) {
   out_6592862647789865695[0] = 0;
   out_6592862647789865695[1] = 9.8100000000000005*cos(state[1]);
   out_6592862647789865695[2] = 0;
   out_6592862647789865695[3] = 0;
   out_6592862647789865695[4] = -state[8];
   out_6592862647789865695[5] = state[7];
   out_6592862647789865695[6] = 0;
   out_6592862647789865695[7] = state[5];
   out_6592862647789865695[8] = -state[4];
   out_6592862647789865695[9] = 0;
   out_6592862647789865695[10] = 0;
   out_6592862647789865695[11] = 0;
   out_6592862647789865695[12] = 1;
   out_6592862647789865695[13] = 0;
   out_6592862647789865695[14] = 0;
   out_6592862647789865695[15] = 1;
   out_6592862647789865695[16] = 0;
   out_6592862647789865695[17] = 0;
   out_6592862647789865695[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_6592862647789865695[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_6592862647789865695[20] = 0;
   out_6592862647789865695[21] = state[8];
   out_6592862647789865695[22] = 0;
   out_6592862647789865695[23] = -state[6];
   out_6592862647789865695[24] = -state[5];
   out_6592862647789865695[25] = 0;
   out_6592862647789865695[26] = state[3];
   out_6592862647789865695[27] = 0;
   out_6592862647789865695[28] = 0;
   out_6592862647789865695[29] = 0;
   out_6592862647789865695[30] = 0;
   out_6592862647789865695[31] = 1;
   out_6592862647789865695[32] = 0;
   out_6592862647789865695[33] = 0;
   out_6592862647789865695[34] = 1;
   out_6592862647789865695[35] = 0;
   out_6592862647789865695[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_6592862647789865695[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_6592862647789865695[38] = 0;
   out_6592862647789865695[39] = -state[7];
   out_6592862647789865695[40] = state[6];
   out_6592862647789865695[41] = 0;
   out_6592862647789865695[42] = state[4];
   out_6592862647789865695[43] = -state[3];
   out_6592862647789865695[44] = 0;
   out_6592862647789865695[45] = 0;
   out_6592862647789865695[46] = 0;
   out_6592862647789865695[47] = 0;
   out_6592862647789865695[48] = 0;
   out_6592862647789865695[49] = 0;
   out_6592862647789865695[50] = 1;
   out_6592862647789865695[51] = 0;
   out_6592862647789865695[52] = 0;
   out_6592862647789865695[53] = 1;
}
void h_13(double *state, double *unused, double *out_4300313025089579851) {
   out_4300313025089579851[0] = state[3];
   out_4300313025089579851[1] = state[4];
   out_4300313025089579851[2] = state[5];
}
void H_13(double *state, double *unused, double *out_4615595898685399231) {
   out_4615595898685399231[0] = 0;
   out_4615595898685399231[1] = 0;
   out_4615595898685399231[2] = 0;
   out_4615595898685399231[3] = 1;
   out_4615595898685399231[4] = 0;
   out_4615595898685399231[5] = 0;
   out_4615595898685399231[6] = 0;
   out_4615595898685399231[7] = 0;
   out_4615595898685399231[8] = 0;
   out_4615595898685399231[9] = 0;
   out_4615595898685399231[10] = 0;
   out_4615595898685399231[11] = 0;
   out_4615595898685399231[12] = 0;
   out_4615595898685399231[13] = 0;
   out_4615595898685399231[14] = 0;
   out_4615595898685399231[15] = 0;
   out_4615595898685399231[16] = 0;
   out_4615595898685399231[17] = 0;
   out_4615595898685399231[18] = 0;
   out_4615595898685399231[19] = 0;
   out_4615595898685399231[20] = 0;
   out_4615595898685399231[21] = 0;
   out_4615595898685399231[22] = 1;
   out_4615595898685399231[23] = 0;
   out_4615595898685399231[24] = 0;
   out_4615595898685399231[25] = 0;
   out_4615595898685399231[26] = 0;
   out_4615595898685399231[27] = 0;
   out_4615595898685399231[28] = 0;
   out_4615595898685399231[29] = 0;
   out_4615595898685399231[30] = 0;
   out_4615595898685399231[31] = 0;
   out_4615595898685399231[32] = 0;
   out_4615595898685399231[33] = 0;
   out_4615595898685399231[34] = 0;
   out_4615595898685399231[35] = 0;
   out_4615595898685399231[36] = 0;
   out_4615595898685399231[37] = 0;
   out_4615595898685399231[38] = 0;
   out_4615595898685399231[39] = 0;
   out_4615595898685399231[40] = 0;
   out_4615595898685399231[41] = 1;
   out_4615595898685399231[42] = 0;
   out_4615595898685399231[43] = 0;
   out_4615595898685399231[44] = 0;
   out_4615595898685399231[45] = 0;
   out_4615595898685399231[46] = 0;
   out_4615595898685399231[47] = 0;
   out_4615595898685399231[48] = 0;
   out_4615595898685399231[49] = 0;
   out_4615595898685399231[50] = 0;
   out_4615595898685399231[51] = 0;
   out_4615595898685399231[52] = 0;
   out_4615595898685399231[53] = 0;
}
void h_14(double *state, double *unused, double *out_4861809331317933851) {
   out_4861809331317933851[0] = state[6];
   out_4861809331317933851[1] = state[7];
   out_4861809331317933851[2] = state[8];
}
void H_14(double *state, double *unused, double *out_5366562929692550959) {
   out_5366562929692550959[0] = 0;
   out_5366562929692550959[1] = 0;
   out_5366562929692550959[2] = 0;
   out_5366562929692550959[3] = 0;
   out_5366562929692550959[4] = 0;
   out_5366562929692550959[5] = 0;
   out_5366562929692550959[6] = 1;
   out_5366562929692550959[7] = 0;
   out_5366562929692550959[8] = 0;
   out_5366562929692550959[9] = 0;
   out_5366562929692550959[10] = 0;
   out_5366562929692550959[11] = 0;
   out_5366562929692550959[12] = 0;
   out_5366562929692550959[13] = 0;
   out_5366562929692550959[14] = 0;
   out_5366562929692550959[15] = 0;
   out_5366562929692550959[16] = 0;
   out_5366562929692550959[17] = 0;
   out_5366562929692550959[18] = 0;
   out_5366562929692550959[19] = 0;
   out_5366562929692550959[20] = 0;
   out_5366562929692550959[21] = 0;
   out_5366562929692550959[22] = 0;
   out_5366562929692550959[23] = 0;
   out_5366562929692550959[24] = 0;
   out_5366562929692550959[25] = 1;
   out_5366562929692550959[26] = 0;
   out_5366562929692550959[27] = 0;
   out_5366562929692550959[28] = 0;
   out_5366562929692550959[29] = 0;
   out_5366562929692550959[30] = 0;
   out_5366562929692550959[31] = 0;
   out_5366562929692550959[32] = 0;
   out_5366562929692550959[33] = 0;
   out_5366562929692550959[34] = 0;
   out_5366562929692550959[35] = 0;
   out_5366562929692550959[36] = 0;
   out_5366562929692550959[37] = 0;
   out_5366562929692550959[38] = 0;
   out_5366562929692550959[39] = 0;
   out_5366562929692550959[40] = 0;
   out_5366562929692550959[41] = 0;
   out_5366562929692550959[42] = 0;
   out_5366562929692550959[43] = 0;
   out_5366562929692550959[44] = 1;
   out_5366562929692550959[45] = 0;
   out_5366562929692550959[46] = 0;
   out_5366562929692550959[47] = 0;
   out_5366562929692550959[48] = 0;
   out_5366562929692550959[49] = 0;
   out_5366562929692550959[50] = 0;
   out_5366562929692550959[51] = 0;
   out_5366562929692550959[52] = 0;
   out_5366562929692550959[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_8879515811318903877) {
  err_fun(nom_x, delta_x, out_8879515811318903877);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5299248816140732451) {
  inv_err_fun(nom_x, true_x, out_5299248816140732451);
}
void pose_H_mod_fun(double *state, double *out_4677544152144015683) {
  H_mod_fun(state, out_4677544152144015683);
}
void pose_f_fun(double *state, double dt, double *out_7013494318329414190) {
  f_fun(state,  dt, out_7013494318329414190);
}
void pose_F_fun(double *state, double dt, double *out_1354267763168931680) {
  F_fun(state,  dt, out_1354267763168931680);
}
void pose_h_4(double *state, double *unused, double *out_4996940222577203074) {
  h_4(state, unused, out_4996940222577203074);
}
void pose_H_4(double *state, double *unused, double *out_1403322073353066430) {
  H_4(state, unused, out_1403322073353066430);
}
void pose_h_10(double *state, double *unused, double *out_4736471881593411299) {
  h_10(state, unused, out_4736471881593411299);
}
void pose_H_10(double *state, double *unused, double *out_6592862647789865695) {
  H_10(state, unused, out_6592862647789865695);
}
void pose_h_13(double *state, double *unused, double *out_4300313025089579851) {
  h_13(state, unused, out_4300313025089579851);
}
void pose_H_13(double *state, double *unused, double *out_4615595898685399231) {
  H_13(state, unused, out_4615595898685399231);
}
void pose_h_14(double *state, double *unused, double *out_4861809331317933851) {
  h_14(state, unused, out_4861809331317933851);
}
void pose_H_14(double *state, double *unused, double *out_5366562929692550959) {
  H_14(state, unused, out_5366562929692550959);
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

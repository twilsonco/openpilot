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
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4328417858719399296) {
   out_4328417858719399296[0] = delta_x[0] + nom_x[0];
   out_4328417858719399296[1] = delta_x[1] + nom_x[1];
   out_4328417858719399296[2] = delta_x[2] + nom_x[2];
   out_4328417858719399296[3] = delta_x[3] + nom_x[3];
   out_4328417858719399296[4] = delta_x[4] + nom_x[4];
   out_4328417858719399296[5] = delta_x[5] + nom_x[5];
   out_4328417858719399296[6] = delta_x[6] + nom_x[6];
   out_4328417858719399296[7] = delta_x[7] + nom_x[7];
   out_4328417858719399296[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7622011457833195352) {
   out_7622011457833195352[0] = -nom_x[0] + true_x[0];
   out_7622011457833195352[1] = -nom_x[1] + true_x[1];
   out_7622011457833195352[2] = -nom_x[2] + true_x[2];
   out_7622011457833195352[3] = -nom_x[3] + true_x[3];
   out_7622011457833195352[4] = -nom_x[4] + true_x[4];
   out_7622011457833195352[5] = -nom_x[5] + true_x[5];
   out_7622011457833195352[6] = -nom_x[6] + true_x[6];
   out_7622011457833195352[7] = -nom_x[7] + true_x[7];
   out_7622011457833195352[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7531422148562293488) {
   out_7531422148562293488[0] = 1.0;
   out_7531422148562293488[1] = 0;
   out_7531422148562293488[2] = 0;
   out_7531422148562293488[3] = 0;
   out_7531422148562293488[4] = 0;
   out_7531422148562293488[5] = 0;
   out_7531422148562293488[6] = 0;
   out_7531422148562293488[7] = 0;
   out_7531422148562293488[8] = 0;
   out_7531422148562293488[9] = 0;
   out_7531422148562293488[10] = 1.0;
   out_7531422148562293488[11] = 0;
   out_7531422148562293488[12] = 0;
   out_7531422148562293488[13] = 0;
   out_7531422148562293488[14] = 0;
   out_7531422148562293488[15] = 0;
   out_7531422148562293488[16] = 0;
   out_7531422148562293488[17] = 0;
   out_7531422148562293488[18] = 0;
   out_7531422148562293488[19] = 0;
   out_7531422148562293488[20] = 1.0;
   out_7531422148562293488[21] = 0;
   out_7531422148562293488[22] = 0;
   out_7531422148562293488[23] = 0;
   out_7531422148562293488[24] = 0;
   out_7531422148562293488[25] = 0;
   out_7531422148562293488[26] = 0;
   out_7531422148562293488[27] = 0;
   out_7531422148562293488[28] = 0;
   out_7531422148562293488[29] = 0;
   out_7531422148562293488[30] = 1.0;
   out_7531422148562293488[31] = 0;
   out_7531422148562293488[32] = 0;
   out_7531422148562293488[33] = 0;
   out_7531422148562293488[34] = 0;
   out_7531422148562293488[35] = 0;
   out_7531422148562293488[36] = 0;
   out_7531422148562293488[37] = 0;
   out_7531422148562293488[38] = 0;
   out_7531422148562293488[39] = 0;
   out_7531422148562293488[40] = 1.0;
   out_7531422148562293488[41] = 0;
   out_7531422148562293488[42] = 0;
   out_7531422148562293488[43] = 0;
   out_7531422148562293488[44] = 0;
   out_7531422148562293488[45] = 0;
   out_7531422148562293488[46] = 0;
   out_7531422148562293488[47] = 0;
   out_7531422148562293488[48] = 0;
   out_7531422148562293488[49] = 0;
   out_7531422148562293488[50] = 1.0;
   out_7531422148562293488[51] = 0;
   out_7531422148562293488[52] = 0;
   out_7531422148562293488[53] = 0;
   out_7531422148562293488[54] = 0;
   out_7531422148562293488[55] = 0;
   out_7531422148562293488[56] = 0;
   out_7531422148562293488[57] = 0;
   out_7531422148562293488[58] = 0;
   out_7531422148562293488[59] = 0;
   out_7531422148562293488[60] = 1.0;
   out_7531422148562293488[61] = 0;
   out_7531422148562293488[62] = 0;
   out_7531422148562293488[63] = 0;
   out_7531422148562293488[64] = 0;
   out_7531422148562293488[65] = 0;
   out_7531422148562293488[66] = 0;
   out_7531422148562293488[67] = 0;
   out_7531422148562293488[68] = 0;
   out_7531422148562293488[69] = 0;
   out_7531422148562293488[70] = 1.0;
   out_7531422148562293488[71] = 0;
   out_7531422148562293488[72] = 0;
   out_7531422148562293488[73] = 0;
   out_7531422148562293488[74] = 0;
   out_7531422148562293488[75] = 0;
   out_7531422148562293488[76] = 0;
   out_7531422148562293488[77] = 0;
   out_7531422148562293488[78] = 0;
   out_7531422148562293488[79] = 0;
   out_7531422148562293488[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3545004875602579137) {
   out_3545004875602579137[0] = state[0];
   out_3545004875602579137[1] = state[1];
   out_3545004875602579137[2] = state[2];
   out_3545004875602579137[3] = state[3];
   out_3545004875602579137[4] = state[4];
   out_3545004875602579137[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3545004875602579137[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3545004875602579137[7] = state[7];
   out_3545004875602579137[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2926959116897174320) {
   out_2926959116897174320[0] = 1;
   out_2926959116897174320[1] = 0;
   out_2926959116897174320[2] = 0;
   out_2926959116897174320[3] = 0;
   out_2926959116897174320[4] = 0;
   out_2926959116897174320[5] = 0;
   out_2926959116897174320[6] = 0;
   out_2926959116897174320[7] = 0;
   out_2926959116897174320[8] = 0;
   out_2926959116897174320[9] = 0;
   out_2926959116897174320[10] = 1;
   out_2926959116897174320[11] = 0;
   out_2926959116897174320[12] = 0;
   out_2926959116897174320[13] = 0;
   out_2926959116897174320[14] = 0;
   out_2926959116897174320[15] = 0;
   out_2926959116897174320[16] = 0;
   out_2926959116897174320[17] = 0;
   out_2926959116897174320[18] = 0;
   out_2926959116897174320[19] = 0;
   out_2926959116897174320[20] = 1;
   out_2926959116897174320[21] = 0;
   out_2926959116897174320[22] = 0;
   out_2926959116897174320[23] = 0;
   out_2926959116897174320[24] = 0;
   out_2926959116897174320[25] = 0;
   out_2926959116897174320[26] = 0;
   out_2926959116897174320[27] = 0;
   out_2926959116897174320[28] = 0;
   out_2926959116897174320[29] = 0;
   out_2926959116897174320[30] = 1;
   out_2926959116897174320[31] = 0;
   out_2926959116897174320[32] = 0;
   out_2926959116897174320[33] = 0;
   out_2926959116897174320[34] = 0;
   out_2926959116897174320[35] = 0;
   out_2926959116897174320[36] = 0;
   out_2926959116897174320[37] = 0;
   out_2926959116897174320[38] = 0;
   out_2926959116897174320[39] = 0;
   out_2926959116897174320[40] = 1;
   out_2926959116897174320[41] = 0;
   out_2926959116897174320[42] = 0;
   out_2926959116897174320[43] = 0;
   out_2926959116897174320[44] = 0;
   out_2926959116897174320[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2926959116897174320[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2926959116897174320[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2926959116897174320[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2926959116897174320[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2926959116897174320[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2926959116897174320[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2926959116897174320[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2926959116897174320[53] = -9.8000000000000007*dt;
   out_2926959116897174320[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2926959116897174320[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2926959116897174320[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2926959116897174320[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2926959116897174320[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2926959116897174320[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2926959116897174320[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2926959116897174320[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2926959116897174320[62] = 0;
   out_2926959116897174320[63] = 0;
   out_2926959116897174320[64] = 0;
   out_2926959116897174320[65] = 0;
   out_2926959116897174320[66] = 0;
   out_2926959116897174320[67] = 0;
   out_2926959116897174320[68] = 0;
   out_2926959116897174320[69] = 0;
   out_2926959116897174320[70] = 1;
   out_2926959116897174320[71] = 0;
   out_2926959116897174320[72] = 0;
   out_2926959116897174320[73] = 0;
   out_2926959116897174320[74] = 0;
   out_2926959116897174320[75] = 0;
   out_2926959116897174320[76] = 0;
   out_2926959116897174320[77] = 0;
   out_2926959116897174320[78] = 0;
   out_2926959116897174320[79] = 0;
   out_2926959116897174320[80] = 1;
}
void h_25(double *state, double *unused, double *out_4206336983505903873) {
   out_4206336983505903873[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2027678294181840875) {
   out_2027678294181840875[0] = 0;
   out_2027678294181840875[1] = 0;
   out_2027678294181840875[2] = 0;
   out_2027678294181840875[3] = 0;
   out_2027678294181840875[4] = 0;
   out_2027678294181840875[5] = 0;
   out_2027678294181840875[6] = 1;
   out_2027678294181840875[7] = 0;
   out_2027678294181840875[8] = 0;
}
void h_24(double *state, double *unused, double *out_6602550213615218626) {
   out_6602550213615218626[0] = state[4];
   out_6602550213615218626[1] = state[5];
}
void H_24(double *state, double *unused, double *out_228169780941976754) {
   out_228169780941976754[0] = 0;
   out_228169780941976754[1] = 0;
   out_228169780941976754[2] = 0;
   out_228169780941976754[3] = 0;
   out_228169780941976754[4] = 1;
   out_228169780941976754[5] = 0;
   out_228169780941976754[6] = 0;
   out_228169780941976754[7] = 0;
   out_228169780941976754[8] = 0;
   out_228169780941976754[9] = 0;
   out_228169780941976754[10] = 0;
   out_228169780941976754[11] = 0;
   out_228169780941976754[12] = 0;
   out_228169780941976754[13] = 0;
   out_228169780941976754[14] = 1;
   out_228169780941976754[15] = 0;
   out_228169780941976754[16] = 0;
   out_228169780941976754[17] = 0;
}
void h_30(double *state, double *unused, double *out_7843786511759926223) {
   out_7843786511759926223[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6555374624309449073) {
   out_6555374624309449073[0] = 0;
   out_6555374624309449073[1] = 0;
   out_6555374624309449073[2] = 0;
   out_6555374624309449073[3] = 0;
   out_6555374624309449073[4] = 1;
   out_6555374624309449073[5] = 0;
   out_6555374624309449073[6] = 0;
   out_6555374624309449073[7] = 0;
   out_6555374624309449073[8] = 0;
}
void h_26(double *state, double *unused, double *out_6962450324126076075) {
   out_6962450324126076075[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5769181613055897099) {
   out_5769181613055897099[0] = 0;
   out_5769181613055897099[1] = 0;
   out_5769181613055897099[2] = 0;
   out_5769181613055897099[3] = 0;
   out_5769181613055897099[4] = 0;
   out_5769181613055897099[5] = 0;
   out_5769181613055897099[6] = 0;
   out_5769181613055897099[7] = 1;
   out_5769181613055897099[8] = 0;
}
void h_27(double *state, double *unused, double *out_3049806733587547836) {
   out_3049806733587547836[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8730137936109873984) {
   out_8730137936109873984[0] = 0;
   out_8730137936109873984[1] = 0;
   out_8730137936109873984[2] = 0;
   out_8730137936109873984[3] = 1;
   out_8730137936109873984[4] = 0;
   out_8730137936109873984[5] = 0;
   out_8730137936109873984[6] = 0;
   out_8730137936109873984[7] = 0;
   out_8730137936109873984[8] = 0;
}
void h_29(double *state, double *unused, double *out_3105640462253112293) {
   out_3105640462253112293[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6045143279995056889) {
   out_6045143279995056889[0] = 0;
   out_6045143279995056889[1] = 1;
   out_6045143279995056889[2] = 0;
   out_6045143279995056889[3] = 0;
   out_6045143279995056889[4] = 0;
   out_6045143279995056889[5] = 0;
   out_6045143279995056889[6] = 0;
   out_6045143279995056889[7] = 0;
   out_6045143279995056889[8] = 0;
}
void h_28(double *state, double *unused, double *out_226041079159908753) {
   out_226041079159908753[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4081513008429730638) {
   out_4081513008429730638[0] = 1;
   out_4081513008429730638[1] = 0;
   out_4081513008429730638[2] = 0;
   out_4081513008429730638[3] = 0;
   out_4081513008429730638[4] = 0;
   out_4081513008429730638[5] = 0;
   out_4081513008429730638[6] = 0;
   out_4081513008429730638[7] = 0;
   out_4081513008429730638[8] = 0;
}
void h_31(double *state, double *unused, double *out_4807755141119267427) {
   out_4807755141119267427[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6395389715289248575) {
   out_6395389715289248575[0] = 0;
   out_6395389715289248575[1] = 0;
   out_6395389715289248575[2] = 0;
   out_6395389715289248575[3] = 0;
   out_6395389715289248575[4] = 0;
   out_6395389715289248575[5] = 0;
   out_6395389715289248575[6] = 0;
   out_6395389715289248575[7] = 0;
   out_6395389715289248575[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4328417858719399296) {
  err_fun(nom_x, delta_x, out_4328417858719399296);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7622011457833195352) {
  inv_err_fun(nom_x, true_x, out_7622011457833195352);
}
void car_H_mod_fun(double *state, double *out_7531422148562293488) {
  H_mod_fun(state, out_7531422148562293488);
}
void car_f_fun(double *state, double dt, double *out_3545004875602579137) {
  f_fun(state,  dt, out_3545004875602579137);
}
void car_F_fun(double *state, double dt, double *out_2926959116897174320) {
  F_fun(state,  dt, out_2926959116897174320);
}
void car_h_25(double *state, double *unused, double *out_4206336983505903873) {
  h_25(state, unused, out_4206336983505903873);
}
void car_H_25(double *state, double *unused, double *out_2027678294181840875) {
  H_25(state, unused, out_2027678294181840875);
}
void car_h_24(double *state, double *unused, double *out_6602550213615218626) {
  h_24(state, unused, out_6602550213615218626);
}
void car_H_24(double *state, double *unused, double *out_228169780941976754) {
  H_24(state, unused, out_228169780941976754);
}
void car_h_30(double *state, double *unused, double *out_7843786511759926223) {
  h_30(state, unused, out_7843786511759926223);
}
void car_H_30(double *state, double *unused, double *out_6555374624309449073) {
  H_30(state, unused, out_6555374624309449073);
}
void car_h_26(double *state, double *unused, double *out_6962450324126076075) {
  h_26(state, unused, out_6962450324126076075);
}
void car_H_26(double *state, double *unused, double *out_5769181613055897099) {
  H_26(state, unused, out_5769181613055897099);
}
void car_h_27(double *state, double *unused, double *out_3049806733587547836) {
  h_27(state, unused, out_3049806733587547836);
}
void car_H_27(double *state, double *unused, double *out_8730137936109873984) {
  H_27(state, unused, out_8730137936109873984);
}
void car_h_29(double *state, double *unused, double *out_3105640462253112293) {
  h_29(state, unused, out_3105640462253112293);
}
void car_H_29(double *state, double *unused, double *out_6045143279995056889) {
  H_29(state, unused, out_6045143279995056889);
}
void car_h_28(double *state, double *unused, double *out_226041079159908753) {
  h_28(state, unused, out_226041079159908753);
}
void car_H_28(double *state, double *unused, double *out_4081513008429730638) {
  H_28(state, unused, out_4081513008429730638);
}
void car_h_31(double *state, double *unused, double *out_4807755141119267427) {
  h_31(state, unused, out_4807755141119267427);
}
void car_H_31(double *state, double *unused, double *out_6395389715289248575) {
  H_31(state, unused, out_6395389715289248575);
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

ekf_lib_init(car)

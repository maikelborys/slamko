// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Perspective-3-Point (P3P) closed-form solver — slamko_core geometry primitive.
// Header-only, host/device callable. Grunert (1841) with Haralick's 1994
// coefficient table: a degree-4 polynomial in v = s3/s1, then Horn 3-point
// absolute orientation maps the camera-frame triangle to the world-frame
// triangle, returning (R, t) with R ∈ SO(3). Verbatim port of the validated
// slamko_vio solver into core so any package (slamko_loop's relocalizer, …) can
// reuse it without a cross-package dep (Hard Rule #2). vio keeps its own copy
// for now; dedup later.
//
// Caller responsibilities: bearings f_k unit + positive-z (in front of camera);
// world points non-collinear. Returns the number of valid solutions (0..4).

#ifndef SLAMKO_CORE_P3P_SOLVER_HPP_
#define SLAMKO_CORE_P3P_SOLVER_HPP_

#ifndef SLAMKO_P3P_HD
#ifdef __CUDACC__
#define SLAMKO_P3P_HD __host__ __device__
#else
#define SLAMKO_P3P_HD
#endif
#endif

#include <cmath>

#ifndef SLAMKO_P3P_DEBUG
#define SLAMKO_P3P_DEBUG 0
#endif

namespace slamko {
namespace p3p {

// 3-vector primitives.
SLAMKO_P3P_HD inline double v3_dot(const double a[3], const double b[3]) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
SLAMKO_P3P_HD inline void v3_cross(const double a[3], const double b[3], double c[3]) {
  c[0] = a[1]*b[2] - a[2]*b[1];
  c[1] = a[2]*b[0] - a[0]*b[2];
  c[2] = a[0]*b[1] - a[1]*b[0];
}
SLAMKO_P3P_HD inline double v3_norm(const double a[3]) {
  return sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
}

// Solve depressed quartic y^4 + p y^2 + q y + r = 0 by Ferrari's method.
SLAMKO_P3P_HD inline int solve_depressed_quartic(
    double p, double q, double r, double roots[4]) {
  const double tol = 1e-14;
  if (fabs(q) < tol) {  // biquadratic
    const double disc = p*p - 4.0*r;
    if (disc < 0.0) return 0;
    const double sq = sqrt(disc);
    int cnt = 0;
    const double y2a = 0.5 * (-p + sq);
    const double y2b = 0.5 * (-p - sq);
    if (y2a >= 0.0) { const double y = sqrt(y2a); roots[cnt++] = y; roots[cnt++] = -y; }
    if (y2b >= 0.0) { const double y = sqrt(y2b); roots[cnt++] = y; roots[cnt++] = -y; }
    return cnt;
  }
  const double A_c = -p / 2.0;
  const double B_c = -r;
  const double C_c = (p * r) / 2.0 - q * q / 8.0;
  const double P = B_c - A_c * A_c / 3.0;
  const double Q = 2.0 * A_c * A_c * A_c / 27.0 - A_c * B_c / 3.0 + C_c;
  const double D = Q * Q / 4.0 + P * P * P / 27.0;
  double m;
  if (D >= 0.0) {
    const double sq = sqrt(D);
    const double u_cu = -Q / 2.0 + sq;
    const double v_cu = -Q / 2.0 - sq;
    const double cbu = (u_cu >= 0.0) ? cbrt(u_cu) : -cbrt(-u_cu);
    const double cbv = (v_cu >= 0.0) ? cbrt(v_cu) : -cbrt(-v_cu);
    m = cbu + cbv - A_c / 3.0;
  } else {
    const double R_ = sqrt(-P * P * P / 27.0);
    const double phi = acos(-Q / (2.0 * R_));
    const double mag = 2.0 * cbrt(R_);
    const double m0 = mag * cos(phi / 3.0) - A_c / 3.0;
    const double m1 = mag * cos((phi + 2.0 * M_PI) / 3.0) - A_c / 3.0;
    const double m2 = mag * cos((phi + 4.0 * M_PI) / 3.0) - A_c / 3.0;
    m = m0;
    if (2.0*m1 - p > 2.0*m - p) m = m1;
    if (2.0*m2 - p > 2.0*m - p) m = m2;
  }
  const double alpha2 = 2.0 * m - p;
  if (alpha2 < -1e-9) return 0;
  const double alpha = (alpha2 < 0.0) ? 0.0 : sqrt(alpha2);
  if (alpha < tol) return 0;
  const double beta  = m + q / (2.0 * alpha);
  const double gamma = m - q / (2.0 * alpha);
  int cnt = 0;
  const double d1 = alpha*alpha - 4.0 * beta;
  if (d1 >= -1e-12) {
    const double sq1 = sqrt(d1 < 0.0 ? 0.0 : d1);
    roots[cnt++] = 0.5 * ( alpha + sq1);
    roots[cnt++] = 0.5 * ( alpha - sq1);
  }
  const double d2 = alpha*alpha - 4.0 * gamma;
  if (d2 >= -1e-12) {
    const double sq2 = sqrt(d2 < 0.0 ? 0.0 : d2);
    roots[cnt++] = 0.5 * (-alpha + sq2);
    roots[cnt++] = 0.5 * (-alpha - sq2);
  }
  return cnt;
}

// Horn 3-point absolute orientation: Xc[i] = R Pw[i] + t (R row-major, det=+1).
SLAMKO_P3P_HD inline bool horn_3pt(const double Pw[3][3], const double Xc[3][3],
                                   double R[9], double t[3]) {
  double w12[3] = {Pw[1][0]-Pw[0][0], Pw[1][1]-Pw[0][1], Pw[1][2]-Pw[0][2]};
  double w13[3] = {Pw[2][0]-Pw[0][0], Pw[2][1]-Pw[0][1], Pw[2][2]-Pw[0][2]};
  double e1w[3] = {w12[0], w12[1], w12[2]};
  double n1w = v3_norm(e1w); if (n1w < 1e-12) return false;
  e1w[0]/=n1w; e1w[1]/=n1w; e1w[2]/=n1w;
  double e3w[3]; v3_cross(e1w, w13, e3w);
  double n3w = v3_norm(e3w); if (n3w < 1e-12) return false;
  e3w[0]/=n3w; e3w[1]/=n3w; e3w[2]/=n3w;
  double e2w[3]; v3_cross(e3w, e1w, e2w);

  double c12[3] = {Xc[1][0]-Xc[0][0], Xc[1][1]-Xc[0][1], Xc[1][2]-Xc[0][2]};
  double c13[3] = {Xc[2][0]-Xc[0][0], Xc[2][1]-Xc[0][1], Xc[2][2]-Xc[0][2]};
  double e1c[3] = {c12[0], c12[1], c12[2]};
  double n1c = v3_norm(e1c); if (n1c < 1e-12) return false;
  e1c[0]/=n1c; e1c[1]/=n1c; e1c[2]/=n1c;
  double e3c[3]; v3_cross(e1c, c13, e3c);
  double n3c = v3_norm(e3c); if (n3c < 1e-12) return false;
  e3c[0]/=n3c; e3c[1]/=n3c; e3c[2]/=n3c;
  double e2c[3]; v3_cross(e3c, e1c, e2c);

  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      R[i*3 + j] = e1c[i]*e1w[j] + e2c[i]*e2w[j] + e3c[i]*e3w[j];
  for (int i = 0; i < 3; ++i) {
    const double rp = R[i*3 + 0]*Pw[0][0] + R[i*3 + 1]*Pw[0][1] + R[i*3 + 2]*Pw[0][2];
    t[i] = Xc[0][i] - rp;
  }
  return true;
}

// Solve P3P. Pw[3][3]: 3 world points; f[3][3]: 3 unit bearings (camera-frame).
// Outputs up to 4 (R_out[k] row-major 3×3 world→camera, t_out[k]). Returns count.
SLAMKO_P3P_HD inline int solve(
    const double Pw[3][3], const double f[3][3],
    double R_out[4][9], double t_out[4][3]) {
  double v12[3] = {Pw[1][0]-Pw[0][0], Pw[1][1]-Pw[0][1], Pw[1][2]-Pw[0][2]};
  double v13[3] = {Pw[2][0]-Pw[0][0], Pw[2][1]-Pw[0][1], Pw[2][2]-Pw[0][2]};
  double v23[3] = {Pw[2][0]-Pw[1][0], Pw[2][1]-Pw[1][1], Pw[2][2]-Pw[1][2]};
  const double a = v3_norm(v23);
  const double b = v3_norm(v13);
  const double c = v3_norm(v12);
  if (a < 1e-9 || b < 1e-9 || c < 1e-9) return 0;

  const double cA = v3_dot(f[1], f[2]);
  const double cB = v3_dot(f[0], f[2]);
  const double cG = v3_dot(f[0], f[1]);

  const double a2 = a*a, b2 = b*b, c2 = c*c;
  const double K1 = b2 / a2;
  const double K2 = c2 / a2;

  const double a_2 = K1 + K2 - 1.0;
  const double a_1 = -2.0 * cB * (K2 - 1.0);
  const double a_0 = K2 - K1 - 1.0;
  const double b_1 = -2.0 * K1 * cA;
  const double b_0 =  2.0 * K1 * cG;
  const double g_3 = -2.0 * K2 * cA;
  const double g_2 =  2.0 * cG * (1.0 - K1) + 4.0 * K2 * cA * cB;
  const double g_1 =  2.0 * cA * (K1 - K2) - 4.0 * cB * cG;
  const double g_0 =  2.0 * cG;

  const double G4 = a_2 * a_2 - b_1 * g_3;
  const double G3 = 2.0*a_2*a_1 - (b_1*g_2 + b_0*g_3);
  const double G2 = 2.0*a_2*a_0 + a_1*a_1 - (b_1*g_1 + b_0*g_2);
  const double G1 = 2.0*a_1*a_0 - (b_1*g_0 + b_0*g_1);
  const double G0 = a_0 * a_0 - b_0 * g_0;

  if (fabs(G4) < 1e-14) return 0;
  const double aQ = G3 / G4;
  const double bQ = G2 / G4;
  const double cQ = G1 / G4;
  const double dQ = G0 / G4;
  const double pD = bQ - 3.0*aQ*aQ/8.0;
  const double qD = aQ*aQ*aQ/8.0 - aQ*bQ/2.0 + cQ;
  const double rD = -3.0*aQ*aQ*aQ*aQ/256.0 + aQ*aQ*bQ/16.0 - aQ*cQ/4.0 + dQ;

  double roots[4];
  const int nr = solve_depressed_quartic(pD, qD, rD, roots);
  if (nr <= 0) return 0;
  (void)b2; (void)c2;

  int n_sol = 0;
  for (int k = 0; k < nr && n_sol < 4; ++k) {
    const double v = roots[k] - aQ/4.0;
    if (v <= 0.0) continue;

    const double den_s1 = 1.0 + v*v - 2.0*v*cB;
    if (den_s1 <= 0.0) continue;
    const double s1 = b / sqrt(den_s1);
    const double s3 = v * s1;

    const double Au = K2 - 1.0;
    const double Bu = 2.0 * cG - 2.0 * K2 * v * cA;
    const double Cu = K2 * v * v - 1.0;
    int n_u = 0;
    double u_vals[2];
    if (fabs(Au) < 1e-12) {
      if (fabs(Bu) > 1e-12) u_vals[n_u++] = -Cu / Bu;
    } else {
      const double dU = Bu*Bu - 4.0*Au*Cu;
      if (dU < 0.0) continue;
      const double sqU = sqrt(dU);
      u_vals[n_u++] = (-Bu + sqU) / (2.0*Au);
      u_vals[n_u++] = (-Bu - sqU) / (2.0*Au);
    }

    for (int iu = 0; iu < n_u && n_sol < 4; ++iu) {
      const double u = u_vals[iu];
      if (u <= 0.0) continue;
      const double s2 = u * s1;

      const double chk_a = s2*s2 + s3*s3 - 2.0*s2*s3*cA;
      if (fabs(chk_a - a2) > 1e-3 * a2 + 1e-6) continue;

      double Xc[3][3] = {
        {s1*f[0][0], s1*f[0][1], s1*f[0][2]},
        {s2*f[1][0], s2*f[1][1], s2*f[1][2]},
        {s3*f[2][0], s3*f[2][1], s3*f[2][2]},
      };

      double R[9], t[3];
      if (!horn_3pt(Pw, Xc, R, t)) continue;

      bool ok = true;
      for (int p = 0; p < 3 && ok; ++p) {
        const double pz = R[6]*Pw[p][0] + R[7]*Pw[p][1] + R[8]*Pw[p][2] + t[2];
        if (pz <= 0.0) { ok = false; break; }
        const double px = R[0]*Pw[p][0] + R[1]*Pw[p][1] + R[2]*Pw[p][2] + t[0];
        const double py = R[3]*Pw[p][0] + R[4]*Pw[p][1] + R[5]*Pw[p][2] + t[1];
        const double ex = px - Xc[p][0], ey = py - Xc[p][1], ez = pz - Xc[p][2];
        if (ex*ex + ey*ey + ez*ez > 1e-6) { ok = false; break; }
      }
      if (!ok) continue;

      for (int j = 0; j < 9; ++j) R_out[n_sol][j] = R[j];
      for (int j = 0; j < 3; ++j) t_out[n_sol][j] = t[j];
      ++n_sol;
    }
  }
  return n_sol;
}

}  // namespace p3p
}  // namespace slamko

#endif  // SLAMKO_CORE_P3P_SOLVER_HPP_

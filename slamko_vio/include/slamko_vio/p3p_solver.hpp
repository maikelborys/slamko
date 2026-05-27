// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Maikel Borys
//
// Perspective-3-Point (P3P) closed-form solver. Header-only, __host__
// __device__ callable so the same code is used by the CPU unit test and the
// CUDA RANSAC kernel.
//
// Algorithm: Grunert (1841) with Haralick's 1994 review-paper coefficient
// table. Polynomial in v = s3/s1 (cube-of-distance ratios). The full system
// is three quadratic distance equations
//
//   |Xc_i - Xc_j|^2 = |Pw_i - Pw_j|^2   (i ≠ j ∈ {1,2,3})
//
// where Xc_i = s_i · f_i are camera-frame points along unit bearings f_i.
// Elimination yields a degree-4 polynomial in v. Given a root v_k, one
// recovers s1 from the (1,3) equation, u = s2/s1 from a quadratic, then
// s_2 = u s_1, s_3 = v s_1. Horn 3-point absolute orientation maps the
// resulting camera-frame triangle to the world-frame triangle, returning
// (R, t) with R ∈ SO(3) (handedness-preserving by the construction).
//
// Numerics: coefficient computation in double precision; the per-point
// reprojection check at the end can be done in single precision by the
// caller. Output (R, t) returned in double.
//
// Caller responsibilities:
//   - Bearings f_k must be unit vectors (||f_k|| = 1) with positive z (i.e.,
//     in front of the camera).
//   - World points must be non-collinear (otherwise P3P is ill-defined).
//
// Returns the number of valid solutions (0..4).

#ifndef KLT_VO_P3P_SOLVER_HPP_
#define KLT_VO_P3P_SOLVER_HPP_

#ifdef __CUDACC__
#define KLT_VO_HD __host__ __device__
#else
#define KLT_VO_HD
#endif

#include <cmath>

#ifndef KLT_VO_P3P_DEBUG
#define KLT_VO_P3P_DEBUG 0
#endif

#if KLT_VO_P3P_DEBUG
#include <cstdio>
#endif

namespace slamko_vio {
namespace p3p {

// 3-vector primitives.
KLT_VO_HD inline double v3_dot(const double a[3], const double b[3]) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
KLT_VO_HD inline void v3_cross(const double a[3], const double b[3], double c[3]) {
  c[0] = a[1]*b[2] - a[2]*b[1];
  c[1] = a[2]*b[0] - a[0]*b[2];
  c[2] = a[0]*b[1] - a[1]*b[0];
}
KLT_VO_HD inline double v3_norm(const double a[3]) {
  return sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
}

// Solve depressed quartic y^4 + p y^2 + q y + r = 0 by Ferrari's method.
// Writes real roots to roots[]; returns count (0..4).
KLT_VO_HD inline int solve_depressed_quartic(
    double p, double q, double r, double roots[4]) {
  const double tol = 1e-14;
  // Special: q ≈ 0 → biquadratic.
  if (fabs(q) < tol) {
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
  // Ferrari's resolvent cubic for depressed quartic y^4 + p y^2 + q y + r = 0:
  //   m^3 - (p/2) m^2 - r m + (p r / 2 - q^2 / 8) = 0
  // i.e. monic m^3 + A m^2 + B m + C = 0 with:
  const double A_c = -p / 2.0;
  const double B_c = -r;
  const double C_c = (p * r) / 2.0 - q * q / 8.0;
  // Depress cubic: m = z - A_c/3 → z^3 + P z + Q = 0
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
    // Casus irreducibilis: three real roots; we want one s.t. 2m - p ≥ 0.
    const double R_ = sqrt(-P * P * P / 27.0);
    const double phi = acos(-Q / (2.0 * R_));
    const double mag = 2.0 * cbrt(R_);
    const double m0 = mag * cos(phi / 3.0) - A_c / 3.0;
    const double m1 = mag * cos((phi + 2.0 * M_PI) / 3.0) - A_c / 3.0;
    const double m2 = mag * cos((phi + 4.0 * M_PI) / 3.0) - A_c / 3.0;
    // Pick the root that maximises 2m - p (for the largest real α).
    m = m0;
    if (2.0*m1 - p > 2.0*m - p) m = m1;
    if (2.0*m2 - p > 2.0*m - p) m = m2;
  }
  // Ferrari split: α² = 2m - p; β = m - q/(2α); γ = m + q/(2α).
  // Quartic factors as (y² - α y + β)(y² + α y + γ) = 0.
  const double alpha2 = 2.0 * m - p;
  if (alpha2 < -1e-9) return 0;
  const double alpha = (alpha2 < 0.0) ? 0.0 : sqrt(alpha2);
  if (alpha < tol) return 0;
  const double beta  = m + q / (2.0 * alpha);
  const double gamma = m - q / (2.0 * alpha);
  int cnt = 0;
  // y² - α y + β = 0
  const double d1 = alpha*alpha - 4.0 * beta;
  if (d1 >= -1e-12) {
    const double sq1 = sqrt(d1 < 0.0 ? 0.0 : d1);
    roots[cnt++] = 0.5 * ( alpha + sq1);
    roots[cnt++] = 0.5 * ( alpha - sq1);
  }
  // y² + α y + γ = 0
  const double d2 = alpha*alpha - 4.0 * gamma;
  if (d2 >= -1e-12) {
    const double sq2 = sqrt(d2 < 0.0 ? 0.0 : d2);
    roots[cnt++] = 0.5 * (-alpha + sq2);
    roots[cnt++] = 0.5 * (-alpha - sq2);
  }
  return cnt;
}

// Horn 3-point absolute orientation. Given 3 paired world-frame points Pw and
// 3 camera-frame points Xc with the SAME pairwise distances, find R, t such
// that Xc[i] = R Pw[i] + t. Construct an orthonormal basis from each triangle
// and form R as the product of the two basis matrices. R is automatically a
// proper rotation (det=+1) because both bases are right-handed.
KLT_VO_HD inline bool horn_3pt(const double Pw[3][3], const double Xc[3][3],
                               double R[9], double t[3]) {
  // World-frame triangle basis.
  double w12[3] = {Pw[1][0]-Pw[0][0], Pw[1][1]-Pw[0][1], Pw[1][2]-Pw[0][2]};
  double w13[3] = {Pw[2][0]-Pw[0][0], Pw[2][1]-Pw[0][1], Pw[2][2]-Pw[0][2]};
  double e1w[3] = {w12[0], w12[1], w12[2]};
  double n1w = v3_norm(e1w); if (n1w < 1e-12) return false;
  e1w[0]/=n1w; e1w[1]/=n1w; e1w[2]/=n1w;
  double e3w[3]; v3_cross(e1w, w13, e3w);
  double n3w = v3_norm(e3w); if (n3w < 1e-12) return false;
  e3w[0]/=n3w; e3w[1]/=n3w; e3w[2]/=n3w;
  double e2w[3]; v3_cross(e3w, e1w, e2w);

  // Camera-frame triangle basis.
  double c12[3] = {Xc[1][0]-Xc[0][0], Xc[1][1]-Xc[0][1], Xc[1][2]-Xc[0][2]};
  double c13[3] = {Xc[2][0]-Xc[0][0], Xc[2][1]-Xc[0][1], Xc[2][2]-Xc[0][2]};
  double e1c[3] = {c12[0], c12[1], c12[2]};
  double n1c = v3_norm(e1c); if (n1c < 1e-12) return false;
  e1c[0]/=n1c; e1c[1]/=n1c; e1c[2]/=n1c;
  double e3c[3]; v3_cross(e1c, c13, e3c);
  double n3c = v3_norm(e3c); if (n3c < 1e-12) return false;
  e3c[0]/=n3c; e3c[1]/=n3c; e3c[2]/=n3c;
  double e2c[3]; v3_cross(e3c, e1c, e2c);

  // R = E_c · E_w^T, in row-major:
  //   R[i,j] = sum_k e_c_k[i] * e_w_k[j]
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      R[i*3 + j] = e1c[i]*e1w[j] + e2c[i]*e2w[j] + e3c[i]*e3w[j];
    }
  }
  // t = Xc[0] - R Pw[0]
  for (int i = 0; i < 3; ++i) {
    const double rp = R[i*3 + 0]*Pw[0][0] + R[i*3 + 1]*Pw[0][1] + R[i*3 + 2]*Pw[0][2];
    t[i] = Xc[0][i] - rp;
  }
  return true;
}

// Solve P3P. Inputs:
//   Pw[3][3] : 3 world-frame points
//   f[3][3]  : 3 unit bearing vectors (camera-frame)
// Outputs (up to 4 solutions):
//   R_out[k][9] : row-major 3×3 rotation, world → camera
//   t_out[k][3] : translation, world → camera
// Returns count (0..4).
KLT_VO_HD inline int solve(
    const double Pw[3][3], const double f[3][3],
    double R_out[4][9], double t_out[4][3]) {
  // Sides of world triangle.
  double v12[3] = {Pw[1][0]-Pw[0][0], Pw[1][1]-Pw[0][1], Pw[1][2]-Pw[0][2]};
  double v13[3] = {Pw[2][0]-Pw[0][0], Pw[2][1]-Pw[0][1], Pw[2][2]-Pw[0][2]};
  double v23[3] = {Pw[2][0]-Pw[1][0], Pw[2][1]-Pw[1][1], Pw[2][2]-Pw[1][2]};
  const double a = v3_norm(v23);
  const double b = v3_norm(v13);
  const double c = v3_norm(v12);
  if (a < 1e-9 || b < 1e-9 || c < 1e-9) return 0;

  // Cosines between camera rays. cA = cos(angle between f2,f3) = cos α
  // opposite to side a = |P2-P3|; analogous for cB, cG.
  const double cA = v3_dot(f[1], f[2]);
  const double cB = v3_dot(f[0], f[2]);
  const double cG = v3_dot(f[0], f[1]);

  const double a2 = a*a, b2 = b*b, c2 = c*c;
  const double K1 = b2 / a2;
  const double K2 = c2 / a2;

  // Substitute u = s2/s1, v = s3/s1 into the three squared-distance equations:
  //   (1) |X1-X2|² = c² → s1²(1 - 2u cG + u²) = c²
  //   (2) |X1-X3|² = b² → s1²(1 - 2v cB + v²) = b²
  //   (3) |X2-X3|² = a² → s1²(u² - 2uv cA + v²) = a²
  // Eliminate s1 (≡ ratios) → two quadratics in (u, v):
  //   Q1: K1 u² - 2 K1 v cA u + ((K1-1) v² + 2 v cB - 1) = 0    (from (3)=(2)·K1)
  //   Q2: (K2-1) u² + (2 cG - 2 K2 v cA) u + (K2 v² - 1) = 0    (from (3)=(1)·K2)
  // Sylvester resultant in u → quartic in v. Coefficients of α(v), β(v), γ(v):
  //   α(v) = A1 C2 - A2 C1 = a_2 v² + a_1 v + a_0   (degree 2)
  //   β(v) = A1 B2 - A2 B1 = b_1 v + b_0            (degree 1)
  //   γ(v) = B1 C2 - B2 C1 = g_3 v³ + g_2 v² + g_1 v + g_0   (degree 3)
  // Resultant P(v) = α(v)² - β(v) γ(v) — degree 4 in v, roots v = s3/s1.
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
  // y = v + aQ/4 → depressed quartic in y. (Note: substitute v = y - aQ/4.)
  const double pD = bQ - 3.0*aQ*aQ/8.0;
  const double qD = aQ*aQ*aQ/8.0 - aQ*bQ/2.0 + cQ;
  const double rD = -3.0*aQ*aQ*aQ*aQ/256.0 + aQ*aQ*bQ/16.0 - aQ*cQ/4.0 + dQ;

  double roots[4];
  const int nr = solve_depressed_quartic(pD, qD, rD, roots);
#if KLT_VO_P3P_DEBUG
  printf("  G coeffs: G4=%.6g G3=%.6g G2=%.6g G1=%.6g G0=%.6g\n", G4, G3, G2, G1, G0);
  printf("  depressed: pD=%.6g qD=%.6g rD=%.6g\n", pD, qD, rD);
  printf("  nr=%d\n", nr);
  for (int i = 0; i < nr; ++i) printf("    y[%d]=%.6g -> v=%.6g\n", i, roots[i], roots[i] - aQ/4.0);
#endif
  if (nr <= 0) return 0;

  int n_sol = 0;
  for (int k = 0; k < nr && n_sol < 4; ++k) {
    // v = s3/s1 by construction of the resultant above.
    const double v = roots[k] - aQ/4.0;
    if (v <= 0.0) {
#if KLT_VO_P3P_DEBUG
      printf("  k=%d v=%.6g (skip <=0)\n", k, v);
#endif
      continue;
    }

    // s1 from |X1-X3|² = b² (eq 2):  s1² (1 - 2 v cB + v²) = b².
    const double den_s1 = 1.0 + v*v - 2.0*v*cB;
    if (den_s1 <= 0.0) continue;
    const double s1 = b / sqrt(den_s1);
    const double s3 = v * s1;

    // u = s2/s1 from Q2 quadratic:
    //   (K2 - 1) u² + (2 cG - 2 K2 v cA) u + (K2 v² - 1) = 0
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
#if KLT_VO_P3P_DEBUG
    printf("  k=%d v=%.6g s1=%.6g s3=%.6g u_vals(%d): ", k, v, s1, s3, n_u);
    for (int i = 0; i < n_u; ++i) printf("%.6g ", u_vals[i]);
    printf("\n");
#endif

    for (int iu = 0; iu < n_u && n_sol < 4; ++iu) {
      const double u = u_vals[iu];
      if (u <= 0.0) continue;
      const double s2 = u * s1;

      // Consistency: |X2-X3|² = a² (eq 3). Should hold automatically since the
      // resultant root means Q1, Q2 share a common u — but verify to reject
      // numerical noise / spurious roots from the quartic.
      const double chk_a = s2*s2 + s3*s3 - 2.0*s2*s3*cA;
#if KLT_VO_P3P_DEBUG
      printf("    u=%.6g s2=%.6g chk_a=%.6g (want %.6g)\n", u, s2, chk_a, a2);
#endif
      if (fabs(chk_a - a2) > 1e-3 * a2 + 1e-6) continue;

      // Camera-frame points along the bearings.
      double Xc[3][3] = {
        {s1*f[0][0], s1*f[0][1], s1*f[0][2]},
        {s2*f[1][0], s2*f[1][1], s2*f[1][2]},
        {s3*f[2][0], s3*f[2][1], s3*f[2][2]},
      };

      double R[9], t[3];
      if (!horn_3pt(Pw, Xc, R, t)) {
#if KLT_VO_P3P_DEBUG
        printf("    horn_3pt failed\n");
#endif
        continue;
      }

      // Cheirality: all three reprojected points must have z > 0.
      // R Pw[k] + t should equal Xc[k]; verify by reprojecting all 3.
      bool ok = true;
      for (int p = 0; p < 3 && ok; ++p) {
        const double px = R[0]*Pw[p][0] + R[1]*Pw[p][1] + R[2]*Pw[p][2] + t[0];
        const double py = R[3]*Pw[p][0] + R[4]*Pw[p][1] + R[5]*Pw[p][2] + t[1];
        const double pz = R[6]*Pw[p][0] + R[7]*Pw[p][1] + R[8]*Pw[p][2] + t[2];
        if (pz <= 0.0) { ok = false; break; }
        const double ex = px - Xc[p][0];
        const double ey = py - Xc[p][1];
        const double ez = pz - Xc[p][2];
        if (ex*ex + ey*ey + ez*ez > 1e-6) { ok = false; break; }
      }
#if KLT_VO_P3P_DEBUG
      printf("    cheirality=%s\n", ok ? "ok" : "fail");
#endif
      if (!ok) continue;

      for (int j = 0; j < 9; ++j) R_out[n_sol][j] = R[j];
      for (int j = 0; j < 3; ++j) t_out[n_sol][j] = t[j];
      ++n_sol;
    }
  }
  return n_sol;
}

}  // namespace p3p
}  // namespace slamko_vio

#endif  // KLT_VO_P3P_SOLVER_HPP_

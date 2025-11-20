// -----------------------------------------------------------------------------
// StaticForcesEx: Robust static-tension solver for Hangprinter
// - Any N >= 3 (HP4, HP5, SkyCam-like, or 3-cable planar)
// - Regularized pseudoinverse + nullspace projection
// - Hard per-cable caps + non-negativity
// - Drives tensions toward per-cable targets without breaking equilibrium
// - Optional gravity (ignore or include)
// - Reports achieved net support and residual (so coplanar cases are graceful)
// -----------------------------------------------------------------------------

#include <cmath>
#include <cassert>
#include <iostream>
#include <algorithm>
#include <limits>

#include "flex_via_pseudoinverse.hpp"
static inline float dot(const Vec3& a, const Vec3& b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline float norm(const Vec3& a){ return std::sqrt(dot(a,a)); }
static inline Vec3 unit_or_zero(const Vec3& a){
    float n = norm(a);
    if (n > 0.f) { float inv = 1.0f/n; return {a.x*inv, a.y*inv, a.z*inv}; }
    return Vec3();
}

// ------------------ 3x3 inverse (tiny helper) -------------------
static inline bool invert3x3(const float M[3][3], float Minv[3][3], float eps = 1e-9f)
{
    float a = M[0][0], b = M[0][1], c = M[0][2];
    float d = M[1][0], e = M[1][1], f = M[1][2];
    float g = M[2][0], h = M[2][1], i = M[2][2];

    float A =  (e*i - f*h);
    float B = -(d*i - f*g);
    float C =  (d*h - e*g);
    float D = -(b*i - c*h);
    float E =  (a*i - c*g);
    float F = -(a*h - b*g);
    float G =  (b*f - c*e);
    float H = -(a*f - c*d);
    float I =  (a*e - b*d);

    float det = a*A + b*B + c*C;
    if (std::fabs(det) < eps) return false;

    float invdet = 1.0f / det;
    Minv[0][0] = A*invdet; Minv[0][1] = D*invdet; Minv[0][2] = G*invdet;
    Minv[1][0] = B*invdet; Minv[1][1] = E*invdet; Minv[1][2] = H*invdet;
    Minv[2][0] = C*invdet; Minv[2][1] = F*invdet; Minv[2][2] = I*invdet;
    return true;
}

// ------------- Build A (3xN), columns are unit cable directions --------------
static inline void build_direction_matrix(const Vec3* anchors, int N, const Vec3& mover, float* A /*3xN row-major*/)
{
    for (int j = 0; j < N; ++j) {
        Vec3 u = unit_or_zero(anchors[j] - mover);
        A[0*N + j] = u.x;
        A[1*N + j] = u.y;
        A[2*N + j] = u.z;
    }
}

// ------ Min-norm (regularized) base: T = A^T * (A A^T + λI)^-1 * F_ext -------
static inline void solve_min_norm_T(const float* A, int N, const Vec3& Fext, float lambda, float* T)
{
    float S[3][3] = {{lambda,0,0},{0,lambda,0},{0,0,lambda}};
    for (int j = 0; j < N; ++j) {
        float ax=A[0*N + j], ay=A[1*N + j], az=A[2*N + j];
        S[0][0]+=ax*ax; S[0][1]+=ax*ay; S[0][2]+=ax*az;
        S[1][0]+=ay*ax; S[1][1]+=ay*ay; S[1][2]+=ay*az;
        S[2][0]+=az*ax; S[2][1]+=az*ay; S[2][2]+=az*az;
    }
    float Sinv[3][3];
    if (!invert3x3(S, Sinv)) {
        S[0][0]+=1e-6f; S[1][1]+=1e-6f; S[2][2]+=1e-6f;
        invert3x3(S, Sinv);
    }
    float y0 = Sinv[0][0]*Fext.x + Sinv[0][1]*Fext.y + Sinv[0][2]*Fext.z;
    float y1 = Sinv[1][0]*Fext.x + Sinv[1][1]*Fext.y + Sinv[1][2]*Fext.z;
    float y2 = Sinv[2][0]*Fext.x + Sinv[2][1]*Fext.y + Sinv[2][2]*Fext.z;
    for (int j = 0; j < N; ++j) {
        float ax=A[0*N + j], ay=A[1*N + j], az=A[2*N + j];
        T[j] = ax*y0 + ay*y1 + az*y2;
    }
}

// --------- Nullspace projector: P ≈ I - A^T (A A^T + λI)^-1 A  ---------------
static inline void build_null_projector(const float* A, int N, float lambda, float* P /*N x N*/)
{
    float S[3][3] = {{lambda,0,0},{0,lambda,0},{0,0,lambda}};
    for (int j = 0; j < N; ++j) {
        float ax=A[0*N + j], ay=A[1*N + j], az=A[2*N + j];
        S[0][0]+=ax*ax; S[0][1]+=ax*ay; S[0][2]+=ax*az;
        S[1][0]+=ay*ax; S[1][1]+=ay*ay; S[1][2]+=ay*az;
        S[2][0]+=az*ax; S[2][1]+=az*ay; S[2][2]+=az*az;
    }
    float Sinv[3][3];
    if (!invert3x3(S, Sinv)) {
        S[0][0]+=1e-6f; S[1][1]+=1e-6f; S[2][2]+=1e-6f;
        invert3x3(S, Sinv);
    }
    // P = I - A^T * Sinv * A
    for (int r = 0; r < N; ++r) {
        for (int c = 0; c < N; ++c) {
            float ax=A[0*N + c], ay=A[1*N + c], az=A[2*N + c];
            float B0 = Sinv[0][0]*ax + Sinv[0][1]*ay + Sinv[0][2]*az;
            float B1 = Sinv[1][0]*ax + Sinv[1][1]*ay + Sinv[1][2]*az;
            float B2 = Sinv[2][0]*ax + Sinv[2][1]*ay + Sinv[2][2]*az;
            float arx=A[0*N + r], ary=A[1*N + r], arz=A[2*N + r];
            float Mrc = arx*B0 + ary*B1 + arz*B2;
            P[r*N + c] = (r==c ? 1.0f : 0.0f) - Mrc;
        }
    }
}

static inline void proj_nullspace(const float* P, int N, const float* v, float* out)
{
    for (int r = 0; r < N; ++r) {
        float acc = 0.f;
        for (int c = 0; c < N; ++c) acc += P[r*N + c] * v[c];
        out[r] = acc;
    }
}

static inline float alpha_towards_target(const float* T, const float* d, const float target, int N, float damp)
{
    float num=0.f, den=0.f;
    for (int i = 0; i < N; ++i){ num+=d[i]*(target-T[i]); den+=d[i]*d[i]; }
    float alpha = (den>0.f) ? (num/den) : 0.f;
    return alpha * damp;
}

static inline Vec3 applyA(const float* A, int N, const float* T)
{
    float fx=0, fy=0, fz=0;
    for (int j = 0; j < N; ++j){
        fx += A[0*N + j]*T[j];
        fy += A[1*N + j]*T[j];
        fz += A[2*N + j]*T[j];
    }
    return Vec3(fx,fy,fz);
}

/**
 * StaticForcesEx
 * @param anchors N anchors in world coords
 * @param N       cables (>=3), supports up to N<=16
 * @param mover   effector pos
 * @param cfg     config
 * @param out     output structure; caller must set out.tensions to valid float[N]
 */
void StaticForcesEx(
    const Vec3* anchors, int N,
    const Vec3& mover,
    const StaticForcesConfig& cfg,
    StaticForcesResult& out)
{
    float* T = out.tensions;
    float A[3*16];
    build_direction_matrix(anchors, N, mover, A);

    // requested external force
    out.requestedForce = Vec3(0,0,0);
    for (int i=0;i<N;++i) {
      T[i] = 0.0f;
    }
    if (!cfg.ignoreGravity) {
      out.requestedForce = Vec3(0,0,cfg.massKg*cfg.g);
      // base min-norm solution (regularized pseudoinverse)
      solve_min_norm_T(A, N, out.requestedForce, cfg.lambda, T);
    }

    if (!cfg.ignorePretension) {
      float P[16*16];
      build_null_projector(A, N, cfg.lambda, P);

      for (int it = 0; it < cfg.maxItersTarget; ++it) {
          float gradient[16] = {0};

          for (int i = 0; i < N; ++i) {
              // Gradient from the target objective: 0.5 * (T - T_target)^2
              float target_grad = 0.0;

              // If tension is beyond max, pull strongly back into acceptable range
              if (T[i] > cfg.Tmax[i]) {
                target_grad += T[i] - cfg.Tmax[i];
              }
              // If tenson is below min, pull strongly back into acceptable range
              if (T[i] < cfg.Tmin[i]) {
                target_grad += T[i] - cfg.Tmin[i];
              }
              // Always pull weakly towards the minvalue
              target_grad += 0.1*(T[i] - cfg.Tmin[i]);

              gradient[i] = target_grad;
          }
          float d[16];
          proj_nullspace(P, N, gradient, d);
          float dn=0.f;
          for (int i=0;i<N;++i) dn+=d[i]*d[i];

          // Check if we have already converged to a good solution
          if (dn < cfg.tol * cfg.tol) {
            //std::cerr << it << ", ";
            break;
          } else if (it == cfg.maxItersTarget - 1) {
            //std::cerr << "Max: " << it << ", ";
          };

          // Take a gradient descent step
          float alpha = cfg.stepDamp;
          for (int i = 0; i < N; ++i) {
              T[i] -= alpha * d[i];
          }
      }

      // Hard cap on Tmax and 0
      for (int i=0;i<N;++i){
          if (T[i] < 0.f) T[i] = 0.f;
          if (T[i] > cfg.Tmax[i]) T[i] = cfg.Tmax[i];
      }
    }

    out.achievedForce = applyA(A, N, T);
    out.residual      = out.requestedForce - out.achievedForce;
    std::cerr << out.achievedForce.x << ", " << out.achievedForce.y << ", " << out.achievedForce.z << '\n';

    out.supportedGravityFrac = 0.f;
    if (!cfg.ignoreGravity) {
        if (out.requestedForce.z > 1e-9f) out.supportedGravityFrac = out.achievedForce.z/out.requestedForce.z;
    }

    // Injects unrealistic force just to be able to spot anchors in the python plots
    // Don't copy this over into a real firmware
    //if (N > 3) {
    //  for (int i=0;i<std::min(N, 4);++i){
    //    if (std::abs(mover.x - anchors[i].x) < 50
    //        && std::abs(mover.y - anchors[i].y) < 50 ) {
    //      for (int j=0;j<N;++j){
    //        T[j] = -cfg.Tmax[j];
    //      }
    //    }
    //  }
    //}
}

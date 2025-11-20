// -----------------------------------------------------------------------------
// StaticForcesEx: Robust static-tension solver for Hangprinter (QP version)
// - Any N >= 3 (HP4, HP5, SkyCam-like, or 3-cable planar)
// - Convex bound-constrained ridge LS:
//     minimize 0.5||A T - F||^2 + 0.5*lambda*||T||^2  s.t. L <= T <= U
// - L = 0 if cfg.ignorePretension, else L = cfg.Tmin
// - U = cfg.Tmax (per cable hard cap)
// - Returns achieved support and residual; handles coplanar/near-singular cases
// -----------------------------------------------------------------------------

#include <cmath>
#include <cassert>
#include <iostream>
#include <algorithm>
#include <limits>
#include <vector>

#include "flex_via_pseudoinverse.hpp"

static inline float dot(const Vec3& a, const Vec3& b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline float norm(const Vec3& a){ return std::sqrt(dot(a,a)); }
static inline Vec3 unit_or_zero(const Vec3& a){
    float n = norm(a);
    if (n > 0.f) { float inv = 1.0f/n; return {a.x*inv, a.y*inv, a.z*inv}; }
    return Vec3();
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

template<typename T>
static inline Vec3 applyA(const float* A, int N, const T* x)
{
    double fx=0, fy=0, fz=0;
    for (int j = 0; j < N; ++j){
        fx += (double)A[0*N + j]*x[j];
        fy += (double)A[1*N + j]*x[j];
        fz += (double)A[2*N + j]*x[j];
    }
    return Vec3((float)fx,(float)fy,(float)fz);
}

// ---- Cholesky solver for small SPD systems (k x k), row-major in/out -------
static inline bool chol_decompose(std::vector<double>& G, int k)
{
    const double eps = 1e-14;
    for (int i=0;i<k;i++){
        for (int j=0;j<=i;j++){
            double s = G[i*k + j];
            for (int p=0;p<j;p++) s -= G[i*k + p]*G[j*k + p];
            if (i==j){
                if (s <= eps) s = eps;
                G[i*k + j] = std::sqrt(s);
            } else {
                G[i*k + j] = s / G[j*k + j];
            }
        }
        for (int j=i+1;j<k;j++) G[i*k + j] = 0.0; // zero upper
    }
    return true;
}

static inline void chol_solve(const std::vector<double>& L, int k,
                              const std::vector<double>& b, std::vector<double>& x)
{
    std::vector<double> y(k,0.0);
    // Solve L y = b
    for (int i=0;i<k;i++){
        double s = b[i];
        for (int p=0;p<i;p++) s -= L[i*k + p]*y[p];
        y[i] = s / L[i*k + i];
    }
    // Solve L^T x = y
    x.assign(k,0.0);
    for (int i=k-1;i>=0;i--){
        double s = y[i];
        for (int p=i+1;p<k;p++) s -= L[p*k + i]*x[p];
        x[i] = s / L[i*k + i];
    }
}

// -------- Bound-constrained ridge LS: H = A^T A + lambda I, f = A^T F --------
// minimize 0.5 T^T H T - f^T T, s.t. L <= T <= U  (equivalent to problem above)
static inline void solve_box_ridge_ls(const float* A, int N,
                                      const Vec3& F, double lambda,
                                      const double* L, const double* U,
                                      int max_iters, double tol,
                                      double* T_out)
{
    // Build H and f
    std::vector<double> H(N*N, 0.0);
    std::vector<double> f(N, 0.0);

    for (int i=0;i<N;i++){
        double aix = (double)A[0*N + i], aiy = (double)A[1*N + i], aiz = (double)A[2*N + i];
        f[i] = aix*F.x + aiy*F.y + aiz*F.z;
        for (int j=0;j<=i;j++){
            double ajx = (double)A[0*N + j], ajy = (double)A[1*N + j], ajz = (double)A[2*N + j];
            double dot = aix*ajx + aiy*ajy + aiz*ajz;
            double v = dot + (i==j ? lambda : 0.0);
            H[i*N + j] = v;
            H[j*N + i] = v;
        }
    }

    // Initial guess: unconstrained solution clamped to [L,U]
    // Solve H t = f
    std::vector<double> Lfull = H; // copy for factorization
    chol_decompose(Lfull, N);
    std::vector<double> t(N, 0.0);
    chol_solve(Lfull, N, f, t);
    for (int i=0;i<N;i++){
        double li = L ? L[i] : 0.0;
        double ui = U ? U[i] : std::numeric_limits<double>::infinity();
        if (ui < li) ui = li; // safety
        t[i] = std::min(std::max(t[i], li), ui);
    }

    // Active-set / free-set loop
    std::vector<int> free_idx; free_idx.reserve(N);
    std::vector<double> g(N, 0.0);

    auto projected_grad_norm = [&](const std::vector<double>& x)->double{
        double s2 = 0.0;
        for (int i=0;i<N;i++){
            double li = L ? L[i] : 0.0;
            double ui = U ? U[i] : std::numeric_limits<double>::infinity();
            double gi = 0.0;
            // gi = (H x - f)_i
            for (int j=0;j<N;j++) gi += H[i*N + j]*x[j];
            gi -= f[i];
            // Projected gradient: zero out infeasible components
            bool atL = (x[i] <= li + 1e-12);
            bool atU = (x[i] >= ui - 1e-12);
            double pgi = gi;
            if (atL && gi > 0) pgi = 0.0;        // can't go negative direction
            if (atU && gi < 0) pgi = 0.0;        // can't go positive direction
            s2 += pgi*pgi;
        }
        return std::sqrt(s2);
    };

    for (int it=0; it<max_iters; ++it){
        // Gradient g = H t - f
        for (int i=0;i<N;i++){
            double gi = 0.0;
            for (int j=0;j<N;j++) gi += H[i*N + j]*t[j];
            g[i] = gi - f[i];
        }

        // Build free set: those not at bounds OR that violate KKT on a bound
        free_idx.clear();
        for (int i=0;i<N;i++){
            double li = L ? L[i] : 0.0;
            double ui = U ? U[i] : std::numeric_limits<double>::infinity();
            bool atL = (t[i] <= li + 1e-12);
            bool atU = (t[i] >= ui - 1e-12);
            bool violateL = atL && (g[i] < -tol);   // KKT requires g[i] >= 0 at L
            bool violateU = atU && (g[i] >  tol);   // KKT requires g[i] <= 0 at U
            if ((!atL && !atU) || violateL || violateU) free_idx.push_back(i);
        }

        // Check for optimality via projected gradient
        double pgn = projected_grad_norm(t);
        if (pgn <= tol) break;

        if (free_idx.empty()){
            // Nothing free but not optimal -> free the most violating bound
            int k = 0;
            double best = 0.0;
            for (int i=0;i<N;i++){
                double li = L ? L[i] : 0.0;
                double ui = U ? U[i] : std::numeric_limits<double>::infinity();
                bool atL = (t[i] <= li + 1e-12);
                bool atU = (t[i] >= ui - 1e-12);
                double viol = 0.0;
                if (atL) viol = std::max(0.0, -g[i]);
                if (atU) viol = std::max(0.0,  g[i]);
                if (viol > best){ best = viol; k = i; }
            }
            free_idx.push_back(k);
        }

        // Solve Newton step on free set: H_FF p_F = -g_F
        int k = (int)free_idx.size();
        std::vector<double> Hff(k*k, 0.0), gf(k,0.0), pf(k,0.0);
        for (int p=0;p<k;p++){
            int ip = free_idx[p];
            gf[p] = g[ip];
            for (int q=0;q<k;q++){
                int iq = free_idx[q];
                Hff[p*k + q] = H[ip*N + iq];
            }
        }
        chol_decompose(Hff, k);
        for (int i=0;i<k;i++) gf[i] = -gf[i];
        chol_solve(Hff, k, gf, pf);

        // Compute step length to remain inside [L,U]
        double alpha = 1.0;
        for (int idx=0; idx<k; ++idx){
            int i = free_idx[idx];
            double pi = pf[idx];
            if (std::abs(pi) < 1e-16) continue;
            double li = L ? L[i] : 0.0;
            double ui = U ? U[i] : std::numeric_limits<double>::infinity();
            if (pi > 0.0){
                double amax = (ui - t[i]) / pi;
                if (amax < alpha) alpha = std::max(0.0, amax);
            } else if (pi < 0.0){
                double amax = (li - t[i]) / pi; // pi<0 -> denominator negative -> positive amax
                if (amax < alpha) alpha = std::max(0.0, amax);
            }
        }

        // Take step
        for (int idx=0; idx<k; ++idx){
            int i = free_idx[idx];
            t[i] += alpha * pf[idx];
        }
        // Clamp (handles numerical drift to exactly hit bounds)
        for (int i=0;i<N;i++){
            double li = L ? L[i] : 0.0;
            double ui = U ? U[i] : std::numeric_limits<double>::infinity();
            if (ui < li) ui = li;
            if (t[i] < li) t[i] = li;
            if (t[i] > ui) t[i] = ui;
        }
    }

    for (int i=0;i<N;i++) T_out[i] = t[i];
}

/**
 * StaticForcesEx
 * @param anchors N anchors in world coords
 * @param N       cables (>=3), supports up to N<=16
 * @param mover   effector pos
 * @param cfg     config (expects .ignoreGravity, .massKg, .g, .lambda,
 *                          .ignorePretension, .Tmin[], .Tmax[], .tol, .maxItersTarget)
 * @param out     output; caller must set out.tensions to valid float[N]
 */
void StaticForcesEx_qp(
    const Vec3* anchors, int N,
    const Vec3& mover,
    const StaticForcesConfig& cfg,
    StaticForcesResult& out)
{
    float* T = out.tensions;
    float A[3*16];
    build_direction_matrix(anchors, N, mover, A);

    // Requested external force (e.g., gravity)
    out.requestedForce = Vec3(0,0,0);
    if (!cfg.ignoreGravity) {
        out.requestedForce = Vec3(0, 0, cfg.massKg * cfg.g);
    }

    // Bounds
    std::vector<double> L(N, 0.0), U(N, std::numeric_limits<double>::infinity());
    for (int i=0;i<N;i++){
        double li = cfg.ignorePretension ? 0.0 : (double)cfg.Tmin[i];
        double ui = (double)cfg.Tmax[i];
        if (ui < li) ui = li; // ensure feasibility
        L[i] = li; U[i] = ui;
    }

    // Solve convex QP
    std::vector<double> Td(N, 0.0);
    solve_box_ridge_ls(
        A, N,
        out.requestedForce,
        (double)cfg.lambda,
        L.data(), U.data(),
        cfg.maxItersTarget,
        (double)cfg.tol,
        Td.data()
    );

    // Copy to output (float)
    for (int i=0;i<N;i++) T[i] = (float)Td[i];

    // Achieved force and residual
    out.achievedForce = applyA(A, N, Td.data());
    out.residual      = out.requestedForce - out.achievedForce;
    //std::cerr << out.achievedForce.x << ", " << out.achievedForce.y << ", " << out.achievedForce.z << '\n';

    out.supportedGravityFrac = 0.f;
    if (!cfg.ignoreGravity) {
        if (out.requestedForce.z > 1e-9f)
            out.supportedGravityFrac = out.achievedForce.z / out.requestedForce.z;
    }

    // (Optional) debug anchor marking for Python plots
    // TODO: Make a command line flag to place out proper markings in the plot using some
    // matplotlib built in marker feature, not by changing the actual data like this.
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

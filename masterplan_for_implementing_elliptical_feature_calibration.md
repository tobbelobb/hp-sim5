# Master Plan: Elliptical Feature Calibration for Cable-Driven Robots

## Executive Summary

This document outlines a calibration approach for cable-driven parallel robots that exploits a fundamental geometric property: when the effector is constrained to move along a 1-DOF path (by fixing N-1 cable lengths), the relationship between any two cable lengths squared traces an **ellipse**. By fitting ellipses to measurement data and optimizing anchor positions to match theoretical ellipses, we achieve noise-robust calibration that generalizes across machine configurations. Cost comparisons use a canonical geometric parameterization `(x0, y0, a, b, θ)` (with `a >= b` and θ wrapped into a fixed range) rather than raw `(A,B,C,D,E,F)` coefficients to avoid scale and conditioning pitfalls.

## Supported Machine Configurations

| Configuration | Anchors | Dimensions | Type            | Constraints for 1-DOF |
|---------------|---------|------------|-----------------|-----------------------|
| Slideprinter  |       3 |         2D | Overconstrained | Fix 1 cable           |
| Hangprinter   |       4 |         3D | Overconstrained | Fix 2 cables          |
| Hangprinter   |       5 |         3D | Overconstrained | Fix 2 cables          |
| CubeCorners   |       8 |         3D | Overconstrained | Fix 2 cables          |
| SkyCam        |       4 |         3D | Underactuated   | Fix 2 cables          |

## The Mathematical Foundation

### The Ellipse Invariant

When the effector is constrained to move along a circular arc in Cartesian space:

1. **2D Case (Slideprinter):** Fixing 1 cable length constrains the effector to a circle centered at that anchor.

2. **3D Case (Hangprinter, etc.):** Fixing 2 cable lengths constrains the effector to the intersection of two spheres, which is a circle in 3D space.

For any cable connecting a fixed anchor to a point on this circle, the squared length follows:

$$L^2(\phi) = K + M \cos(\phi) + N \sin(\phi)$$

where $\phi$ is the angular position on the circle, and K, M, N are constants depending on geometry.

Since both the drive cable ($L_{drive}^2$) and sensor cable ($L_{sensor}^2$) follow this form with respect to the same angle $\phi$:

$$X(\phi) = L_{drive}^2 = K_D + M_D \cos\phi + N_D \sin\phi$$
$$Y(\phi) = L_{sensor}^2 = K_S + M_S \cos\phi + N_S \sin\phi$$

This is the parametric equation of an **ellipse** in the $(X, Y)$ plane.

### Why Ellipses?

The general conic equation $Ax^2 + Bxy + Cy^2 + Dx + Ey + F = 0$ has 5 degrees of freedom (up to scale). An ellipse-specific constraint ($B^2 - 4AC < 0$) ensures we get a closed curve.

## Calibration Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     PHASE 1: DATA CAPTURE                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Raw Sweep Data (ΔL only, origin-relative)                      │
│  ┌──────────────┐                                               │
│  │ (l_d, l_s)   │   Store sweeps + metadata                     │
│  │ (l_d, l_s)   │   (optional lightweight QC)                   │
│  │ ...          │                                               │
│  │ (l_d, l_s)   │                                               │
│  └──────────────┘                                               │
│                                                                 │
├─────────────────────────────────────────────────────────────────┤
│                     PHASE 2: OPTIMIZATION                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Anchor Guess     Reconstruct Abs.     Ellipse Fitting          │
│  ┌──────────────┐ Lengths per Sweep    (per guess)              │
│  │ A_0, A_1,... │ ┌──────────────────┐ ┌──────────────────┐     │
│  │              │ │ L_abs = ||A|| +  │ │ ΔL→(L²)→coeffs   │     │
│  │              │ │  ΔL_measured     │ │ (Maini/Stefano) │     │
│  └──────────────┘ └──────────────────┘ └──────────────────┘     │
│         │                        │                   │          │
│         │                        ▼                   │          │
│         │                 Theoretical Ellipse        │          │
│         │                 (anchors, roles)           │          │
│         │                        │                   │          │
│         └────────────────────────┴───────────────────┘          │
│                              │                                  │
│                              ▼                                  │
│                    Cost Function + Optimizer                    │
│                     (SLSQP / L-BFGS-B)                          │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Data Collection Strategy

### Sweep Definition

A **sweep** is a sequence of measurements where:
- **Fixed cables**: 2 cables (for 3D) or 1 cable (for 2D) are held at constant length
- **Drive cable**: 1 cable is actively varied over a range
- **Sensor cable**: 1 cable is in torque mode (passively actuated), measuring its length

**Length reference model:** During data collection we have encoder deltas but we do *not* know anchor locations, so all recorded lengths (`fixed_lengths`, `l_drive`, `l_sensor`) are relative to the length when the mover sat at the origin (encoders zeroed). Phase 2 must reconstruct absolute lengths as `L_abs(φ) = ||A_i - origin|| + ΔL_measured(φ)` using the current anchor guess; the cost function should therefore either re-fit ellipses on these reconstructed lengths or compare against theoretical ellipses built from the guessed absolute lengths. Keep the raw encoder-driven measurements intact (no baked-in spool, sag, or flex corrections) so future physics models can re-interpret the same dataset when we extend beyond the initial "rigid + no-buildup" assumption.

### Generalized Sweep Configuration

For an N-anchor system in D dimensions:
- **Constraints needed for 1-DOF**: D-1 fixed cables
- **Free cables**: N - (D-1) = N - D + 1

| System        | N | D | Fixed | Drive     | Sensor    | Permutations         |
|---------------|---|---|-------|-----------|-----------|----------------------|
| Slideprinter  | 3 | 2 |     1 |         1 |         1 |      3 × 2 × 1 = 6   |
| Hangprinter 4 | 4 | 3 |     2 |         1 |         1 | C(4,2) × 2 × 1 = 12  |
| Hangprinter 5 | 5 | 3 |     2 |         1 | 2 choices | C(5,2) × 3 × 2 = 60  |
| CubeCorners   | 8 | 3 |     2 |         1 | 5 choices | C(8,2) × 6 × 5 = 840 |

In practice, we don't need all permutations. A well-distributed subset provides sufficient constraints.
Pay special attention to which anchor/drives are drive/fixed/sense on Hangprinter configurations, because they only have one motor pulling upwards.
We'll want at least one high anchor/drive to be either fixed or drive, in order to avoid the mover dropping down due
gravity potentially overwhelming the sensor's constant torque.
- Capture enough metadata to visualize sweep observability: retain timestamps/drive progress so later QC plots can reveal which arc regions were sampled most densely and how noise/residuals vary along the path.

### Recommended Sweep Patterns

#### Slideprinter (3 anchors)
```
Sweep Set A: Fix anchor 0
  - Drive 1, Sense 2
  - Drive 2, Sense 1

Sweep Set B: Fix anchor 1
  - Drive 0, Sense 2
  - Drive 2, Sense 0

Sweep Set C: Fix anchor 2
  - Drive 0, Sense 1
  - Drive 1, Sense 0
```

#### Hangprinter (4 anchors, Drive 3 is high, so it never takes the "Sense" role)
```
Fix anchors {0,1}: Drive 3, Sense 2
Fix anchors {0,2}: Drive 3, Sense 1
Fix anchors {0,3}: Drive 1, Sense 2 and Drive 2, Sense 1
Fix anchors {1,2}: Drive 3, Sense 0
Fix anchors {1,3}: Drive 0, Sense 2 and Drive 2, Sense 0
Fix anchors {2,3}: Drive 0, Sense 1 and Drive 1, Sense 0
```

#### Hangprinter (5 anchors)
Same as 4-anchor but with anchor I (index 4) being the high anchor.
Therefore, Hangprinter 5's drive 3 can be used as Fixed, Drive or Sense,
while HP5's drive 4 can only act as Fixed or Drive.

#### CubeCorners (8 anchors)
Select a representative subset:
- Choose fixing pairs that span the workspace geometry
- Prioritize combinations that exercise different regions of the workspace
- Make sure the mover is sufficiently light to carry the mover with only torque mode,
  or make sure enough high anchors are either fixed or drive so mover doesn't sag down
  due to gravity.

## JSON Data Format

```json
{
  "version": "1.0",
  "machine_type": "slideprinter",
  "num_anchors": 3,
  "dimensions": 2,
  "timestamp": "2024-01-15T10:30:00Z",
  "sweeps": [
    {
      "id": "sweep_001",
      "fixed_anchors": [1],
      "fixed_lengths": [12.5],
      "drive_anchor": 0,
      "sensor_anchor": 2,
      "drive_range": {"start": -100, "end": 100, "unit": "mm"},
      "data_points": [
        {"l_drive": -100.0, "l_sensor": -50.2, "timestamp_ms": 0},
        {"l_drive": -95.0, "l_sensor": -53.1, "timestamp_ms": 50},
        ...
      ],
      "metadata": {
        "feed_rate": 2000,
        "torque": 0.05,
        "settle_ms": 100
      }
    },
    ...
  ]
}
```

Store raw sweeps; ellipse fitting happens inside the optimizer for each anchor guess after reconstructing absolute lengths. All length-valued fields represent encoder-derived deltas from the origin; absolute lengths must be reconstructed during optimization using anchor guesses.

## Advantages Over Differential/Gradient Methods

1. **Noise Immunity**: Fitting a rigid geometric shape to 100+ points averages out zero-mean noise (vibration, encoder quantization).

2. **Time Independence**: No reliance on synchronized timing between motor movement and sensor reading. As long as (l_drive, l_sensor) pairs are correct, the ellipse holds.

3. **Global Constraint**: Captures the global curvature of the workspace, not just local gradients.

4. **Implicit Quality Control**: If data doesn't form an ellipse (cable slack, snag, etc.), the regression residual explodes, automatically flagging bad data.

5. **Data Compression**: Each sweep with 100+ points compresses to 5 ellipse coefficients, dramatically reducing optimization dimensionality.

## Implementation Substeps

The implementation is divided into 7 substeps:

1. **Sweep Data Structure & JSON Format** - Define the generalized data structures
2. **Data Collector Script** - Modify `collect_encoder_data.mjs` or create `collect_sweep_data.mjs`
3. **Ellipse Fitting Module** - Implement Maini & Eliseo Stefano method in Python
4. **Theoretical Ellipse Projection** - Forward math to predict ellipses from anchor positions
5. **Feature Cost Function** - Link observed and predicted ellipses
6. **Visualization Module** - Debug plots for ellipse fitting and optimization
7. **Integration with simulation.py** - Merge into existing calibration workflow

## Success Criteria

1. **Synthetic Data Test**: Generate exact sweeps from known anchors; verify cost = 0 when comparing canonical `(x0, y0, a, b, θ)` tuples.
2. **Perturbation Test**: Small anchor perturbations produce smoothly increasing cost.
3. **Noise Robustness Test**: Add Gaussian noise to synthetic data; recover anchors within tolerance.
4. **Real Hardware Validation**: Run on physical Slideprinter; compare to known geometry.
5. **Convergence Speed**: Solver converges faster than point-based method with similar data volume.

## Risk Assessment

| Risk                                    | Mitigation                                     |
|-----------------------------------------|------------------------------------------------|
| Near-degenerate ellipses (near-circles) | Multiple sweeps with diverse configurations    |
| Cable slack during sweep                | QC residual check; discard bad sweeps          |
| Numerical instability in fitting        | Use Maini's method (guaranteed ellipse) and compare in canonical `(x0, y0, a, b, θ)` space to avoid scale drift |
| Local minima in optimization            | Multiple random restarts; good initial guess   |
| 8-anchor complexity                     | Start with 3/4 anchors; scale up incrementally |

## References

1. Maini, Eliseo Stefano. "Enhanced direct least square fitting of ellipses." International Journal of Pattern Recognition and Artificial Intelligence 20.06 (2006): 939-953. (Enhanced_Direct_Least_Square_Fitting_of_Ellipses.pdf)
2. "A Novel Calibration Algorithm for Cable-Driven Parallel Robots with Application to Rehabilitation" (applsci-09-02182-v2.pdf)
3. Current codebase: `simulation.py`, `collect_encoder_data.mjs`

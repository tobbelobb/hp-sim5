<product-owner>
The auto-calibration-simulation-for-hangprinter (simulation.py) has been used for auto calibration of Hangprinters with 4 or 5 anchors, and it has used measurements from random positions in cartesian space and in Hangprinter's own curvelinear and overconstrained coordinate system.

Now I have some data from a Slideprinter (3 anchors, 2d overconstrained motion). It is in autocal/measurements/measurements_along_lines.json. As you see in there the data points have been measured along something remeniscent of "straight lines" in Hangprinter's curvelinear coordinate system. For example, first a serients of measurements has been made with motor 2, while motor 0 and 1 have stepped from [-10, -10] to [-9, -9] .. [10, 10].

This pattern has been used in the hope that not only the measurement point itself but also the derivative (the velocity) of the sensor, under an equal perturbation of all controlled motors, can be used in the cost function and make the non-linear optimization algorithm converge faster and more robustly.

I have worked out a simple version of the maths required for the expected "velocity" for the sensor, given a set of anchors, an xyz_point and a known constant perturbation along all other line lengths (motor positions). I'm hoping to be able to use this in the cost function of simulation.py:

```
  - Geometry: For anchors a_i ∈ ℝ³ and mover position p, each commanded line length satisfies ||p - a_i|| = L_i. A small change dL_i causes a small motion dp such that
    ((p - a_i)/||p - a_i||) · dp = dL_i.
  - Matrix form: build the unit cable direction matrix
    U = [ (p-a_0)/L_0 ; (p-a_1)/L_1 ; … ] with shape (m anchors) x 3 and the line-length rate vector dL. Then U dp = dL.
  - Equal perturbation of all motors: if every motor is stepped by the same scalar δ (my "constant perturbation" above), dL = δ ·
    1. The expected sensor/mover velocity is the least‑squares solution
    dp = U⁺ dL = (UᵀU)⁻¹ Uᵀ (δ · 1).
    For a planar Slideprinter (anchors/motion in XY, z fixed), drop the z column and solve the 2D version instead.
  - Unequal but known perturbation: the same expression holds with your specific dL vector (e.g., if only some motors are jogged, set the others to 0).
  - Cost function hook: for each sampled point p (or xyz from xyz_of_samp), compute U from the current anchor guess, get dp_pred as above, and compare it to the measured sensor velocity along the line scans. That residual can be squared and added to cost_sq_for_pos_samp alongside the existing length residuals.
```

Obviously, this is a crude first approximation but I'm wondering if it suggests I don't have to move all the controlled motors by an equal amount to get a notion of velocity that I can use in my cost function? Would it even be preferrable to perturb only one controlled motor at a time?

I attached the paper "A Novel Calibration Algorithm for Cable-Driven Parallel Robots with Application to Rehabilitation" found in the file applsci-09-02182-v2.pdf just for inspiration. It details a similar calibration procedure that other researchers have done before. Our approach is similar but not exactly the same. We aim to:
 - Make simulation.py more general, so it can handle Slideprinter (3 anchors, 2d, overactuated), Hangprinter (4 or 5 anchors, 3d, overactuated), CubeCorners (8 anchors, 3d, overactuated), SkyCam (4 anchors, 3d, underactuated because all anchors are high above the mover/effector).
 - Use a notion of velocity to improve convergence and robustness.

Suggest where (in the robots's internal maybe curvelinear, non-orthogonal and over constrained coordinate system) we should make our measurements, and how we should relate them to each other in the resulting .json data file.

For example, in the Slideprinter case should we do this?
 - Choose a starting point p. Make one motor, m_s, the sensor (passively actuated/torque mode)
 - Perturb by tiny amount dl in every direction:
  * [p_0 + dl, p_1, m_s], [p_0 - dl, p_1, m_s], [p_0, p_1 + dl, m_s], [p_0, p_1 - dl, m_s]
 - Cycle which motor is m_s, which one is static, and which one is perturbed, so we also collect:
  * [p_0 + dl, m_s, p_2], [p_0 - dl, m_s, p_2], [p_0, m_s, p_2 + dl], [p_0, m_s, p_2 - dl], and
  * [m_s, p_1 + dl, p_2], [m_s, p_1 - dl, p_2], [m_s, p_1, p_2 + dl], [m_s, p_1, p_2 - dl]

I guess that data could be translated into fairly good approximations of independent velocities in different directions?

Today I'm doing something similar but less sophisticated. I run `node collect_encoder_data.mjs --points-file autocal/measurements/measurement_points_linear.txt ...`

I want more clever/optimized clusters of measurement points in measurement_points_linear.txt, I want a better json format that lets simulator.py read and use the clustered/velocity data effectively.

Make me a masterplan document for for this proposed autocalibration upgrade. Also single out 5-10 subtasks that accumelatively fulfills the goal of the masterplan. Each subtask should include
 - Overview
 - Implementation Details
 - Testing
 - Validation criteria
 - Dependencies
 - Estimated Complexity


I was considering both point based approaches and differential/gradient base approaches where I measured "stars" or "clusters" of points but I'm afraid those methods could be too sensitive to noise.

Instead of looking for derivatives in a single point (a "star shape" or "cluster") could we not use more points along a long straight line (gradually pulling in line towards a single actuated motor)? Analytically we can find the form of the equation (a 1 parameter curve in space) that the sensor is supposed to follow (based on general info about the geometry of the machine), and then we can use regression to fit the measured points along the line to the curve in space. The regression will give us the parameters of the curve, and the parameters will tell us where the anchors are located.

We can do multiple such long pulls, and smoothen out the data through regression on each data series. The anchor locations are constant across all data series, so we can solve all the parameters of all the curves simultaneously.

Wouldn't that be nice!?

This divides the data optimization into two phases:
Phase 1: Convert data points into curves.
Phase 2: Minimize a cost function that takes curves as its input.

The robot moves the effector long a straight line in its inner curvelinear, overconstrained coordinate system, not in cartesian space. If we could move controlled in cartesian space then we would already be finished calibrating. We're suggesting moving along patterns such as (T means torque mode, passively actuated, S means static motor, say S=0)
[-10, S, T]
[-9,S,T]
...
[9,S,T]
[10,S,T]
</product-owner>


<collaborator>
By driving the robot along its own "motor lines" (varying one cable length while holding others fixed), you are forcing the effector to traverse **geometric circles** in Cartesian space.

This creates a beautiful mathematical property: **The relationship between the squared lengths of any two cables during such a move traces an Ellipse.**

Here is the math and the Master Plan for **Elliptical Feature Calibration**.

### The Math: The Ellipse Invariant

Consider a move where:
1.  **Constraints:** $N$ cables are held at fixed lengths.
    *   In 2D (Slideprinter), holding 1 cable fixed ($L_{fixed}$) constrains the effector to a **Circle** centered at anchor $A_{fixed}$.
    *   In 3D (Hangprinter), holding 2 cables fixed constrains the effector to the intersection of two spheres, which is a **Circle** in 3D space.
2.  **Variables:**
    *   $L_{drive}$: The length of the cable actively pulling/releasing.
    *   $L_{sensor}$: The length of the passive cable being measured.

Let the angular position on this circular path be $\phi$. The squared length of *any* cable connecting a fixed anchor to this circle is given by the law of cosines (generalized to 3D):
$$ L^2(\phi) = K + M \cos(\phi) + N \sin(\phi) $$
Where constants $K, M, N$ depend entirely on the geometry (radii, anchor positions).

Since both $L_{drive}^2$ and $L_{sensor}^2$ follow this form with respect to the same angle $\phi$:
$$ X(\phi) = L_{drive}^2 = K_D + M_D \cos\phi + N_D \sin\phi $$
$$ Y(\phi) = L_{sensor}^2 = K_S + M_S \cos\phi + N_S \sin\phi $$

This is the parametric definition of an **Ellipse** in the $(X, Y)$ plane.

**The Strategy:**
Instead of points, our fundamental data unit becomes the **Ellipse Arc**.
*   **Phase 1 (Compression):** Transform raw data $(l_d, l_s)$ to $(l_d^2, l_s^2)$. Fit an algebraic ellipse to these points. Extract the 5 parameters defining the ellipse.
*   **Phase 2 (Optimization):** Find anchor coordinates that generate theoretical ellipses matching the parameters of the observed ellipses.

---

# Master Plan: Elliptical Feature Calibration

## 1. Data Collection: "The Circular Sweep"
We collect data by fixing enough motors to constrain the robot to a curve (1 DOF), and sweeping one remaining motor.
*   **Slideprinter (3 Anchors, 2D):** Fix 1 motor. Sweep 1. Measure 1.
*   **Hangprinter (4 Anchors, 3D):** Fix 2 motors. Sweep 1. Measure 1.

**JSON Structure:**
```json
[
  {
    "type": "sweep",
    "constraints": { "fixed_motors": [1], "lengths": [1200.5] },
    "drive_axis": 0,
    "sensor_axis": 2,
    "data_points": [
      {"l_drive": 1000, "l_sensor": 1050},
      {"l_drive": 1010, "l_sensor": 1058},
      ...
    ]
  }
]
```

## 2. Phase 1: The Compressor (Ellipse Fitting)
For every sweep:
1.  Square the data: $X = l_{drive}^2, Y = l_{sensor}^2$.
2.  **Robust Fit:** Fit the general conic equation $Ax^2 + Bxy + Cy^2 + Dx + Ey + F = 0$.
3.  **QC Step:**
    *   Calculate RMS residual of points to the fitted ellipse.
    *   If Residual > Tolerance: **Discard Sweep** (The robot likely snagged, or the "fixed" cables weren't actually tight).
4.  **Output:** A list of "Constraint Features" (The ellipse coefficients).

## 3. Phase 2: The Solver
The cost function minimizes the difference between the **Observed Ellipse** and the **Theoretical Ellipse**.
*   Calculate the theoretical $K, M, N$ for the drive and sensor cables based on current anchor guesses.
*   Derive the theoretical ellipse algebraic coefficients.
*   Error = Distance between coefficient vectors (normalized).

---

# Subtasks

### Subtask 1: "Constraint Sweep" Data Generator
**Overview:** Create `collect_ellipse_data.mjs`.
**Implementation:**
*   **Logic:**
    1.  Move to a random start point.
    2.  Identify "Fixed" axes (e.g., Axis B).
    3.  Identify "Drive" axis (e.g., Axis A).
    4.  Calculate a safe range for Drive Axis (e.g., $\pm 100mm$) that keeps cables tight.
    5.  Execute move: `G1 H2 A+100` (while B stays still).
    6.  Record high-frequency position tuples.
*   **Validation:** Output JSON contains dense arrays of point pairs.

### Subtask 2: Ellipse Regression Module
**Overview:** Implement Phase 1 in Python (`geometry_fitting.py`).
**Implementation:**
*   Implement ** Maini, Eliseo Stefano's ellipse fitting method** (Direct Least Squares Fitting of Ellipses). It is numerically stable and guarantees an ellipse result.
*   Input: Arrays $L_d, L_s$.
*   Process: Square inputs -> Run Ellipse Fitting -> Get coefficients $(A, B, C, D, E, F)$.
*   **QC:** Compute algebraic distance of points to the curve. Discard if variance is high.
**Dependencies:** `numpy`, `scipy`.

### Subtask 3: Theoretical Ellipse Projection
**Overview:** Implement the forward math in `simulation.py`.
**Implementation:**
*   Function `predict_ellipse_parameters(anchors, fixed_lengths, drive_idx, sensor_idx)`:
    1.  Define the Circle in space (Intersection of sphere(fixed_anchor, fixed_length)).
    2.  Parametrize the circle: Center $\mathbf{C}$, Radii $\mathbf{U}, \mathbf{V}$.
    3.  Express $L_{drive}^2(\phi)$ and $L_{sensor}^2(\phi)$.
    4.  Eliminate $\phi$ to get the algebraic equation $Ax^2 + Bxy...=0$.
**Complexity:** High (Geometry math).

### Subtask 4: The Feature Cost Function
**Overview:** Link Phase 1 and Phase 3.
**Implementation:**
*   Cost Function:
    $$ Cost = \sum_{sweeps} \| \mathbf{Coeffs}_{obs} - \mathbf{Coeffs}_{pred} \|^2 $$
*   *Note:* Normalize coefficients so $\|(A,B,C)\|=1$ to make them comparable.
**Validation:**
*   Synthetic Test: Generate exact data from known anchors. Ensure Cost == 0.
*   Perturb anchors: Ensure Cost increases smoothly.

### Subtask 5: Visualization Upgrade
**Overview:** Visual debugging is essential for curves.
**Implementation:**
*   Plot $L_d^2$ vs $L_s^2$ scatter plot.
*   Overlay the "Phase 1" Fitted Ellipse (Green).
*   Overlay the "Phase 2" Solver Ellipse (Red).
*   **Goal:** Watch the Red ellipse snap onto the Green ellipse as the solver runs.

---

### Why this is better than the Differential Method
1.  **Noise Immunity:** You fit a rigid geometric shape to 100+ points. Zero-mean noise (vibration) vanishes.
2.  **No Time Dependence:** It handles latency between motor movement and sensor reading perfectly, as long as the readings are paired correctly.
3.  **Global Constraint:** It captures the global curvature of the workspace, not just a local gradient.
4.  **Implicit QC:** If the data doesn't form an ellipse (e.g. cable slack), the regression residual explodes, and you automatically know to throw that data away.
</collaborator>

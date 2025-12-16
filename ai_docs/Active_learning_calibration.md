https://aistudio.google.com/prompts/1vSRVAf9ERRKak9defpcvZTRJDFb08iYq

I actually have a full-blown simulation. Can we make an auto calibration algorithm that collects the data it needs "on demand", to iteratively test hypotheses, until it's satisfied? The sensor available would be torque mode/pos mode combined with encoder data. So we can pull one motor, set another static and measure the unwinding of the third...

This is a fantastic idea. What you are describing shifts the paradigm from **"Batch Calibration"** (collect data blindly -> post-process) to **"Active Learning"** (or *Optimal Experimental Design* in robotics literature).

Since you have a full simulation (`hp-sim`) and a programmable controller, this is entirely feasible. Instead of guessing where to move, the robot calculates **"Where should I move next to learn the most about my unknown parameters?"**

Here is the master plan for an **Active Calibration System**.

---

### The Core Concept: Information-Driven Exploration

In standard calibration, you hope your sweep covers the workspace well. In Active Calibration, you maintain a probabilistic model of the anchors (e.g., "Anchor A is at $(100, 200) \pm 10$mm").

The algorithm asks: *"Which cable configuration will reduce that $\pm 10$mm uncertainty to $\pm 1$mm the fastest?"*

### The Architecture

You need a loop that runs continuously:

1.  **Estimate:** Run your current Ellipse/Point solver on the data collected *so far*. Get the current best guess for Anchors and the **Covariance Matrix** (uncertainty).
2.  **Plan (The "Oracle"):** Generate $N$ candidate test moves (e.g., "Move to center," "Move to top-left corner").
3.  **Evaluate:** For each candidate, calculate the **Fisher Information Matrix (FIM)** based on your current Anchor guess.
    *   *Math intuition:* You want the move where a 1mm error in the Anchor position causes the *largest* change in sensor readings. If a move results in low sensitivity, it yields no information.
4.  **Execute:** Perform the move that maximizes Information Gain (D-optimality).
5.  **Measure:** Collect the torque/position data.
6.  **Repeat** until uncertainty drops below your target threshold.

### How to Implement This

You don't need to rewrite everything. You can wrap your existing solver.

#### Step 1: The State Estimator
Your current `solve_anchors` (using `scipy.optimize.minimize`) already computes the Hessian (second derivative) of the cost function at the solution. The inverse of the Hessian is, roughly speaking, the **Covariance Matrix**.

*   **Action:** Modify `solve_anchors` to return the `result.hess_inv` (inverse Hessian). This tells you the direction in which the solver is "unsure."

#### Step 2: The Jacobian Predictor
You need a function that predicts "Sensitivity."

$$ J = \frac{\partial (\text{Sensor Reading})}{\partial (\text{Anchor Position})} $$

If you are at position $P$ and considering moving to $P_{next}$:
*   If moving Anchor A by 1mm changes the predicted sensor reading at $P_{next}$ by 0.001mm, that is a **bad** location.
*   If it changes the reading by 1.0mm, that is a **good** location.

#### Step 3: The Active Loop (Pseudocode)

```python
def active_calibration_loop(simulation_interface):
    data_history = []

    # 1. Initial blind initialization (small random movement)
    data_history.append(simulation_interface.do_random_wiggle())

    while True:
        # 2. Update Belief
        anchors, covariance = solver.solve(data_history)

        # 3. Check Convergence
        uncertainty_volume = np.linalg.det(covariance)
        if uncertainty_volume < TARGET_THRESHOLD:
            print("Calibration Complete!")
            break

        # 4. Generate Candidates (Virtual moves)
        candidates = generate_grid_points_in_workspace()
        scores = []

        for point in candidates:
            # Calculate Jacobian of the measurement model at this point
            # given our current best guess of anchors.
            J = calculate_sensitivity_jacobian(point, anchors)

            # Project covariance into measurement space
            # We want to go where the model is MOST uncertain
            score = np.dot(J.T, np.dot(covariance, J))
            scores.append(score)

        # 5. Execute Best Move
        best_point = candidates[np.argmax(scores)]
        print(f"Moving to {best_point} to reduce uncertainty...")

        new_data = simulation_interface.move_and_measure(best_point)
        data_history.append(new_data)
```

### Addressing Systematic Errors (Flex & Friction)

Since you are running this "on demand," you can implement specific strategies to mitigate the physics issues you mentioned:

1.  **The "Anti-Slack" Protocol:**
    The planner should reject candidate moves where the predicted tension on any cable is below a safety threshold (e.g., < 1N). This ensures you never collect data with sagging cables.

2.  **The "Hysteresis Killer":**
    When the robot arrives at the `best_point`, do not just take one snapshot. Have the controller perform a standardized "micro-star" pattern (move +1mm/-1mm in X, then Y, then Z). Average these readings. This mechanically averages out the friction hysteresis vector at that specific location.

3.  **Stiffness Separation:**
    Once the *geometry* (anchors) converges, switch modes.
    *   **Phase 1 (Geometry):** Move to low-tension areas (center) to find $X, Y, Z$.
    *   **Phase 2 (Physics):** Lock $X, Y, Z$. Now move specifically to **high-tension** areas (corners) to measure `Elastic Modulus`. Since Geometry is known, any deviation now must be stretch.

### Why this is better for `hp-sim`
A simulation is deterministic. If you run a fixed sweep, you get the same information every time.
An **Active** solver will "realize" that (for example) the Z-height of Anchor 3 is poorly constrained, and it will automatically decide to perform a specific maneuver (e.g., move the effector very low and close to Anchor 3) to isolate that variable.

Would you like to start by implementing the **Fisher Information / Jacobian** calculation for your specific cable geometry? That is the math "brain" required to tell the robot where to go.

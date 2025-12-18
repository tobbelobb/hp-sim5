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

----
Me telling Codex what to do
----
We're working on auto calibration of a (simulated) cabled driven robot.

We currently only have a planar 3 line robot simulated, called "Slideprinter". We collect data from it like this:
```
node scripts/collect_sweep_data.mjs --sweep-method torque-ramp --speedup 4 --trace --torque-low 0.03 --torque-min 0.03 --torque-max 0.3 --torque-step 0.05 --feed 400 --superSweepRange 600 --superSweepPoints 4
```

... which gives us a batch of known valid encoder data. One such batch can be seen in autocal/data/big_even_pattern.json.

As you can guess from some of the option names, this procedure creates movements by utilizing a special "torque mode" on the motors (one or two at a time, called the "sensor motor" and the "driver motor") combined with moves that winds or unwinds a (roughly) known amount on the third motor (the "fixed motor". By combining these movement, the script moves the effector in radial sweeps around the fixed motor within the reachable area. All three motors take turns to act as sensor, driver, and fixed, and we collect sweeps at four different distances from each motor  (that's the `--superSweepPoints 4).

We have implemented two methods that run optimizations on the data in order to find the CDPR's configuration values, most notably the anchor point positions. They are invoked like this:

```
python autocal/calibrate.py ellipse autocal/data/big_even_pattern.json
python autocal/calibrate.py point autocal/data/big_even_pattern.json
```

The "ellipse" method takes advantage of the radial pattern in the data by inferring that distances to the sensor and drive motors must follow an elliptical relationship. It fits ellipses to each sweep, takes a guess at anchor positions, which implies some guessed ellipses, and measures the distance between the observed ellipses and the guessed ellipses. Moving anchor guesses closer to the true anchor positions reduces the difference between guessed ellipses and observed ellipses.

The "point" method takes a more point-wise approach and mostly cares about distances between guessed anchors and the xyz coordinates where it guesses that the measurements were made. Improving the guessed positions of anchors and measurements decreases the difference between guessed line lengths and observed (relative) line lengths.

<feature_request>
The procedure described above is called "Batch Calibration" (collect data blindly -> post-process). We want to move into an "Active Learning Calibration" (collect data point -> improve parameter guess -> find the most valuable next position for measurement -> repeat).
</feature_request>

Ideation and initial work on this has been done in ai_docs/Active_learning_calibration.md.

For the ellipse method, the question between repetition becomes "at which fixed length should I collect the next sweep" and possibly "how high torques should the sweep be?" since this determines how close to the outskirts of the reachable area we'll move, and maybe "how many points should I collect during the next sweep?" although I expect the default number of data points per sweep to work quite well.

For the point method, the question between iterations is more direct and easy to translate from theory. It's simply "Where should I move next?". The question is mostly if we should reason in the robot's inverse coordinates (since those are the only ones we can control without knowing the anchor locations), or if we should try to make movements in cartesian coordinates once anchor position estimations start to be fairly good.

We don't have access to any other sensors than the encoders on the motors (via M569.3, see the code).

We do have access to a forward transform, in autocal/auto-calibration-simulation-for-hangprinter/hangprinter_forward_transform.py

You might find some inspiration on how to tackle various issues in ai_docs/Extended_Kalman_Filter-Based_State_Estimation_and_Adaptive_Control_of_Cable-Driven_Parallel_Robots/Extended_Kalman_Filter-Based_State_Estimation_and_Adaptive_Control_of_Cable-Driven_Parallel_Robots.md although it describes a Kalman filter rather than the non-linear optimizations we will be doing between iterations.

Another good source of inspiration is ai_docs/An_Approach_for_Predicting_the_Calibration_Accuracy/An_Approach_for_Predicting_the_Calibration_Accuracy.md.
It proposes an automatic calibration method for redundantly actuated cable-driven parallel robots (CDPRs) that exclusively utilizes the inner encoders of the driving motors, thus not requiring any external sensors.
 - Mapping Parameters for Calibration: Establishes a mapping between unknown geometric parameters, specifically the coordinates of the attachment points, and measurable parameters, which are the lengths of the redundant cables, and then uses a least squares algorithm for finding the solutions.
 - Simulation and Validation: Validates the method through simulations on a 3-DOF parallel robot driven by four cables, demonstrating precise determination of attachment point coordinates and significant improvement in positioning accuracy.

A third source of inspiration could be ai_docs/Controlling_Cable_Driven_Parallel_Robots_OperationsDeep_Reinforcement_Learning_Approach/Controlling_Cable_Driven_Parallel_Robots_OperationsDeep_Reinforcement_Learning_Approach.md
It applies a Deep Reinforcement Learning (DRL) approach, specifically the adaptive-featured Twin Delayed Deep Deterministic (TD3) policy gradient algorithm, for controlling a Cable-Driven Parallel Robot (CDPR).
 - Integrates PID in RL Framework: Integrates a Proportional-Integral-Derivative (PID) controller within the Reinforcement Learning (RL) framework to establish a unique approach to CDPR control that leverages adaptive learning capabilities.
 - Utilizes Motor Encoder Data: Receives the current kinematic state of the CDPR, which is characterized by the length and angular velocity of the cable and the target cable length, as input to the controller, where the output vector specifies the updated PID parameters that command the motor activations of the eight actuators.

Some of these papers describe cutting edge complicated calibration routines. We'll probably want to keep our algorithm simpler, but the papers cover a lot of general knowledge about the subject of Adaptive Learning for CDPRs that will apply to us for sure.

I will not give you network access, so you will not be able to collect data from the simulator yourself. Just assume that hp-sim is running at "http://localhost:5173/hp-sim5/hp-sim/?gcode_ws=ws://localhost:8790" and ask me to do the actual test runs for you when the algorithm is ready to be tested. Make your script print a calibration report and I will run tests for you and send the full report of each test run.

Give it your best! I know you can do it!

----
Codex taking a stab at this
----

• What’s Implemented

  - Active-learning sweep planner (D-optimal Fisher information on predicted ellipse geometry) in autocal/active_learning.py:1, with CLI/reporting in autocal/active_calibrate.py:1.
  - Data collector upgrades in scripts/collect_sweep_data.mjs:1 (+ arg parsing in scripts/gcode_bridge.mjs:253):
      - --fixed-targets <spec> to collect specific fixed-anchor target(s) (overrides --superSweepRange/--superSweepPoints).
      - --return-to-origin to end the run back at encoder origin so multiple runs stay mergeable.

  How To Use (Ellipse Active Learning)

  - Suggest next sweep (prints anchors + uncertainty + top candidates + a ready-to-run collect_command):
      - python autocal/active_calibrate.py ellipse <dataset.json> --collector-args <your usual collect_sweep_data args...>
  - Example (your parameters; collect_command will include --fixed-targets ... --return-to-origin):
      - python autocal/active_calibrate.py ellipse autocal/data/big_even_pattern.json --collector-args --speedup 4 --trace --torque-low 0.03 --torque-min 0.03 --torque-max 0.3 --torque-step 0.05 --feed 400
  - One-step “suggest → collect → merge into dataset”:
      - python autocal/active_calibrate.py ellipse <dataset.json> --collect-once --collector-output /tmp/next.json --collector-args --speedup 4 ...

  Run it once on your current dataset and paste back the full stdout (the “report” lines + suggested collect_command + calibration summary), and I’ll tune the candidate range/scoring if needed.

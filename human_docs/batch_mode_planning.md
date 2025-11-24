
Based on your confirmation, the **Batch Process & Analyze** workflow is the only viable path for performance reasons. We will shift the app from a "Real-time Simulator" to an "Experiment Runner" workflow.

Here is the proposed UX, broken down into three distinct phases: **Setup**, **Execution**, and **Analysis**.

I have generated wireframes below to help you visualize these new screens.

### Phase 1: The Experiment Builder (Setup)
Instead of immediate 3D rendering, the user starts in a "Lab" view. This replaces the current right-hand menu when "Experiment Mode" is toggled.

*   **Drag-and-Drop Buckets:** You have distinct drop zones for `Machines (.usda)`, `Inputs (.serial/.can)`, and `Configs`.
*   **Combinatorial Logic:** A "Linkage" section allows you to say "Run *all* Inputs on *all* Machines" (Cartesian product) or manually pair them.
*   **The Manifest:** A table preview appears below showing the generated jobs (e.g., "Job 1: Hexagon Machine + Print_A.serial").

### Phase 2: The Headless Runner (Execution)
When the user clicks **"Run Batch,"** the 3D canvas fades out or blurs.
*   **Progress UI:** A modal or overlay shows the list of 27 jobs.
*   **Fast-Forward Math:** The browser runs the kinematic loops as fast as the CPU allows (no `requestAnimationFrame` throttling).
*   **Status Indicators:** Simple icons (⏳ Pending, ⚡ Running, ✅ Done, ❌ Failed) update in real-time.

### Phase 3: The "Score Tuner" Dashboard (Analysis)
This is the most critical new view. Once the batch is done, the user is presented with a data-heavy dashboard.

*   **The Leaderboard:** A table listing all 27 runs, sorted by the Aggregate Score.
*   **The Score Tuner (Your Constraint #3):** A panel of toggles (e.g., "Include Position Error," "Include Slack Penalty," "Include Jerk Limit"). Unchecking a box instantly recalculates the Aggregate Score and re-orders the leaderboard.
*   **Drill Down:** Clicking a row in the leaderboard loads that *specific* run into the 3D viewer for visual inspection, scrubbing directly to the moment where the worst error occurred.

---

### Visual Sketches
I have created a set of low-fidelity wireframes to demonstrate this flow using the code interpreter.

![ux_setup_wireframe.png ux_dashboard_wireframe.png](ux_setup_wireframe.png ux_dashboard_wireframe.png)
![ux_setup_wireframe.png ux_dashboard_wireframe.png](ux_setup_wireframe.png ux_dashboard_wireframe.png)

### The "Batch & Analyze" Workflow

I have designed a 3-phase workflow that respects your client-side performance limits while giving you the scientific rigor you need.

#### 1. The Setup Phase (Experiment Builder)
*Refer to `ux_setup_wireframe.png`*
Since we are moving away from "instant visual feedback" for these large batches, we need a dedicated staging area.
*   **The "Bucket" System:** Instead of a complex file tree, use three large drag-and-drop zones:
    *   **Machines:** Drop your `.usda` files here.
    *   **Inputs:** Drop your `.serial` or `.can` files here.
    *   **Configs:** Drop any parameter overrides here.
*   **The Generator:** A logic layer (hidden in the wireframe for simplicity) automatically creates the Cartesian product (Matrix). If you drop 3 machines and 3 inputs, the "Manifest" table below instantly populates with 9 rows.
*   **Manual Override:** The user can delete specific rows from the Manifest if they don't make sense (e.g., "Don't run the huge print on the tiny machine").

#### 2. The Execution Phase (Headless Runner)
*Not wireframed, as it is a transient state.*
When you hit **RUN BATCH**, the 3D canvas is hidden.
*   **Why?** Rendering WebGL is expensive. By hiding the canvas, you free up the JS event loop to crunch the kinematics math much faster than real-time.
*   **Feedback:** A simple progress bar `[====......] Run 4/27` is sufficient. This turns the tool into a "Calculator" for a few minutes.

#### 3. The Analysis Phase (Results Dashboard)
*Refer to `ux_dashboard_wireframe.png`*
This is where the value lies. Instead of trying to watch 27 robots at once, we look at the data first.
*   **The Leaderboard (Left):** This table ranks every run based on the Aggregate Score. You instantly see which combination (Machine + Algorithm) won.
*   **The Score Tuner (Top):** This directly addresses your **Constraint #3**. It is a row of toggle switches (Position Error, Slack Lines, Torque, etc.).
    *   *Interaction:* If you decide "I don't care about slack lines for this experiment," you uncheck that box. The Aggregate Scores recalculate in real-time, and the Leaderboard re-sorts itself.
*   **Deep Dive (Right):** Clicking a row in the leaderboard shows a summary graph (e.g., Error vs. Time).
*   **The "Watch Replay" Action:** This is the bridge back to your existing beautiful 3D view. When you click this, the dashboard slides away, and the simulator loads *just that one specific run*, scrubbed to the timestamp of the worst error shown in the graph.

### Clarifying Questions to Refine Implementation
1.  **Data Persistence:** When the batch run finishes, if the user refreshes the page, is the data lost? Do we need a "Download Experiment Report" button to save the results to a JSON/CSV?
2.  **Comparison View:** When looking at the "Replay," is it valuable to see *two* runs at once (e.g., The Winner vs. The Loser) ghosted over each other? Or is one at a time sufficient?

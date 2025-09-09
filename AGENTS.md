# Instructions for Agents

## Research Papers
- The directory `ai_docs/` contains `.md` versions of research papers that inform the physics algorithms used here.
- **Always read the relevant paper and confirm your approach before changing any physics behavior.**
- Each subfolder of `ai_docs/` (such as `CableJoints`, `PBDBodies`, `XPBD`, and `smallsteps`) contains the paper in markdown form.

## JavaScript and Python Ports
- The JavaScript version of the flipper and cable joints lives in `flipper.html (redirects to examples/js/flipper/index.html)` and the modules under `src/js/cable_joints/` and `examples/js/flipper/`.
- The Python port lives in `examples/python/flipper/index.html`, `examples/python/flipper/server.py`, and the modules under `src/python/cable_joints/` and
  `examples/python/flipper/`.
- When working on one implementation, cross reference the corresponding files from the other implementation to understand how features translate between JS and Python.

## Tests
- Don't change integration test expectations unless explicitly asked to do so.

I know you want to apply this patch or something similar:
```
    EXPECTED_SCORE = 11
    # Match the JavaScript integration test expectation for the pure Python
    # solver. The Warp variant settles with a slightly different score on this
    # platform.
    EXPECTED_SCORE = 11 if use_warp else 12
```
DON'T DO THAT. The EXPECTED_SCORE is supposed to be 11 like it is. This matches my local machines, and the Github CI environment's results.
Your (the Agent's) python environment doesn't exactly match the other environments', creating a discrepancy in expected score.


**AGENTS Usage Guide — About `apply_patch`**

- **Purpose:** Ensure the model uses the correct patch‑tool name when modifying the workspace, avoiding misuse that leads to errors or confusion.

- **Never use:** `applypatch` or `apply-patch` (without the underscore). These commands do not exist in the system and will result in “command not found” errors.

- **Correct usage (must include the underscore):** `apply_patch`.

- **Recommended examples (patch‑call examples for the model’s reference):**

  1. **Patch format to modify a file**

     ```
     *** Begin Patch
     *** Update File: path/to/file.txt
     @@
     -old line
     +new line
     *** End Patch
     ```

  2. **Calling it from the shell (handled by the execution environment)**

     ```json
     shell {"command":["apply_patch","*** Begin Patch\n*** Update File: path/to/file.txt\n@@\n-old line\n+new line\n*** End Patch\n"]}
     ```

- **Note:** Always write the underscore (`_`) explicitly. If you encounter `applypatch` or `apply-patch`, replace it with `apply_patch`.

- **Why this matters:** Using the correct name guarantees that the patch is applied properly and prevents unnecessary execution errors.


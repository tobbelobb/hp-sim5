# Second UI review

Reviewed the full [supplied diff](ui.diff), [desktop](v13.png), [effector close-up](v13-close.png), [phone](v10-mobile.png), and previous review. Keep the graphite palette, Settings popover, eyelets and z=0 axes. **Fix cable-batch growth and geometry occlusion before approval.** Locations below refer to the supplied patch; the live tree changed during review.

## Bugs/regressions — ranked

1. **P1 — Growing the cable batch truncates cables/wraps and leaks GPU buffers.** `cable_lines_3d.js:28–48` replaces attributes on the same geometry without disposing the old buffers or resetting Three.js’s cached instance limit. An isolated SwiftShader render, growing after the first draw from 4,096 capacity to 5,000 segments, retained `_maxInstanceCount=4096`: **24,576 triangles drawn instead of 30,000**. Disposal deleted **5 of 7 created buffers**. Larger uploads/multiple machines can trigger this; changing slack also changes segment count. **Fix:** replace and dispose the complete geometry on growth, preserving the batch data and reconnecting `object.geometry`.

2. **P2 — New frame nodes swallow eyelets and invent spherical joints.** `machine_details_3d.js:206–224` places opaque **6.4 mm radius** spheres at every frame corner. The pinhole torus’s entire outer radius is **3.6 mm**. In `slideprinter_single_pinholes.usda` and `slideprinter_hexagon.usda`, the authored frame edges terminate at those pinholes, so the rings are completely enclosed. A static three-pinhole fixture confirms this. The frame constant is also **4 mm radius (8 mm diameter)**, not 4 mm diameter. **Fix:** remove the added spherical nodes and terminate tubes before existing eyelets/bearings; use authored dimensions, with a simple edge representation where dimensions are unknown.

3. **P2 — The floor still hides supported geometry.** `render_system_3d.js:2344–2355` subtracts radius but ignores cylinder height/orientation, moving group members, cable sag/wrap extents and deposited material. A supported z-aligned cylinder at z=0, radius 5 mm, height 100 mm gets `floorZ=-25 mm`, burying its bottom **25 mm**. This remains an uploaded-scene failure despite fixing the earlier sphere example. **Fix:** make the backdrop unable to depth-occlude simulated geometry, or conservatively bound all rendered content. Do not add another support surface.

4. **P2 — Machine/authored colours are still discarded.** `render_system_3d.js:2528–2539,2686–2785` forces every cylinder to `ROTOR_COLOR` and every cable/wrap to pink. Tagged anchors/eyelets also lose their tint. Overlapping machines retain only a weak frame tint; uploads without a frame lose that cue entirely. **Fix:** preserve explicit machine/authored tints, using the graphite/steel/pink palette as defaults. Preserve the dark rotation half and consistent span/wrap colour. Untagged entities otherwise correctly fall back to the generic renderer; do not infer hardware from names.

5. **P3 — Solver select overhangs its Settings row.** `style.css:232–263` leaves insufficient room for the runtime label “Machine default (1)”. In the frozen patch at 800 px, the row has **264 px client width / 274 px scroll width**; the select extends about 10 px beyond its row. **Fix:** allow the label/select row to wrap or constrain the select with a shrinkable minimum width. Keep the full selected value discoverable.

## Screenshot/motion risks — ranked

1. **Eyelets can visibly snap, beyond ordinary aliasing.** `machine_details_3d.js:148–170,240–244` recomputes ring orientation from sign-aligned cable directions. Two meeting directions crossing 90° can switch the chosen bisector by about 90° even with a tiny endpoint change. Fixed eyelets also swivel independently of their assembly. **Fix:** follow authored/body orientation; where unavailable, retain a stable initial orientation. In the close-up, also raise eyelet roughness slightly to quiet the sharp white highlights without enlarging or hiding rings.

2. **The print still reads as a jagged yellow trace.** Both wide screenshots show a bright outline; the close-up reveals a stippled edge. The new bead shader uses a binary circular `discard`, so sub-pixel motion can toggle edge pixels. **Fix:** add derivative-based coverage/alpha-to-coverage to the existing point shader and slightly darken/desaturate the default yellow. Preserve quality colours and batching; avoid per-bead meshes.

3. **Thin frame/axes coverage remains fragile.** In the desktop view the frame approaches pixel width, and the axes show stair steps. Keep physical dimensions and z=0 placement; do not make the tubes thinner. Use filtered coverage for the same edges at distance. The cable batch’s claimed antialiasing is incomplete: `LineMaterial.alphaToCoverage` is **false**; enable its supported coverage path with the existing MSAA, then check moving wraps for merged/obscured markers. Static screenshots establish aliasing risk, not observed temporal flicker.

## Performance/lifecycle

- Batching cables is valuable, and the floor is skipped on detected software renderers. However, four new PBR instance pools add torus/sphere triangles, environment sampling, full matrix uploads, and another cable-direction/topology scan every frame. Cache topology/static data on scene changes, reuse scratch values, and avoid uploading unchanged pools. **No FPS non-regression claim is justified yet**, especially on software GL and multi-machine scenes.
- PMREM now owns/disposes its render target and rebuilds on context restore. A forced restore successfully rebuilt it, but emitted three old-context deletion warnings (`INVALID_OPERATION`); subsequent draws were clean. Dispose/clear the lost target during context loss before recreating it on restore.

Verification used isolated renderer/geometry probes and bootstrap-disabled Chromium layout checks, including an in-memory, hash-verified reconstruction of the supplied patch. No simulations. Toolbar overflow passed at 320, 360, 390, 600, 601, 640, 760, 761 and 800 px, including “Exit Fullscreen”; no repeat finding for the corrected gear hover or removed blur. No motion/FPS benchmark or physical-phone test.

Diff SHA-256: `336bfcdef6c05600a0e9fdc89ac5d9926c268e1819ee1c3b04f0afc783862bf5`.

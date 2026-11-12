# UI/rendering review

Keep the compact layout and studio direction: v05 gives the simulation substantially more room. The machine still reads as a coloured wire diagram over a photographic floor; improving the simulated material and small-part readability matters more than adding scenery.

Reviewed [ui.diff](ui.diff), all six supplied screenshots, and the new `stage_3d.js`. Findings below refer to the supplied diff, not subsequent edits. The live tree changed during review; its later `background-color` hover fix already addresses finding 4. Its later cable batching changes are outside this review. `stage_3d.js` is absent from `ui.diff`, so that file was inspected separately.

## Correctness — ranked by impact

1. **P2 — Toolbar overflows at narrow widths and immediately above the breakpoint.** [style.css:74](../../../hp-sim-3d/style.css) and the rules at 730–772 leave a nonwrapping row with intrinsic minimum widths. In Chromium, a 320 px viewport produces 329 px document width and puts the gear's right edge at 328.9 px. At 601 px, document width is 644 px and Fullscreen extends to 643.8 px. The 390 px screenshot misses both failures. Keep short labels until the full row actually fits; consolidate an action into an existing popover or permit a controlled fallback row. Check the longer “Exit Fullscreen” label too.

2. **P2 — The opaque floor can hide real geometry.** [render_system_3d.js:2487](../../../src/js/cable_joints_3d/render_system_3d.js) places it 20 mm below the lowest ungrouped **center**, ignoring rendered extents, spool orientation, accumulated wraps, cable sag, and printed material. A static renderer-supported sphere at z=0 with radius 50 mm yields floorZ=-20 mm, burying its bottom 30 mm. This is a demonstrated layout failure for larger/uploaded geometry, not an observed cut in the default HP4 screenshots. Use conservative rendered bounds, including wraps, and ensure the backdrop cannot occlude simulated information. Group membership alone also does not identify every moving body.

3. **P2 — PMREM render target is leaked on disposal.** [stage_3d.js:102](../../../src/js/cable_joints_3d/stage_3d.js) retains only `fromScene(...).texture`; `dispose()` disposes that texture rather than the owning `WebGLRenderTarget`. A three-cycle create/dispose probe on one renderer created 6 framebuffers but deleted 3, and created 3 renderbuffers but deleted none. Retain and dispose the returned target. This is a lifecycle leak, not evidence of allocations every frame.

4. **P2 — The phone gear disappears on hover/tap hover state.** In the supplied [style.css:122](../../../hp-sim-3d/style.css), `.sim-controls .sim-secondary:hover` uses the `background` shorthand, clearing the SVG `background-image`. With the button's text at `font-size:0`, it becomes blank, exactly as in [v05-mobile-settings.png](v05-mobile-settings.png). Confirmed computed `backgroundImage: "none"` while hovered. Use `background-color`; the live tree already contains this fix.

5. **P2 — Hardcoded pink discards authored/machine cable colours.** [render_system_3d.js:2888](../../../src/js/cable_joints_3d/render_system_3d.js) and the wrap paths replace `RenderableComponent.color` outright. `machineSceneController` still creates machine-specific cable palettes, so overlapping selected machines lose that identifying cue. Use pink as the default while retaining explicit tint support, or provide an equally clear existing selection cue. Keep span and wrap colouring consistent.

6. **P3 — Floor colour is decoded twice.** [stage_3d.js:21](../../../src/js/cable_joints_3d/stage_3d.js) creates a linear `THREE.Color`, writes those components into canvas bytes, then labels the texture sRGB. The nominal red/green value 198 becomes approximately 144 before sampling and is decoded again. Write sRGB canvas values, then tune exposure/noise against the corrected material.

No missing/duplicate control IDs or disconnected toggle/select handlers found. Reusing the menu controller preserves outside dismissal and Escape handling. At 390×844 the settings menu fits: x=58, width=320, y=52, bottom=384. Fullscreen handlers remain wired; native fullscreen itself was not exercised.

## Most valuable visual improvements — ranked

1. **Make deposited material look like material.** In [v05.png](v05.png) and [v05-mobile.png](v05-mobile.png), the pale-yellow print looks luminous and flat, easily confused with a reference/trace overlay. Existing unlit, fixed-pixel `PointsMaterial` explains this. Use a slightly darker, opaque filament colour and continuous world-scale thickness with restrained lighting. If geometry changes, use appendable batched strips or low-sided segments with bounded detail; preserve quality colouring. Add no support board.

2. **Improve cable coverage and colour balance.** Thin pink/amber strokes still look like CAD edges; the long diagonal amber spans dominate, while pale print material loses contrast against the floor. Use antialiased line coverage with a restrained minimum pixel width, test it at phone DPR, and darken/desaturate amber slightly while keeping slack unmistakable. Avoid thicker wraps that merge into solid discs or cover rotation markers. Pink can remain the taut default.

3. **Frame the print and moving assembly deliberately for each aspect ratio.** Bearings and wraps are tiny blue specks in the desktop view; on the phone, the rightmost assembly is pressed against the edge and important upper parts are cropped. Adjust initial target, distance, azimuth and polar angle using the selected machine and viewport. Reduce overlap between vertical cables, bearings and the print. Make the existing zoom/orbit controls reveal real-scale markers and buildup clearly; do not enlarge hardware or add brackets/flanges.

4. **Reduce the floor's visual weight.** The mottled floor occupies most of the frame and has more texture than the simulated objects. Substantially reduce noise contrast and use a quieter gradient. Put material highlights on existing bearing/spool surfaces, retaining their orientation marks and exposed wraps. The print is above this floor, so avoid fake contact shadows implying a support that is absent.

5. **Preserve compactness without tiny touch targets.** The 36 px phone toolbar and 30 px floating controls are considerably smaller than before. Aim for roughly 44 px touch hit areas, using invisible padding or fewer visible actions in the existing menus. Keep secondary controls collapsed on phones. The desktop settings label wrapping is a minor polish issue, below the rendering work above.

## Performance

- Fix the measured PMREM lifetime issue first. PMREM generation is otherwise a one-time startup cost; the stage adds no shadow maps or recurring environment capture.
- The textured, environment-lit floor covers almost the whole viewport, and numerous floating elements use `backdrop-filter: blur(8px)` over an animated canvas. These are mobile fill/compositing risks, not measured FPS regressions. Compare floor enabled/disabled and blur enabled/disabled on a real phone before adding effects; use opaque/translucent fills if blur is costly.
- `_syncStage` adds two rigid-membership queries plus another position scan every render, alongside the existing extent scan. Cache static membership/bounds when machines change, retaining updates for genuinely moving bounds. Small current scenes make this a lower priority. Preserve batching when improving lines or filament; avoid one mesh/material per sample.

## Verification and limits

`node node_modules/jest/bin/jest.js tests/js/hp-sim-3d --runInBand --no-cache`: **17 suites, 102 tests passed** in about two seconds. Jest emitted an existing duplicate-package-name warning from an archived source tree. These tests do not cover responsive layout or the new stage's WebGL lifetime.

Browser checks used the supplied diff reconstructed under `/tmp`, with simulator bootstrap disabled and the real menu controller enabled. Additional probes checked floor layout and Stage3D disposal without running a print. No long simulations, FPS benchmark, or physical-phone test. Screenshots have different poses/print progress, so visual comparisons are qualitative.

Input base: `3bfd9fefc2f54234528ec7b5be1f51a45a43a6da`; `ui.diff` SHA-256: `c5daeceb940c0f2fc72bbc012ac0be51c20d3644354067d5330fec0d1486306b`. Ensure the new stage module is included when packaging the patch.

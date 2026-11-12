You are a reviewer. DO NOT modify any files in the repository except writing your review to research/output/ui-review-1/review.md. Do not run long simulations; this is a UI/rendering review.

Context: hp-sim-3d (hp-sim-3d/index.html, hp-sim-3d/style.css, hp-sim-3d/app/appBootstrap.js, src/js/cable_joints_3d/render_system_3d.js, new src/js/cable_joints_3d/stage_3d.js) is a Three.js Hangprinter simulator UI. The goal is to make it look very good and clear, with REALISM as the leading star, without sacrificing performance. The owner's hard rules:
- Never add invented hardware/scenery (a table/build plate, spool flanges/motors, effector brackets were tried and rejected: they z-fought, didn't scale with the print, collided with lines, hid bearings and the spool line buildup).
- Don't clutter. Information visibility (bearing rotation markers, line wrap/buildup arcs, printed material) must never be hidden.
- Fewer buttons, especially on mobile.

The current uncommitted diff is in research/output/ui-review-1/ui.diff. Screenshots (1600x1000 desktop, 390x844 phone) in research/output/ui-review-1/: v00-baseline-print.png and v04-mobile.png are BEFORE; v05*.png are AFTER (v05-settings / v05-mobile-settings show the new Settings popover). Look at the images.

Changes so far: light studio backdrop + floor fading into it (custom PMREM studio environment, NeutralToneMapping), removal of origin axes and the old translucent print-surface board, taut cables recoloured to Dyneema pink (#e8606f) while slack stays amber, camera near plane scales with orbit distance (fixes z-fighting from near=1e-4), top bar compacted: toggles/selects moved into a Settings popover (reusing the machines-menu controller), short labels and a gear icon on phones, compact dark-glass floating controls.

Please review critically and concretely:
1. Correctness bugs or regressions in the diff (JS/CSS/HTML), e.g. broken IDs, event wiring, fullscreen, mobile popover placement, tests that would break (tests/js/hp-sim-3d/*).
2. Visual clarity/realism: what still looks bad or unclear in v05 screenshots? Prioritised list of the most valuable next improvements that respect the hard rules above (e.g. line rendering, colours of slack vs taut lines, printed material rendering, bearing/spool readability, depth cues, default camera).
3. Performance risks of the changes.
Keep it to the point; rank by impact.

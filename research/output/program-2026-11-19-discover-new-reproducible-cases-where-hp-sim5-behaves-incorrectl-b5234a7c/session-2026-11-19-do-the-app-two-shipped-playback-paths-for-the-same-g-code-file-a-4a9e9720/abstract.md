# Preset `.can` vs Upload G-code on HP4: agreement to 0.47 mm, but the upload path's `G6` applies its line move and then cancels it (0 of −42 mm)

The app's two shipped playback paths agree on HP4 within the documented flex-compensation envelope
(≤0.023 % line payout, 0.47 mm max toolpath on a 600 mm square), while the Upload path's `G6` support
emits its interpolated `Move` ramp and then an `Add to reference` carrying the same delta, which zeroes the
net line payout (`public/gcode/tighten.gcode`: 0.0 mm instead of −42 mm per line) — see
[report.md](report.md).

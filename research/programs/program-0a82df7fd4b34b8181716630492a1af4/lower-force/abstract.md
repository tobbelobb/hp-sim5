# HP3 lower sensor measurement force under motor drag and encoder noise

Status: in progress.

We compare the fixed4×midpoint sensor measurement force with the current5×midpoint rule in six fresh paired HP3 autocalibrations. The nominal and development worlds precede a heldout evaluation pair. All worlds use known30mm radii, zero buildup, no layering or guide friction, native Rust, and seeded Gaussian encoder-read noise. Motor Coulomb and smoothed-stiction factors vary by axis. 4 of six primary collections have completed or failed. No adoption decision yet. Complete the frozen matrix without retuning. A small hypothetical three-world simulation cannot establish hardware reliability or a lower failure rate. Applied anchor errors come from the rounded M669 command and a checked firmware readback. Every intermediate fit, failure and full command/noise journal is preserved separately from final applied geometry.

Topics: HP3, autocalibration, measurement force, motor friction, encoder noise, applied anchors.

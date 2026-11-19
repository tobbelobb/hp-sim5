# G-code program-level motion semantics in the browser Upload path

The `.gcode`/`.gc` Upload path silently mis-executes eight valid, documented G-code features
(`G91`, `G0`, `G01/G00`, `N`/`*` line framing, `G20`, `G2/G3`, `G4`, and the default feed rate):
relative-mode files travel 50 % short, `G0` travel is replaced by diagonals (−29 % travel, +33 % time),
`G01` files produce zero motion, arcs lose 61 % of travel and 50 % of extrusion, and dwell time is
dropped entirely — see [report.md](report.md).

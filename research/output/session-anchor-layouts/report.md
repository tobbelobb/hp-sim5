# Anchor-layout search within a 2 m radius

## Result

The archived configuration files are dated 1 September 2026. The original solver revision is unknown. The measurements refer to the bundled scenes and configurations under `best/`.

The best 2D result used six fixed line anchors: two for each of the three kinematic cable directions. The best 3D result used the HP4 topology: four kinematic cable groups, with paired fixed guides on each of the three low groups and one overhead group. Adding more kinematic anchors did not help. The stock eight-anchor CubeCorners machine scored 0.

The selected layouts are:

| Search | Firmware | Score | RMSE | p95 error | Corner radius | Ringing | Coverage / IoU | Missed steps |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 2D, six fixed anchors | RRF | **70** | 0.52 mm | 0.82 mm | 0.30 mm | 0.38 mm | unavailable in 2D | 0 |
| 3D, four cable groups | RRF, closed-loop | **63** | 0.43 mm | 0.61 mm | 0.40 mm | 0.56 mm | 87.6% / 69.8% | 0 |
| Same 3D geometry | Klipper, closed-loop | **65** | 0.42 mm | 0.62 mm | 0.41 mm | 0.54 mm | 89.4% / 70.6% | 0 |

RRF was used for the search. Klipper reproduced both the winner and the abrupt failure just beyond it, so the discontinuity is not an RRF-only effect.

## Exact placement

All coordinates are metres relative to the machine origin. “Attachment inradius” is the perpendicular distance from the origin to a side of the moving attachment triangle.

### 2D

The moving attachment inradius is 0.075 m. The three virtual kinematic anchors are 1.920776791 m from their corresponding moving attachment points. The six physical fixed anchors are:

```text
A-left   (-0.129904, -1.995777, 0)
A-right  ( 0.129904, -1.995777, 0)
B-left   ( 1.793345,  0.885388, 0)
B-right  ( 1.663441,  1.110388, 0)
C-left   (-1.663441,  1.110388, 0)
C-right  (-1.793345,  0.885388, 0)
```

Every fixed anchor is exactly 2.0 m from the origin. The matching RRF geometry is:

```gcode
M669 A0.0:-1920.776791:0.0 B1663.441496:960.388396:0.0 C-1663.441496:960.388396:0.0
M666 Y520.688049:520.688049:520.688049
```

### 3D

The moving attachment inradius is 0.175 m. The virtual low-anchor radius is 1.752336554 m, and the virtual overhead distance is 1.869135117 m. The paired low guide coordinates are:

```text
A-left   (-0.303109, -1.962337, -0.1800 / -0.2395)
A-right  ( 0.303109, -1.962337, -0.1800 / -0.2395)
B-left   ( 1.850988,  0.718668, -0.1800 / -0.2395)
B-right  ( 1.547879,  1.243668, -0.1800 / -0.2395)
C-left   (-1.547879,  1.243668, -0.1800 / -0.2395)
C-right  (-1.850988,  0.718668, -0.1800 / -0.2395)
```

The overhead D guides use the same 0.175 m XY attachment footprint and a fixed guide plane chosen so its outermost point is on the 2.0 m sphere. The matching RRF geometry is:

```gcode
M669 A0.0:-1752.336554:-280.0 B1517.567971:876.168277:-280.0 C-1517.567971:876.168277:-280.0 D0.0:0.0:1869.135117
M666 Y501.871497:501.871497:501.871497:344.413865
```

The `.usda` file is the source of truth for the individual pulley and guide locations. Its most distant fixed cable-guide points are exactly 2.0 m from the origin.

## What was tested

Every run printed `public/gcode/draw_squares_bigger.gcode`. This is a nested-square path extending to roughly ±600 mm. Scores came from the simulator quality monitor, which combines path RMSE, p95 error, rounded corners, ringing, path coverage, intersection-over-union, and missed steps.

The stock 2D comparison was:

| Machine | Score | RMSE | p95 | Corner avg. | Ringing | Missed |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Slideprinter Multi Unit, six fixed anchors | **68** | 0.57 | 1.00 | 0.31 | 0.36 | 0 |
| Hexagon | 64 | 0.55 | 0.90 | 0.50 | 0.87 | 0 |
| Single pinholes | 34 | 0.82 | 1.93 | 2.37 | 2.46 | 0 |
| Original | 0 | 17.50 | 23.90 | 6.42 | 11.50 | 5 |

The six-anchor candidate was then swept across moving attachment inradii from 0.05 to 0.30 m. A 0.075 m inradius scored 70; the neighbouring 0.05, 0.10, 0.125, and 0.15 m cases scored 68, 68, 69, and 68.

The stock 3D comparison, using open-loop RRF, was:

| Machine | Kinematic groups | Score | RMSE | p95 | Corner avg. | Ringing | Coverage / IoU | Missed |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| HP4 | 4 | **45** | 0.49 | 0.62 | 1.06 | 1.28 | 57.3% / 35.3% | 2 |
| Four High Anchors | 4 | 30 | 1.08 | 2.14 | 0.87 | 1.26 | 78.3% / 54.0% | 0 |
| HP3 | 3 | 27 | 0.60 | 1.03 | 2.52 | 4.21 | 65.2% / 43.6% | 2 |
| CubeCorners | 8 | 0 | 5.07 | 9.29 | 7.01 | 5.04 | 0.5% / 0.2% | 4 |

The radius-constrained HP4 geometry was swept from 0.075 to 0.30 m attachment inradius. Closed-loop control removed the missed-step penalty and made the useful geometry comparison possible. The score rose from 59 at 0.075 m to 63 at 0.175 m. At 0.176 m it fell to 59 because the maximum detected corner radius jumped from 1.28 mm to about 64 mm. The same event appeared in Klipper. A 1.99 m boundary with a 0.170 m inradius also entered this poorer regime and scored 55.

This is a sharp simulated cable/geometry transition, not a smooth optimum. Do not blindly shrink the 2.0 m layout for installation clearance. Run a finer sweep around the actual toleranced dimensions.

## Reproduction

Use the files under `best/` to replay the archived inputs. The following commands rebuild the same geometry using the current templates. Motor torque, firmware settings, and USD transforms have changed since the original runs. Write rebuilt inputs to a separate directory.

```bash
.venv/bin/python research/output/session-anchor-layouts/generate_slideprinter_multi_unit.py \
  --virtual-radius 1.920776791126703 \
  --attachment-inradius 0.075 \
  --scene-output output/experiments/anchor-layout-rebuild/slideprinter_six_anchor_radius2.usda \
  --config-output output/experiments/anchor-layout-rebuild/config_slideprinter_six_anchor_radius2.g
```

Rebuild the 3D scene and both firmware configurations:

```bash
.venv/bin/python research/output/session-anchor-layouts/generate_hp4_symmetric.py \
  --attachment-inradius 0.175 \
  --boundary-radius 2.0 \
  --scene-output output/experiments/anchor-layout-rebuild/hp4_four_anchor_radius2.usda \
  --config-output output/experiments/anchor-layout-rebuild/config_hp4_four_anchor_radius2.g

.venv/bin/python research/output/session-anchor-layouts/rrf_hp4_to_klipper.py \
  output/experiments/anchor-layout-rebuild/config_hp4_four_anchor_radius2.g \
  output/experiments/anchor-layout-rebuild/printer-hp4-four-anchor-radius2-linux-mcu-with-buildup.cfg
```

Produce an RRF motion dump for browser upload:

```bash
./RRF/build/rrf_simulator \
  --vsd RRF/run/vsd \
  --gcode gcodes/draw_squares_bigger.gcode \
  --can-log /tmp/draw_squares_bigger_radius2.csv \
  --config "$PWD/research/output/session-anchor-layouts/best/config_hp4_four_anchor_radius2.g"
```

For Klipper, use `klipper/klippy/klippy.py` with the selected config and parse the resulting serial dump with `klipper/scripts/parsedump.py`, as described in the repository README.

## Limits of the result

- The quality score is a simulator heuristic, not a physical metrology result.
- The 2D monitor reported coverage as unavailable, so 2D and 3D absolute scores should not be compared. Rankings within the same simulator remain useful.
- Closed-loop is a simulator motor option, not something encoded by `M669`. A real machine needs the corresponding encoders and motor-control stack.
- The search was broad enough to identify the topology and the narrow 3D transition, but it is not a global optimizer over arbitrary asymmetric anchor placements.
- The virtual anchors in `M669` are line-action geometry. The individual fixed guides in the `.usda` scenes are the points constrained to the 2.0 m radius.

; Simple Cartesian configuration for testing
G21              ; Work in millimetres
G90              ; Absolute coordinates
M83              ; Relative extruder moves

; Axis to driver mapping (local drivers)
M584 X1.0 Y2.0 Z3.0 P3
M584 E4.0

; Kinematics
M669 K1  ; Cartesian

; Motor directions
M569 P0 S1        ; X motor forward
M569 P1 S1        ; Y motor forward
M569 P2 S0        ; Z motor backward
M569 P3 S1        ; E motor forward

M208 Z2000.00  ; maximum Z
M208 S1 Z-10.0 ; minimum Z

M564 S0

; Steps per mm (different from Hangprinter)
M92 X160 Y160 Z400 E400

M201 X3000 Y3000 Z500 E5000    ; Accelerations
M203 X12000 Y12000 Z1200 E6000 ; Max speeds
M204 P2000 T4000               ; Accelerations while printing and for travel moves
M566 X600 Y600 Z60 E1800       ; Jerk (different from Hangprinter)

M563 P0 D0 H1                                      ; Tool number 0, with extruder drive 0 uses heater 1 and no fan
G10 P0 S0 R0                                       ; Set initial tool 0 active at standby temperature 0

M574 X0 Y0 Z0                                ; set endstop configuration (no endstops)

T0

; Simple Cartesian configuration for testing
G21              ; Work in millimetres
G90              ; Absolute coordinates
M83              ; Relative extruder moves

; Axis to driver mapping (local drivers)
M584 X0 Y1 Z2 P3  ; 3 axes on local drivers 0, 1, 2
M584 E3           ; Extruder on local driver 3

; Motor directions
M569 P0 S1        ; X motor forward
M569 P1 S1        ; Y motor forward
M569 P2 S0        ; Z motor backward
M569 P3 S1        ; E motor forward

; Steps per mm (different from Hangprinter)
M92 X160 Y160 Z400 E400

; Accelerations (lower than Hangprinter)
M201 X3000 Y3000 Z500 E5000

; Max speeds (different from Hangprinter)
M203 X12000 Y12000 Z1200 E6000

; Jerk (different from Hangprinter)
M566 X600 Y600 Z60 E1800

; Kinematics
M669 K1  ; Cartesian

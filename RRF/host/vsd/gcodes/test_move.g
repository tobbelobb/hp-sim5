; Simple test G-code for Step 9.2.1
; Two linear moves to test CAN packet generation

G90          ; Absolute positioning
G92 X0 Y0 Z0 A0  ; Set origin

; Move 1: Simple XY move
G1 X10 Y10 F6000

; Move 2: 3-axis move
G1 X20 Y20 Z5 F3000

; Move 3: With extrusion
G1 X30 Y30 E1.5 F3000

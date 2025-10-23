; Simple Hangprinter test file
; The config.g should have configured 4 axes (XYZU) mapped to CAN drivers 40-43
G90              ; Absolute positioning
G92 X0 Y0 Z0 U0  ; Set current position as zero
G1 X10 Y10 Z5 U15 F6000  ; Move all four axes
G1 X20 Y20 Z10 U30 F3000  ; Another move with different speed

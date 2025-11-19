; Communication and general
G21              ; Work in millimetres
G90              ; Send absolute coordinates...
M83              ; ...but relative extruder moves

; Kinematics
M584 X40.0 Y41.0 Z42.0 E43.0 P3 ; map ABCE-axes to CAN addresses, and set four visible axes. Please excuse that ABC motors are called XYZ here.
M669 K6                         ; "This is a Hangprinter"
M669 N3                         ; This Hangprinter has three anchors (is a Slideprinter)
M666 A2                         ; anchorMode=HangprinterAnchorMode::AllOnTop, // 0=None, 1=last-top, 2=all-top, 3-half-top, etc
M669 S130 T0.5                  ; Segments per second and min segment length

; Anchor Location
M669 A0.0:-1900.0:0.0 B1645.44826719:950.0:0.0 C-1645.44826719:950.0:0.0
M666 Q0.0 R30.0:30.0:30.0
; Explanation:
; ; M669 defines the positions of the anchors, expressed as X:Y:Z distances between a line's pivot points, when the machine is homed.
; ; M666 sets Q=spool buildup, R=spool radii (incl buildup, when homed)

M208 Z2000.00  ; set maximum Z somewhere below D anchor. See M669 ... D<number>
M208 S1 Z-10.0 ; set minimum Z

; The following values must also be in the auto calibration script for Hangprinter (if you plan to use it)
M666 U1:1:1     ; Mechanical advantages
M666 O1:1:1     ; Number of lines per spool
M666 L20:20:20  ; Motor gear teeth
M666 H20:20:20  ; Spool gear teeth

; Flex compensation
;M666 W0.081               ; Mover weighs 0.081 kg.
M666 W0.0                 ; Disable flex compensation until we have fixed it.
M666 S20000.0             ; Spring constant (rough approximation) for Garda 1.1 mm line (unit N/m).
                          ; The real value is somewhere between 20k and 100k.
                          ; Lower value gives more flex compensation.
M666 I3.0:3.0:3.0         ; Min planned force in four directions (unit N).
                          ; This is a safety limit. Should affect only exceptional/wrong moves,
                          ; for example moves outside of the reachable volume.
M666 X170.0:170.0:170.0   ; Max planned force in four directions (unit N)
                          ; This is a safety limit. Will affect moves close to
                          ; the limits of the reachable volume.
M666 T10.0                ; Desired target force (unit N).
                          ; The flex compensation algorithm aims for at least
                          ; this amount of fource in the ABC line directions at all times.
                          ; It can be thought of as a minimum pre-tension value.
                          ; It's recommended to set it around 10 times higher
                          ; than your W (mover weight in kg) value.

; Guy wire lengths. Needed for flex compenation.
; Guy wires go between spool and final line roller.
; If your spools are all mounted on the D-anchor, on the ceiling plate, then you're all good,
; and you don't need to configure M666 Y values explicitly.
; If your spools are not all on the D-anchor then you must measure guy wire
; lengths and set them here.
; If your spools are all mounted on their respective anchors, so that you have no guy wires,
; then you should configure zeroed guy wire lengths M666 Y0.0:0.0:0.0:0.0.
;M666 Y528.3:528.3:528.3

; Uncomment M564 S0 if you don't want G0/G1 moves to be be limited to a software defined volume
M564 S0

; Drives
M666 J200:200:200 ; Full steps per ABC motor revolution

M569 P0 S1 ; Drive 0 goes forwards
M569 P1 S1 ; Drive 1 goes forwards
M569 P2 S1 ; Drive 2 goes forwards
M569 P3 S1 ; Drive 3 goes forwards
M569 P4 S1 ; Drive 4 goes forwards
M569 P5 S1 ; Drive 5 goes forwards

M569 P40.0 S1 ; Drive 40.0 (A) goes forwards
M569 P41.0 S1 ; Drive 41.0 (B) goes forwards
M569 P42.0 S1 ; Drive 42.0 (C) goes forwards
M569 P43.0 S1 ; Drive 43.0 (E) goes forwards

; Speeds and accelerations
M201 X3000 Y3000 Z3000 E3000       ; Max accelerations (mm/s^2)
M203 X18000 Y18000 Z18000 E18000   ; Max speeds (mm/min)
M204 P3000 T3000                   ; Accelerations while printing and for travel moves
M566 X300 Y300 Z300 E1200          ; Maximum instant speed changes mm/minute

; Currents
M906 E1400 I60                     ; Set motor currents (mA) and increase idle current to 60%

; Endstops
M574 X0 Y0 Z0                      ; set endstop configuration (no endstops)

; Thermistors and heaters
M308 S1 P"temp0" Y"thermistor" T100000 B3950 ; Configure sensor 1 as thermistor on temp1
M950 H1 C"out1" T1                           ; create nozzle heater output on out1 and map it to sensor 1
M307 H1 B0 S1.00                             ; disable bang-bang mode for nozzle heater and set PWM limit
M307 H1 A1271.9 C432.5 D8.2 V24              ; Set heater parameters (for Super Volcano 80W. You probably want to tune this yourself with M303.)
M143 H1 S280                                 ; set temp limit for nozzle heater to 280C
M570 H1 S180                                 ; Hot end may be a little slow to heat up so allow it 180 seconds

; Fans
M950 F1 C"out7"
M106 P1 X255 T45 H1                          ; Enable Fan 1 thermostatic mode for sensor or heater 1 at 45 degrees
M950 F0 C"out8"                              ; Defines a part cooling fan

; Tool definitions
M563 P0 D0 H1                                ; Tool number 0, with extruder drive 0 uses heater 1 and no fan
G10 P0 S0 R0                                 ; Set initial tool 0 active at standby temperature 0

; Miscellaneous
M92 E95.922                                  ; Set extruder steps per mm
T0                                           ; Select tool 0

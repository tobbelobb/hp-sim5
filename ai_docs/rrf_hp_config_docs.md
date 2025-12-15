M666 parameters U, L, H, and J are related to gearing or "steps per millimeter" of the Hangprinter.
M666 U parameter sets the mechanical advantage (number of times each line travels back and forth between anchor and effector) in each direction. Doubling this from say U1:1:1:2 to U2:2:2:4 will make motors move ca twice as many steps per millimeter.
M666 L parameter sets the number of motor gear teeth per motor (axis). Each motor is assumed to have a gear with a number of teeth on it. Used for calculating gear ratios and hence affects the effective "steps per millimeter".
M666 H parameter sets the number of spool gear teeth per spool (axis). Each spool is assumed to have a gear with a number of teeth on it. Used for calculating gear ratios and hence affects the effective "steps per millimeter".
M666 J parameter sets the number of full steps per motor revolution for each motor. Match this with your ODrive configuration if using ODrives, or your stepper motors if using steppers.
M666 C parameter sets the torque constants for each motor. These are required for reading motor forces from ODrives. They are the same values as is configured in the ODrives themselves (8.27/330 for motors in the standard  HP4 BOM). Formula/equation source: https://discourse.odriverobotics.com/t/where-does-the-formula-for-calculating-torque-come-from/1169.
M666 parameters R is related to both gearing of the Hangprinter and the buildup compensation feature.
M666 R parameter sets the spool radii (including line buildup, when printer is homed).
M666 parameters O and Q are related to the buildup compensation feature.
M666 O parameter sets the number of lines per spool. Used by the spool buildup compensation. Two lines sharing the same spool area, like the HP1 and HP2 did doubles the speed of buildup. This is usually set to 1:1:1:1 for HP3 and HP4.
M666 Q parameter sets the spool buildup factor. "How quickly the line builds up", derived from line thickness and shape. Usually derived through auto calibration.
M666 Q can be approximated by hand. Example: line diameter: 0.5 mm, spool height: 8.0 mm, (line_cross_section_area)/(height*pi): ((0.5/2)*(0.5/2)*pi)/(8.0*pi) = 0.0078 mm. Measure and fill in your own numbers. In practice you might want to compensate a bit more or a bit less.
M666 parameters W, S, I, X, T, and Y are related to the flex compensation feature.
M666 W parameter sets the effectors weight in kilograms.
M666 S parameter sets the Spring constant (rough approximation) of the line in units of N/m. HP4 uses Garda 1.1 mm line, which has a spring constant of about 20000 N/m.
M666 I parameter sets the minimum planned force in each direction (unit N).
M666 X parameter sets the maximum planned force in each direction (unit N). This is a safety limit, and will affect moves close to the edges of the reachable volume the most.
M666 Y parameter sets the guy wire lengths. Needed for flex compensation. Guy wires go between spool and final line roller.
M666 A and M669 N together define the Hangprinters anchor configuration.
M666 A parameter sets the anchor mode. 0=None, 1=last-top, 2=all-top. Default: last-top.
M669 K parameter sets the kinematics. Hangprinter users should have a M669 K6 line early in their Reprapfirmware config.g.
M669 N parameter sets the number of anchors Default: 4.
M669 P parameter sets the printable radius. It is unused by Hangprinters.
M669 S and T parameters sets segments per second and min segment length. That's basically how many times per second or per length the firmware will change the rotational speed of the motors.
M669 ABCDIJKLO parameters sets anchor locations, expressed as X:Y:Z distances between a line's pivot points, when the machine is homed. You only need as many anchor locations as you have anchors.

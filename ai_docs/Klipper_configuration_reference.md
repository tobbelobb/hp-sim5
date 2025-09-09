Configuration reference
This document is a reference for options available in the Klipper config file.

The descriptions in this document are formatted so that it is possible to cut-and-paste them into a printer config file. See the installation document for information on setting up Klipper and choosing an initial config file.

Micro-controller configuration
Format of micro-controller pin names
Many config options require the name of a micro-controller pin. Klipper uses the hardware names for these pins - for example PA4.

Pin names may be preceded by ! to indicate that a reverse polarity should be used (eg, trigger on low instead of high).

Input pins may be preceded by ^ to indicate that a hardware pull-up resistor should be enabled for the pin. If the micro-controller supports pull-down resistors then an input pin may alternatively be preceded by ~.

Note, some config sections may "create" additional pins. Where this occurs, the config section defining the pins must be listed in the config file before any sections using those pins.

[mcu]
Configuration of the primary micro-controller.


[mcu]
serial:
#   The serial port to connect to the MCU. If unsure (or if it
#   changes) see the "Where's my serial port?" section of the FAQ.
#   This parameter must be provided when using a serial port.
#baud: 250000
#   The baud rate to use. The default is 250000.
#canbus_uuid:
#   If using a device connected to a CAN bus then this sets the unique
#   chip identifier to connect to. This value must be provided when using
#   CAN bus for communication.
#canbus_interface:
#   If using a device connected to a CAN bus then this sets the CAN
#   network interface to use. The default is 'can0'.
#restart_method:
#   This controls the mechanism the host will use to reset the
#   micro-controller. The choices are 'arduino', 'cheetah', 'rpi_usb',
#   and 'command'. The 'arduino' method (toggle DTR) is common on
#   Arduino boards and clones. The 'cheetah' method is a special
#   method needed for some Fysetc Cheetah boards. The 'rpi_usb' method
#   is useful on Raspberry Pi boards with micro-controllers powered
#   over USB - it briefly disables power to all USB ports to
#   accomplish a micro-controller reset. The 'command' method involves
#   sending a Klipper command to the micro-controller so that it can
#   reset itself. The default is 'arduino' if the micro-controller
#   communicates over a serial port, 'command' otherwise.
[mcu my_extra_mcu]
Additional micro-controllers (one may define any number of sections with an "mcu" prefix). Additional micro-controllers introduce additional pins that may be configured as heaters, steppers, fans, etc.. For example, if an "[mcu extra_mcu]" section is introduced, then pins such as "extra_mcu:ar9" may then be used elsewhere in the config (where "ar9" is a hardware pin name or alias name on the given mcu).


[mcu my_extra_mcu]
# See the "mcu" section for configuration parameters.
Common kinematic settings
[printer]
The printer section controls high level printer settings.


[printer]
kinematics:
#   The type of printer in use. This option may be one of: cartesian,
#   corexy, corexz, hybrid_corexy, hybrid_corexz, generic_cartesian,
#   rotary_delta, delta, deltesian, polar, winch, or none.
#   This parameter must be specified.
max_velocity:
#   Maximum velocity (in mm/s) of the toolhead (relative to the
#   print). This value may be changed at runtime using the
#   SET_VELOCITY_LIMIT command. This parameter must be specified.
max_accel:
#   Maximum acceleration (in mm/s^2) of the toolhead (relative to the
#   print). Although this parameter is described as a "maximum"
#   acceleration, in practice most moves that accelerate or decelerate
#   will do so at the rate specified here. The value specified here
#   may be changed at runtime using the SET_VELOCITY_LIMIT command.
#   This parameter must be specified.
#minimum_cruise_ratio: 0.5
#   Most moves will accelerate to a cruising speed, travel at that
#   cruising speed, and then decelerate. However, some moves that
#   travel a short distance could nominally accelerate and then
#   immediately decelerate. This option reduces the top speed of these
#   moves to ensure there is always a minimum distance traveled at a
#   cruising speed. That is, it enforces a minimum distance traveled
#   at cruising speed relative to the total distance traveled. It is
#   intended to reduce the top speed of short zigzag moves (and thus
#   reduce printer vibration from these moves). For example, a
#   minimum_cruise_ratio of 0.5 would ensure that a standalone 1.5mm
#   move would have a minimum cruising distance of 0.75mm. Specify a
#   ratio of 0.0 to disable this feature (there would be no minimum
#   cruising distance enforced between acceleration and deceleration).
#   The value specified here may be changed at runtime using the
#   SET_VELOCITY_LIMIT command. The default is 0.5.
#square_corner_velocity: 5.0
#   The maximum velocity (in mm/s) that the toolhead may travel a 90
#   degree corner at. A non-zero value can reduce changes in extruder
#   flow rates by enabling instantaneous velocity changes of the
#   toolhead during cornering. This value configures the internal
#   centripetal velocity cornering algorithm; corners with angles
#   larger than 90 degrees will have a higher cornering velocity while
#   corners with angles less than 90 degrees will have a lower
#   cornering velocity. If this is set to zero then the toolhead will
#   decelerate to zero at each corner. The value specified here may be
#   changed at runtime using the SET_VELOCITY_LIMIT command. The
#   default is 5mm/s.
[stepper]
Stepper motor definitions. Different printer types (as specified by the "kinematics" option in the [printer] config section) require different names for the stepper (eg, stepper_x vs stepper_a). Below are common stepper definitions.

See the rotation distance document for information on calculating the rotation_distance parameter. See the Multi-MCU homing document for information on homing using multiple micro-controllers.


[stepper_x]
step_pin:
#   Step GPIO pin (triggered high). This parameter must be provided.
dir_pin:
#   Direction GPIO pin (high indicates positive direction). This
#   parameter must be provided.
enable_pin:
#   Enable pin (default is enable high; use ! to indicate enable
#   low). If this parameter is not provided then the stepper motor
#   driver must always be enabled.
rotation_distance:
#   Distance (in mm) that the axis travels with one full rotation of
#   the stepper motor (or final gear if gear_ratio is specified).
#   This parameter must be provided.
microsteps:
#   The number of microsteps the stepper motor driver uses. This
#   parameter must be provided.
#full_steps_per_rotation: 200
#   The number of full steps for one rotation of the stepper motor.
#   Set this to 200 for a 1.8 degree stepper motor or set to 400 for a
#   0.9 degree motor. The default is 200.
#gear_ratio:
#   The gear ratio if the stepper motor is connected to the axis via a
#   gearbox. For example, one may specify "5:1" if a 5 to 1 gearbox is
#   in use. If the axis has multiple gearboxes one may specify a comma
#   separated list of gear ratios (for example, "57:11, 2:1"). If a
#   gear_ratio is specified then rotation_distance specifies the
#   distance the axis travels for one full rotation of the final gear.
#   The default is to not use a gear ratio.
#step_pulse_duration:
#   The minimum time between the step pulse signal edge and the
#   following "unstep" signal edge. This is also used to set the
#   minimum time between a step pulse and a direction change signal.
#   The default is 0.000000100 (100ns) for TMC steppers that are
#   configured in UART or SPI mode, and the default is 0.000002 (which
#   is 2us) for all other steppers.
endstop_pin:
#   Endstop switch detection pin. If this endstop pin is on a
#   different mcu than the stepper motor then it enables "multi-mcu
#   homing". This parameter must be provided for the X, Y, and Z
#   steppers on cartesian style printers.
#position_min: 0
#   Minimum valid distance (in mm) the user may command the stepper to
#   move to.  The default is 0mm.
position_endstop:
#   Location of the endstop (in mm). This parameter must be provided
#   for the X, Y, and Z steppers on cartesian style printers.
position_max:
#   Maximum valid distance (in mm) the user may command the stepper to
#   move to. This parameter must be provided for the X, Y, and Z
#   steppers on cartesian style printers.
#homing_speed: 5.0
#   Maximum velocity (in mm/s) of the stepper when homing. The default
#   is 5mm/s.
#homing_retract_dist: 5.0
#   Distance to backoff (in mm) before homing a second time during
#   homing. Set this to zero to disable the second home. The default
#   is 5mm.
#homing_retract_speed:
#   Speed to use on the retract move after homing in case this should
#   be different from the homing speed, which is the default for this
#   parameter
#second_homing_speed:
#   Velocity (in mm/s) of the stepper when performing the second home.
#   The default is homing_speed/2.
#homing_positive_dir:
#   If true, homing will cause the stepper to move in a positive
#   direction (away from zero); if false, home towards zero. It is
#   better to use the default than to specify this parameter. The
#   default is true if position_endstop is near position_max and false
#   if near position_min.


Cable winch Kinematics
See the example-winch.cfg for an example cable winch kinematics config file:

```
file: example-winch.cfg
# This file is an example config file for cable winch style printers.
# One may copy and edit this file to configure a new cable winch
# printer.

# CABLE WINCH SUPPORT IS EXPERIMENTAL - PROCEED WITH CAUTION!

# Homing is not implemented on cable winch kinematics. In order to
# home the printer, manually send movement commands until the toolhead
# is at 0, 0, 0 and then issue a G28 command.

[stepper_a]
step_pin: PF0
dir_pin: PF1
enable_pin: !PD7
microsteps: 16
rotation_distance: 40
anchor_x: 0
anchor_y: -2000
anchor_z: -100

[stepper_b]
step_pin: PF6
dir_pin: PF7
enable_pin: !PF2
microsteps: 16
rotation_distance: 40
anchor_x: 2000
anchor_y: 1000
anchor_z: -100

[stepper_c]
step_pin: PL3
dir_pin: PL1
enable_pin: !PK0
microsteps: 16
rotation_distance: 40
anchor_x: -2000
anchor_y: 1000
anchor_z: -100

[stepper_d]
step_pin: PC1
dir_pin: PC3
enable_pin: !PC7
microsteps: 16
rotation_distance: 40
anchor_x: 0
anchor_y: 0
anchor_z: 3000

[extruder]
step_pin: PA4
dir_pin: PA6
enable_pin: !PA2
microsteps: 16
rotation_distance: 33.500
nozzle_diameter: 0.400
filament_diameter: 1.750
heater_pin: PB4
sensor_type: ATC Semitec 104GT-2
sensor_pin: PK5
control: pid
pid_Kp: 22.2
pid_Ki: 1.08
pid_Kd: 114
min_temp: 0
max_temp: 250

[heater_bed]
heater_pin: PH5
sensor_type: EPCOS 100K B57560G104F
sensor_pin: PK6
control: watermark
min_temp: 0
max_temp: 130

[mcu]
serial: /dev/ttyACM0

[printer]
kinematics: winch
max_velocity: 300
max_accel: 3000
```

Only parameters specific to cable winch printers are described here - see common kinematic settings for available parameters.

CABLE WINCH SUPPORT IS EXPERIMENTAL. Homing is not implemented on cable winch kinematics. In order to home the printer, manually send movement commands until the toolhead is at 0, 0, 0 and then issue a G28 command.

[printer]
kinematics: winch

# The stepper_a section describes the stepper connected to the first
# cable winch. A minimum of 3 and a maximum of 26 cable winches may be
# defined (stepper_a to stepper_z) though it is common to define 4.
[stepper_a]
rotation_distance:
#   The rotation_distance is the nominal distance (in mm) the toolhead
#   moves towards the cable winch for each full rotation of the
#   stepper motor. This parameter must be provided.
anchor_x:
anchor_y:
anchor_z:
#   The X, Y, and Z position of the cable winch in cartesian space.
#   These parameters must be provided.





# COPacabana dynamic PlaneA replay

The first 24.26 s of the 13.9 kg standard-control run give a useful split between drive tracking and cable mechanics. Feeding the eight recorded motor targets into the simulator gives 25.4 mm RMS reflector error. Substituting the encoder readings reduces it to 15.8 mm. The two inputs give nearly identical force errors, around 176 N RMS. Neither reaches the measured positional repeatability.

## Replay in hp-sim-3d

Select **COPacabana (13.9 kg)** as the only machine. **Replay targets** drives the eight drums from the logged cable-position targets. **Replay encoders** drives them from measured payout using the same actuator model. The white trace follows the larger sphere, which represents the FARO reflector. The blue path shows where the tracker measured that reflector. The smaller sphere below it marks the payload center of mass. All eight cables attach at the corners of the wireframe. Turn on **Closed loop motors** and turn off line layering to reproduce the numerical comparison. The nominal PlaneA G-code, Logo and Squares go through the catalogued firmware planners. The experiment buttons use a separate translation of the 100 Hz robot log, preserving reversals and all five stationary intervals.

## Physical assumptions

The scene uses the eight crossed attachments, 13.9 kg mass, center of gravity and inertia tensor from the XML. Its origin is the platform reference at P0, 0.6 m above the XML origin. The simulated marker is a massless rigid-body member at the registered P0 reflector offset. The 37/33 mm swivel pulleys are approximated by fixed exit tangents computed at home. Each simulated drum has a 49 mm radius. The firmware and scene use equivalent direct drive at the output of the 5:1 gearbox. Torque capacity is the XML motor limit times five. Rotor inertia, damping and the 0.2 m offset from drum to guide are simulation assumptions. The simulation uses 83.8 kN axial stiffness transferred from the earlier rope identification. Initial span and storage tensions come from the eight P0 force readings. Later force readings are reserved for comparison.

The motion source is Export 1 in robot/20_Std_Load13900/PlaneA. The converter differences rounded cumulative displacements, with 3,200 microsteps per equivalent drum revolution. It encodes those intervals into the simulator's compact CAN format. The source machine ran TwinCAT. The FARO and controller clocks are independent. We register coordinates from the survey reference and subtract one constant time offset estimated from the measured reflector and controller pose crossing the five leg midplanes. Those crossings give offsets of 1.5331–1.5412 s, median 1.5357 s. Absolute transport latency cannot be distinguished from clock offset. [Replication data, Reichenbach, Clar, Pott and Verl, CC BY 4.0](https://doi.org/10.18419/DARUS-4614).

## Measured agreement

The validator samples predicted XYZ at the reflector and the platform-side constraint force on each cable. The comparison uses 1,214 synchronized samples at 50 Hz. Cable forces exclude the initialization sample. Position RMS is the RMS Euclidean distance. Force RMS pools individual cable errors.

| Input or predictor | Reflector RMS (mm) | Peak (mm) | Cable force RMS (N) |
|---|---:|---:|---:|
| Recorded motor targets | 25.4 | 48.8 | 176.6 |
| Recorded motor encoders | 15.8 | 21.2 | 176.2 |
| Logged Cartesian target plus fixed reflector offset | 27.0 | | |
| Hold each cable at its measured P0 force | | | 184.8 |

The table below uses stationary intervals beginning 0.5 s after the platform target stops. Repeat scatter is the root sum of the tracker coordinate variances across repeated visits in the earlier PlaneA fixture. It estimates repeatability at rest. It does not measure clock uncertainty during motion.

| Pose | Encoder replay reflector RMS (mm) | Repeat scatter (mm) | Encoder replay force RMS (N) | Net force from measured tensions and gravity (N) |
|---|---:|---:|---:|---:|
| P1 | 2.5 | 0.63 | 75 | 126 |
| P2 | 16.5 | 0.66 | 16 | 64 |
| P3 | 19.3 | 0.70 | 18 | 77 |
| P4 | 18.8 | 0.40 | 303 | 180 |
| P5 | 19.3 | 0.48 | 216 | 88 |

![Settled reflector error and measured repeat scatter by pose](settled-error.png)

P4 is the clearest failure. Even after settling, the simulation misses the recorded cable forces by 303 N RMS. There is also an independent observability problem. Multiplying each measured force by its nominal span direction and adding the weight leaves 180 N of net force at P4. A stationary platform requires the actual external forces to balance. That calculation assumes the nominal platform orientation, the home pulley tangents and that the recorded forces act directly on the platform. Verifying force measurement location, transmission friction and actual attachment directions would help distinguish an error in that mapping from a material-law error. A single tracked point cannot resolve platform orientation. The results cover one standard-control cycle. Transfer to other payloads and adaptive control remains open.

## Reproduce

From the repository root, run the following commands. Both validations use the bundled WASM module, machine scene, motion streams and observations.

```sh
node scripts/reference_data/validate_4614_replay.mjs planeATargets
node scripts/reference_data/validate_4614_replay.mjs planeAEncoders
```

With the raw measurements mounted, rebuild the comparison inputs with `.venv/bin/python scripts/reference_data/extract_4614_replay.py`. Recompile the nominal firmware examples with `.venv/bin/python scripts/compile_machine_commands.py --machine copacabana`.

Local run ID is `copacabana-planeA-preload` at simulation revision `2bc87ece`. The observation fixture SHA-256 is `8cfe5279136f11093bffc81c13168a73c1451540daf12a9a3c2fa627f6683f2d`. The target and encoder streams have SHA-256 prefixes `ca921001a3818ffd` and `7e912375d1d57628`.

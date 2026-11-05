# Research Questions

## Context
Focus on the current sweep-collection path from the Python autocal entrypoint into the Node collector, including how collector arguments are normalized, how simulator startup is chosen, and how final commands are sent back to firmware. Pay particular attention to the existing RRF-specific helpers and the separate Klipper integration stack, along with the tests that currently pin those behaviors.

## Questions
1. How does `autocal/autocal.py` trace from top-level CLI parsing into `full_auto_loop`, and where does it currently decide how collector arguments, simulator lifecycle, and final calibration-command sends are handled?
2. How do the shared helpers in `autocal/_autocal_common.py` encode firmware or simulator assumptions today, including target resolution, simulation defaults, simulator config selection, process startup, readiness checks, and G-code sending?
3. How does `autocal/control/cli/collect_sweep_data.mjs` trace from CLI parsing into bridge creation and sweep execution, and which parts of that flow are tied to the current RRF bridge and simulator model?
4. How do `autocal/control/primitives/encoder_utils.mjs`, `autocal/control/primitives/machine_type.mjs`, and `autocal/control/behaviors/sweep_data_collection.mjs` divide responsibility for machine configuration, movement timing, hp-sim coordination, and simulator startup during sweep collection?
5. What entrypoints and runtime patterns already exist under `integrations/klipper/*` for starting a Klippy-backed session, sending commands, collecting motion or encoder-related data, and forwarding events to hp-sim, and how do those flows differ from the RRF path?
6. What tests currently cover the Python autocal CLI and the Node collector around `--sim`, collector argument forwarding, simulator startup, and bridge behavior, and what assumptions about the active firmware path are those tests enforcing?

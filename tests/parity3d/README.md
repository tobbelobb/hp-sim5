# Headless differential 3D checks

Run with Node on PATH and the repository Python environment:

```bash
.venv/bin/python -m pytest tests/python/cable_joints_3d/test_differential_parity.py -q
```

Every fixture executes the **production JavaScript implementation**, then the
native Python implementation, and compares initial state and every timestep.
`contract.json` lists the state fields to compare; adapters only construct ECS
components, invoke systems and serialize fields. Relationships use fixture
names, quaternions use XYZW order and compare up to sign. Query and solver order
remain observable. Arrays are copied when snapshots are taken.

For JSON snapshots of a single fixture:

```bash
node tests/parity3d/oracle.mjs tests/fixtures/python_3d_parity/rigid_members.json
PYTHONPATH=src/python:tests/python/cable_joints_3d .venv/bin/python -m parity_harness tests/fixtures/python_3d_parity/rigid_members.json
```

Fixtures specify `entities`, `systems` in execution order, `steps` with `dt`, and
explicit numeric `tolerance` (`atol`, `rtol`). Optional `resources`, `set` mutations
before each step, deferred `addComponents`, `queries`, `initializeRigidBodies`
and `attachments` exercise state transitions and frames. No viewer, browser or
worker is needed. Unknown components/systems and nonfinite outputs fail.

Current coverage:

- `motion`: prediction, world angular frames, PBD velocities, kinematic/static,
  grabbed, zero-dt, pause/error behavior.
- `inertia`: small rotated SPD and rotated rank-2/rank-1/zero PSD inverse moments.
- `query_order`: smallest-store insertion order, including stable query ties.
- `rigid_members`: moving/rotating parents, member offsets, projected spools,
  reference transport, live attachment frames and internal/external reactions.
- `distance_members`: off-center rigid reactions with anisotropic inertia,
  compliant accumulated multipliers, internal endpoints and successive steps.
- `spool_projection`: tilted axes, swing rejection, off-axis angular velocity,
  encoder fallback axes and angle unwrapping through several turns.

Rigid-member, distance and spool fixtures also run for 200 successive steps;
both engines must reproduce their own snapshots exactly on a second run.

These fixtures establish the covered ECS behavior. Authored USDA construction,
the cable/motor pipeline, full machines and richer Rerun recordings remain on
the checklist in `PYTHON_3D_PARITY.md`.

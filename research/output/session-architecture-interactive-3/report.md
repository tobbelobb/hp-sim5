# Interactive tool validation

Called capabilities, then run_experiment(steps=2, record=false, label="interactive-baseline").
Run ID: aeb62bbdbdda4527978599f294bb852d; status complete; warnings empty.
Default scene: public/usd_scenes/hp4_rigid_body.usda; dt=0.002 s; commands omitted (hold targets).
Read numerical telemetry with read_experiment(start_step=0, limit=3): steps 0, 1, 2 at 0, 0.002, 0.004 s.
Effector z: 0.10000000000000002, 0.09996827964069877, 0.09991644562710301 m.
Peak cable force: 0.03322521631182571 N.
Peak absolute tracking error: 0.00014098163905150053 rad; peak missed steps: 0.
Effector displacement: 0.00008355437291262743 m.
Visual inspection skipped because record=false. No source edits or delegation.
This validates the requested tool round trip only; it does not establish robot performance.

[Manifest](../experiments/aeb62bbdbdda4527978599f294bb852d/manifest.json)
[Inputs](../experiments/aeb62bbdbdda4527978599f294bb852d/commands.json)
[Numerical telemetry](../experiments/aeb62bbdbdda4527978599f294bb852d/telemetry.jsonl)

Awaiting the user's next message.

## Steered interactive tool validation

Baseline history retained: interactive-baseline, run ID aeb62bbdbdda4527978599f294bb852d.
Next run: interactive-steered, run ID 165d697d3d5f4e18b62065fa65ec5703.
Called run_experiment(steps=3, commands=[{"type":"Move","A":0.001}], record=false, label="interactive-steered").
Fresh default HP4 scene; dt=0.002 s; the A target is an absolute motor angle in radians and holds for remaining steps.
Status complete; warnings empty.
Read numerical telemetry with read_experiment(start_step=0, limit=4): steps 0–3 at 0, 0.002, 0.004, 0.006 s.

| Step | Effector z (m) | A target (rad) | A encoder (rad) | A tracking error (rad) |
| --- | --- | --- | --- | --- |
| 0 | 0.10000000000000002 | 0 | 0 | 0 |
| 1 | 0.09996827964069877 | 0.001 | 0.001999166770827133 | 0.0009991667708271331 |
| 2 | 0.09991644562710301 | 0.001 | 0.0011211977691044065 | 0.00012119776910440644 |
| 3 | 0.09985732886612055 | 0.001 | 0.00038714107347965793 | -0.0006128589265203421 |

Peak cable force: 0.055044361494350956 N.
Peak absolute length error: 0.00004696083822874897 m.
Peak absolute tracking error: 0.0009991667708271331 rad.
Peak missed steps: 0. Effector displacement: 0.00014267113392338278 m.
Visual inspection skipped because record=false. No source edits or delegation.
The requested run changes both duration and command schedule; no improvement claim is made.
Provenance also reports different Python source hashes: baseline c628896cc103a248ec22dfe504342183cadcd36aeb6bff7c7523f697708c8c3f; steered e769f8a797de0bf0db6e94e6d29b404976502a8e22edeebc568f8809785d16f2. Scene hash, Git revision and timestep match. This validation did not investigate the source hash difference.

[Steered manifest](../experiments/165d697d3d5f4e18b62065fa65ec5703/manifest.json)
[Steered inputs](../experiments/165d697d3d5f4e18b62065fa65ec5703/commands.json)
[Steered numerical telemetry](../experiments/165d697d3d5f4e18b62065fa65ec5703/telemetry.jsonl)

Awaiting the user's next message.

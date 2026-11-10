from pathlib import Path
import json,math,shutil
S=Path(__file__).resolve().parent;R=Path.cwd();D=R/'research/programs/program-0a82df7fd4b34b8181716630492a1af4/lower-force';D.mkdir(exist_ok=True)
M=json.loads((S/'evidence/measurements.json').read_text());names=['n-default','n-candidate','d-default','d-candidate','e-default','e-candidate'];done=all(n in M for n in names)
def val(x):return f'{x:.4f}' if isinstance(x,(float,int)) else ('not comparable' if x is None else str(x))
rows=[]
for n in names:
 if n not in M:continue
 x=M[n];a=x.get('applied') or {};e=a.get('per_anchor_error_mm',{});rows.append('| '+' | '.join([n,x['category'],*[val(e.get(k)) for k in 'ABCD'],val(a.get('max_mm')),val(a.get('rmse_mm')),str(a.get('all_below5')),str(a.get('anchors_over20')),str(x['physical_collected_points']),str(x['dataset_sweep_pairs'])+'/'+str(x['bidirectional_physical_sub_sweeps']),str(x['fit_count']),str(x['total_steps']),val(x['simulated_s']),val(x['wall_s'])])+' |')
pairs=[]
for p,w in [('n','nominal'),('d','development'),('e','evaluation')]:
 if p+'-default' not in M or p+'-candidate' not in M:continue
 b,c=M[p+'-default'],M[p+'-candidate'];bm=(b.get('applied') or {}).get('max_mm');cm=(c.get('applied') or {}).get('max_mm');pairs.append({'world':w,'default_max_mm':bm,'candidate_max_mm':cm,'max_improvement_mm':bm-cm if bm is not None and cm is not None else None,'max_improvement_percent':100*(bm-cm)/bm if bm and cm is not None else None,'effort_ratio':c['total_steps']/b['total_steps'] if b['normal_automatic_acceptance'] and c['normal_automatic_acceptance'] else None,'completed_both':b['normal_automatic_acceptance'] and c['normal_automatic_acceptance'],'readback_both':b['readback_agrees_at_print_precision'] and c['readback_agrees_at_print_precision']})
(S/'evidence/pairs.json').write_text(json.dumps(pairs,indent=2)+'\n')
partial=(S/'partial-decision.json').exists();status='controlled partial' if partial else ('complete' if done else 'in progress');decision='No adoption decision yet. Complete the frozen matrix without retuning.'
if done:
 pp={x['world']:x for x in pairs};ev=pp['evaluation'];supported=all(x['completed_both'] and x['readback_both'] and x['effort_ratio']<=1.20 for x in pairs) and ev['max_improvement_mm']>=.5 and ev['max_improvement_percent']>=10 and pp['development']['max_improvement_mm']>0
 decision=('The frozen candidate meets the pre-result applied-accuracy, transfer, completion and effort gates. Scope-safe production delivery still requires checks and separate review.' if supported else 'The frozen candidate fails at least one pre-result adoption gate. Preserve the counterexample and retain the production5×midpoint default. Do not retune this candidate using these worlds.')
 (S/'evidence/decision.json').write_text(json.dumps({'supported':supported,'decision':decision,'pairs':pairs},indent=2)+'\n')
if partial:
 decision='Default force adoption is blocked by the preserved nominal candidate workflow failure. Evaluation pair NOT TESTED. The manager will fund only the two unrun evaluation jobs and focused transport follow-up in a fresh bounded continuation. No successful run will be repeated.'
 (S/'evidence/decision.json').write_text(json.dumps({'supported':False,'finding':'inconclusive','production_force_adoption':'blocked','decision':decision,'tested_primary_runs':list(M),'unrun_primary_runs':[n for n in names if n not in M],'pairs':pairs,'partial_control':json.loads((S/'partial-decision.json').read_text())},indent=2)+'\n')
abstract=f'''# HP3 lower sensor measurement force under motor drag and encoder noise

Status: {status}.

We compare the fixed4×midpoint sensor measurement force with the current5×midpoint rule in six fresh paired HP3 autocalibrations. The nominal and development worlds precede a heldout evaluation pair. All worlds use known30mm radii, zero buildup, no layering or guide friction, native Rust, and seeded Gaussian encoder-read noise. Motor Coulomb and smoothed-stiction factors vary by axis. {len(M)} of six primary collections have completed or failed. {decision} A small hypothetical three-world simulation cannot establish hardware reliability or a lower failure rate. Applied anchor errors come from the rounded M669 command and a checked firmware readback. Every intermediate fit, failure and full command/noise journal is preserved separately from final applied geometry.

Topics: HP3, autocalibration, measurement force, motor friction, encoder noise, applied anchors.
'''
abstract+='''
## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Lower sensor force improves applied accuracy across the frozen worlds without new failure or >20% extra effort | n-default, n-candidate, d-default, d-candidate | Inconclusive for transfer; development max improves22.81% with6.41% extra simulated effort; nominal candidate workflow failed | [Applied results](evidence/measurements.json) |
| An explicit encoder deadline is silently capped at5s in the shared transport | source and nominal failed trace only | Not tested by the authorized real-WebSocket discriminator yet | [Failure excerpt](evidence/n-candidate-transport-excerpt.jsonl) |

## Decision handoff

- Scope: four fresh simulated HP3 attempts at basea68d6adf, known30mm radii, zero buildup/guide friction, layering off and the fixed nominal/development motors and noise.
- Review: producer hash/readback and offline geometry checks; independent fixed-evidence review pending.
- Decision: keep production5mid force default. The nominal failure stays in every denominator. Evaluation pair NOT TESTED in the controlled partial.
- Next: only the two frozen evaluation jobs and the separate bounded transport follow-up. Do not repeat successful jobs, retune policy, restore an old world or erase the failed trial.

## References

- [Full report](report.md) gives per-anchor geometry, effort and intermediate tails.
- [Research notebook](research.md) records the frozen decision, manager steering and reporting corrections.
- [Frozen policy](policy-frozen.json) preserves exact worlds, code/native hashes and pre-result adoption margin.
'''
report=abstract+'''
## Frozen decision and method

Base implementation is a68d6adffec3aea86a55a17d6b4d65a27badd42a. The candidate sensor-4mid-v1 changes only the isolated helper default from min(forceMax,5*forceMid) to min(forceMax,4*forceMid). Explicit overrides, pullouts at explicit20*midpoint, direct2*midpoint preloads, force tuning, guards, settling, L-BFGS-B fitting, seed selection and automatic stopping remain unchanged. All three copied collector files come from current base. Current service source retains upstream nonfatal recording behavior. Imports were rebased to the current worktree to preserve ECS component identity. No old prepare.py or manual engine.prepare was used.

Before primary results, the manager agreed to operationalize meaningful improvement as evaluation applied maximum error reduced by at least10% and0.5mm, with development maximum also improved. All paired runs must complete with no new failure and each completed pair must use <=1.20×baseline physics steps and simulated time. Nominal regression is disclosed even when other gates pass. Human endpoints remain any anchor>20mm or incomplete calibration failure and every anchor<5mm good. The policy and hashes were frozen before the first primary collection. Truth only builds/evaluates the fixed plant; it does not choose measurements, forces, starts, selection or stopping.

| World | Coulomb A/B/C/D factors | Stiction A/B/C/D factors | Read sigma deg | Seed |
| --- | --- | --- | ---: | ---: |
| nominal | 1/1/1/1 | 1/1/1/1 | .03 | 4001 |
| development | .5/2/1.5/1 | 2/.5/1/1.5 | .03 | 4101 |
| evaluation | .5/1.5/2/.75 | 1.5/2/.5/1 | .05 | 4201 |

Fresh loaded effective defaults were0.001Nm Coulomb and0.0015Nm stiction. The1001step-per-world torque probe checked ECS and packed native fields after the ordinary first step. At +0.01Nm target for1000steps, A encoder response differed from nominal by0.9120rad in development and0.9196rad in evaluation. Guide native mu remained0. This establishes intervention activation, not hardware realism. Supervisor preflight and the separate command-context socket probe passed. The attached idle supervisor used stock layering; each independent primary harness uses config_hp3.g, zero buildup and layering off, checked in its manifest.

## Applied results

Errors are Euclidean anchor distances in millimetres. RMSE is sqrt(mean of the four squared Euclidean distances). The four-decimal presentation does not increase the precision of M669's0.01mm coordinate rounding. Firmware query and applied command agreement is checked to0.005mm per coordinate, and both geometries are preserved. Sweep counts distinguish persisted dataset pairs from completed bidirectional physical sub-sweeps, counted by final point index in all point journals. Started sub-sweeps are separate result fields.

| Run | Outcome | A mm | B mm | C mm | D mm | Max mm | RMSE mm | Every<5 | Anchors>20 | Points | Pair/sub sweeps | Fits | Steps | Sim s | Wall s |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
'''+ '\n'.join(rows)+'\n\n'+ '\n'.join(f"{x['world']}: max improvement {val(x['max_improvement_mm'])}mm, {val(x['max_improvement_percent'])}%; candidate/default effort {val(x['effort_ratio'])}. Both normal completion={x['completed_both']}; both readback checks={x['readback_both']}." for x in pairs)+'''

Wall time includes fitting and host scheduling. Jobs run sequentially under a2CPU allocation. Use steps and simulated time for the effort criterion; do not interpret wall-time differences as policy speed effects. Matched seeds share a random-stream prefix, while adaptive workloads consume later noise values at different physical times.

## Intermediate tails and failures

Final accepted geometry and intermediate fit accuracy are separate endpoints. Production-valid or solver-success flags do not establish accuracy.

'''+ '\n'.join(f"- {n}: {x['fit_count']} fits across {x['iteration_count']} iterations, tail maximum {val(x['iteration_tail_max_mm'])}mm, {x['iteration_any_anchor_over20']} fits with any anchor>20mm, {x['invalid_fits']} invalid-flagged fits, {x['underconstrained_penalty_fits']} underconstrained-penalty fits; stop={x['stop_reason']}, normal acceptance={x['normal_automatic_acceptance']}." for n,x in M.items())+'''

## Evidence and limits

Each run directory contains a readable result, frozen policy, candidate diff and raw-artifact-manifest.json. The compact gzip raw archive retains dataset, every fit and selection state, full auto log, complete command/noise journals, passive initial/terminal/native snapshots, input/invocation and exact harness. Every file was verified by SHA256 after archive readback. Each completed or failed collection was committed and pushed before the next collection. No raw evidence is omitted. The compact review copy, when present, excludes duplicate events.jsonl only and points to each full raw archive and hash.

Only three hypothetical worlds and one seed per world were tested. No hardware data, guide-friction intervention, layout, stiffness, payload, spool-radius or buildup generalization is claimed. The motor model uses smooth friction and does not represent ideal static breakaway. A baseline with no final>20mm placement cannot support a claim of fewer such failures.

The nominal timeout trace preserves request10812 with valid finite A/B/C/D response followed by the5000ms timeout at the same simulator step1,347,144. The event journal lacks wall timestamps per response, so exact reply latency and original delay cause remain unestablished. Source inspection finds createGcodeBridge caps command encoder resolution at5000ms despite explicit120000ms configuration. A separate real-WebSocket stock-vs-fix check is authorized only after allsix primary jobs; it is NOT TESTED in a controlled four-attempt partial. The timeout remains in every denominator and cannot be erased by confirmation or source inspection.\n\nReporting errata are preserved in research.md. Completed physical sub-sweeps now come from final point indices rather than only finalized dataset metadata, so the aborted nominal candidate correctly has5completed/6started despite0finalized dataset pairs. An auxiliary process-wall field originally lacked a failure fallback; final summary uses the correct406.4474s manifest wall. Main reported wall and physical metrics were already correct. Original raw archives stay immutable.\n\nReferences: the current CONTINUE.md and continuation-setup/policy-frozen.json define this continuation. The earlier workers/attempt-939c073f49004eb6949910600200be51/report.md documents the rejected guide-preload experiment. Native and source hashes, plant settings and all exceptions are in each run's policy and raw manifest. research/autocal-recording-performance.md supports recording none for this numerical matrix, not a calibration accuracy claim.
'''
if partial:
 report=report.replace('Only three hypothetical worlds and one seed per world were tested.', 'Calibration was attempted in TWO hypothetical worlds with one seed per world. The motor activation probe covered THREE declared plants. The heldout evaluation calibration pair is NOT TESTED in this controlled partial.')
report=report.replace('- [Full report](report.md) gives per-anchor geometry, effort and intermediate tails.\n','')
for p,text in [(S/'abstract.md',abstract),(S/'report.md',report),(D/'abstract.md',abstract),(D/'report.md',report),(D/'research.md',(S/'research.md').read_text())]:p.write_text(text)
for f in ['policy-frozen.json','budget-ledger.json']:
 if (S/f).exists():shutil.copyfile(S/f,D/f)
shutil.copyfile(S/'evidence/measurements.json',D/'measurements.json');shutil.copyfile(S/'evidence/pairs.json',D/'pairs.json')
print(json.dumps({'status':status,'recorded_runs':len(M),'decision':decision}))

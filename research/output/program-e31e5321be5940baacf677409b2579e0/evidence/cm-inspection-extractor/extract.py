"""CM-E1 saved-evidence inspection; standard library only, no numerical replay.

Run from repo root with .venv/bin/python /absolute/session/extract.py.
Only four allowlisted scientific inputs are read. All writes stay beside this file.
Values are producer evidence; arithmetic and set checks test internal consistency.
"""
from pathlib import Path
import hashlib
import json
import math

SESSION = Path(__file__).resolve().parent
ORIGINAL = Path('/home/torbjorn/repos/hp-sim5/output/research/sessions/2026-10-23-consensus-membership')
NAMES = ('report.md', 'results.json', 'audit.json', 'inputs.json')
EXPECTED = {
    'report.md': '2e47bc589aea727b2e76387ff8fc3339b664134145f15d61df7fbbc7f46ef96a',
    'results.json': 'bdd85e0f758ce3a4f0ef4f5be3c0fe094fb830dfa6f3c059d39d7a2d804ab1cd',
    'audit.json': '6750c2ac25c1f99a90cc10dd3d98389422e95a726e725bfdd1b1e6c8aac9939e',
    'inputs.json': 'f58ef95d783a265b7153cefc07d2d8785f77896846bb07d58317b8a009634b44',
}


def main():
    copies = SESSION / 'sources'
    copies.mkdir(exist_ok=True)
    source_records = []
    docs = {}
    for name in NAMES:
        # Once copied, reruns can inspect the self-contained bundle alone.
        original = ORIGINAL / name
        path = original if original.is_file() else copies / name
        raw = path.read_bytes()
        digest = hashlib.sha256(raw).hexdigest()
        if digest != EXPECTED[name]:
            raise ValueError(f'Source identity changed: {path}: {digest}')
        target = copies / name
        if not target.exists():
            target.write_bytes(raw)
        if target.read_bytes() != raw:
            raise ValueError(f'Copy mismatch: {target}')
        source_records.append(dict(original=str(original), copy=f'sources/{name}',
                                   bytes=len(raw), sha256=digest, byte_identical=True))
        docs[name] = json.loads(raw) if name.endswith('.json') else raw.decode()
    r, a, inputs = (docs[n] for n in NAMES[1:])
    rows = r['rows']
    checks = []

    def check(name, passed, paths):
        checks.append(dict(check=name, passed=bool(passed), source_paths=paths))

    check('source copies below 300000 bytes', sum(x['bytes'] for x in source_records) < 300000, ['sources/*'])
    check('inputs manifest actual hash equals saved results reference',
          EXPECTED['inputs.json'] == r['inputs_manifest_sha256'], ['results.json#/inputs_manifest_sha256', 'inputs.json#'])
    check('eleven unique aligned fixtures across JSON files',
          len(rows) == len(a['rows']) == len(inputs) == len({x['fixture'] for x in rows}) == 11
          and [x['fixture'] for x in rows] == [x['fixture'] for x in a['rows']] == [x['fixture'] for x in inputs],
          ['results.json#/rows', 'audit.json#/rows', 'inputs.json#'])
    check('eleven distinct recorded observation hashes agree with recorded count',
          len({x['sweeps_sha256'] for x in a['rows']}) == a['unique_observation_datasets'] == 11,
          ['audit.json#/rows/*/sweeps_sha256', 'audit.json#/unique_observation_datasets'])
    summaries = {}
    for cohort, saved in r['summary'].items():
        subset = [x for x in rows if x['cohort'] == cohort]
        observed = dict(datasets=len(subset), improved=sum(x['delta_sum_anchor_error_mm'] < 0 for x in subset),
                        worsened=sum(x['delta_sum_anchor_error_mm'] > 0 for x in subset),
                        unchanged=sum(x['delta_sum_anchor_error_mm'] == 0 for x in subset))
        for key in ('baseline_sum_anchor_error_mm', 'variant_sum_anchor_error_mm', 'delta_sum_anchor_error_mm'):
            if key.startswith('baseline'):
                total = sum(x['all_candidates']['sum_anchor_error_mm'] for x in subset)
            elif key.startswith('variant'):
                total = sum(x['finite_prediction_only']['sum_anchor_error_mm'] for x in subset)
            else:
                total = sum(x[key] for x in subset)
            check(f'{cohort}: saved {key} agrees with stored-row arithmetic',
                  math.isclose(total, saved[key], rel_tol=0, abs_tol=1e-10),
                  [f'results.json#/summary/{cohort}/{key}', 'results.json#/rows'])
        check(f'{cohort}: stored outcome counts agree', all(saved[k] == v for k, v in observed.items()),
              [f'results.json#/summary/{cohort}', 'results.json#/rows'])
        summaries[cohort] = dict(pointer=f'results.json#/summary/{cohort}', inspected_producer_values=saved)
    extracted = []
    changed = []
    for index, row in enumerate(rows):
        ar, inp = a['rows'][index], inputs[index]
        base = f'results.json#/rows/{index}'
        audit = f'audit.json#/rows/{index}'
        ipath = f'inputs.json#/{index}'
        candidates = {x['iteration']: x for x in ar['candidates']}
        check(f'{index}: input identity and producer revision references agree',
              row['input_sha256'] == inp['sha256'] and row['input'] == inp['input']
              and row['cohort'] == inp['cohort'] and row['producer_source_revision'] == inp['producer_source_revision'],
              [base, ipath])
        check(f'{index}: candidate iteration list and chosen error agree with audit',
              row['candidate_iterations'] == list(candidates) and row['single_fit_sum_anchor_error_mm'] == candidates[row['single_fit_iteration']]['sum_anchor_error_mm'],
              [base + '/candidate_iterations', base + '/single_fit_sum_anchor_error_mm', audit + '/candidates'])
        saved_delta = row['finite_prediction_only']['sum_anchor_error_mm'] - row['all_candidates']['sum_anchor_error_mm']
        check(f'{index}: saved delta agrees with stored-error subtraction',
              math.isclose(saved_delta, row['delta_sum_anchor_error_mm'], rel_tol=0, abs_tol=1e-12),
              [base + '/delta_sum_anchor_error_mm', base + '/all_candidates/sum_anchor_error_mm', base + '/finite_prediction_only/sum_anchor_error_mm'])
        changed_files = sorted(k for k in row['producer_source_sha256']
                               if row['producer_source_sha256'][k] != r['current_source_sha256'].get(k))
        check(f'{index}: recorded changed-source list agrees with recorded hash maps',
              changed_files == sorted(row['changed_source_files']),
              [base + '/producer_source_sha256', base + '/changed_source_files', 'results.json#/current_source_sha256'])
        future = set(row['chosen_fit_future_fold_ids'])
        selected_train = set(candidates[row['single_fit_iteration']]['training_sweep_ids'])
        arm_training = {}
        arm_extract = {}
        for arm in ('all_candidates', 'finite_prediction_only'):
            model = row[arm]
            members = model['members'] or [row['single_fit_iteration']]
            train = selected_train.union(*(set(candidates[it]['training_sweep_ids']) for it in members))
            arm_training[arm] = train
            check(f'{index}: {arm} overlap IDs agree with saved training IDs',
                  sorted(future & train) == model['future_fold_training_overlap_ids'],
                  [base + '/chosen_fit_future_fold_ids', base + '/' + arm + '/future_fold_training_overlap_ids', audit + '/candidates'])
            arm_extract[arm] = {k: model[k] for k in ('members', 'sum_anchor_error_mm', 'chosen_fit_future_fold',
                                                      'future_fold_training_overlap_ids', 'common_unseen_fold')}
        common = sorted(future - arm_training['all_candidates'] - arm_training['finite_prediction_only'])
        check(f'{index}: common fold agrees with saved future/training ID set difference',
              common == row['common_unseen_fold_ids'], [base + '/common_unseen_fold_ids', base + '/chosen_fit_future_fold_ids', audit + '/candidates'])
        identical_models = all(row['all_candidates'][k] == row['finite_prediction_only'][k]
                               for k in ('anchors_mm', 'applied_spool_model'))
        item = dict(fixture=row['fixture'], cohort=row['cohort'], results_pointer=base,
                    audit_pointer=audit, inputs_pointer=ipath, machine_saved=inp['machine'],
                    delta_sum_anchor_error_mm=row['delta_sum_anchor_error_mm'],
                    common_unseen_fold_ids=row['common_unseen_fold_ids'], applied_models_identical=identical_models,
                    all_candidates=arm_extract['all_candidates'], finite_prediction_only=arm_extract['finite_prediction_only'])
        extracted.append(item)
        if row['delta_sum_anchor_error_mm'] != 0:
            details = dict(item)
            details.update(single_fit_iteration=row['single_fit_iteration'],
                           candidate_iterations=row['candidate_iterations'], eligible_iterations=row['eligible_iterations'],
                           failed_prediction_iterations=row['failed_prediction_iterations'],
                           chosen_fit_future_fold_ids=row['chosen_fit_future_fold_ids'],
                           saved_training_coverage=ar['candidates'],
                           variant_only_unseen_future_ids=sorted(future - arm_training['finite_prediction_only']),
                           baseline_only_unseen_future_ids=sorted(future - arm_training['all_candidates']),
                           reported_available_sweeps=ar['sweeps'])
            changed.append(details)
    check('all eight nonchanged applied anchor/spool models are identical',
          sum(x['applied_models_identical'] for x in extracted) == 8,
          ['results.json#/rows/*/all_candidates', 'results.json#/rows/*/finite_prediction_only'])
    check('saved prediction-match flags all true and saved failure lists empty',
          all(x['single_fit_prediction_matches_saved'] and not x['failed_prediction_iterations'] for x in rows),
          ['results.json#/rows/*/single_fit_prediction_matches_saved', 'results.json#/rows/*/failed_prediction_iterations'])
    evidence = dict(
        inspection_id='CM-SAVED-A1', historical_experiment_id='CM-E1',
        evidence_class='Inspected producer evidence. No independent numerical selection, geometry error, prediction or physics recomputation.',
        scope='Four named CM-E1 archived sources only; string/identity/set/arithmetic consistency checks.',
        source_revision_assignment='a1b4e559887550da6cac6a4c67819a618a766f65',
        archived_evaluation_revision=r['source_revision'], archived_fit_producer_revisions=sorted({x['producer_source_revision'] for x in inputs}),
        sources=source_records, source_copy_bytes=sum(x['bytes'] for x in source_records),
        path_convention='RFC 6901 JSON pointers after filename#. Extracted row fields retain source keys; append /key to results_pointer. Training coverage uses audit_pointer + /candidates; machine_saved uses inputs_pointer + /machine. Wildcards in check paths denote all array elements, not literal JSON pointers.',
        saved_summary=summaries, saved_rows=[{
            **{k: v for k, v in item.items() if k not in ('all_candidates', 'finite_prediction_only')},
            **{arm: {k: item[arm][k] for k in ('members', 'sum_anchor_error_mm')}
               for arm in ('all_candidates', 'finite_prediction_only')}
        } for item in extracted], changed_rows=changed,
        producer_recorded_budget=dict(pointer='results.json#/budget_used', values=r['budget_used']),
        this_audit_budget=dict(collections=0, fresh_fits=0, physics_runs=0, physics_steps=0,
                              prediction_evaluations=0, selection_evaluations=0, recording='none'),
        producer_audit_claims=dict(archived_inputs_and_script_match_results=a['archived_inputs_and_script_match_results'],
                                   pointer='audit.json#/archived_inputs_and_script_match_results', independently_reverified=False),
        consistency_checks=checks, failed_consistency_checks=[x for x in checks if not x['passed']],
        limitations=['No raw/gzip envelopes, probe/audit scripts or source-code bytes inspected.',
                     'Saved sweep hashes and producer script/source hashes are recorded identities, not rehashed originals.',
                     'Distinct saved content hashes do not establish independent fixture families or hardware worlds.',
                     'Machine labels are preserved verbatim: hp3-named inputs are labeled hangprinter_4 in manifest.',
                     'Initial exploratory shape printer assumed inputs.json was an object; it is an array and raised AttributeError. Corrected parser reads array; no scientific trial ran.'],
        decision='Withhold unconditional finite-prediction-only final-consensus adoption in archived CM-E1 scope; common-unseen predictive superiority unresolved.')
    (SESSION / 'evidence.json').write_text(json.dumps(evidence, separators=(',', ':'), allow_nan=False) + '\n')
    print(json.dumps(dict(source_copy_bytes=evidence['source_copy_bytes'], checks=len(checks),
                          failed_checks=len(evidence['failed_consistency_checks']), changed=[x['fixture'] for x in changed])))
    if evidence['failed_consistency_checks']:
        raise SystemExit('Saved-source inconsistency; inspect evidence.json')


if __name__ == '__main__':
    main()

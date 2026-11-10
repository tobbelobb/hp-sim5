"""Use regression checks with the known fixed radii omitted from anchor-only logs."""
from dataclasses import replace
import json

def parse_fixed_radii(ns, path):
    parsed = ns['parse_log_file'](path)
    # --find-radii off keeps the explicit CLI radii [30,30,30,30] fixed, and
    # autocal omits M666/effective radii lines. The battery parser requires them.
    # Supply only this known input; retain the unmodified raw logs as evidence.
    if parsed.summary is None:
        parsed.summary = ns['parse_summary'](path.read_text() + '\nLine model (M666): M666 R30:30:30:30\n')
    parsed.iterations = [replace(it, radii=[30.0]*4) if it.radii is None else it for it in parsed.iterations]
    assert parsed.summary is not None and len(parsed.summary.anchors)==4
    assert len(parsed.iterations)==5 and all(it.anchors is not None for it in parsed.iterations)
    return parsed

def numeric_metrics(path, *, include_settings=True):
    rows=[]
    text=path.read_text()
    if not any(line.startswith('{') for line in text.splitlines()):
        # The extended recorder keeps these records in a separate journal.
        text=path.with_name(path.name.replace('.full_auto.log','.full_auto_log.jsonl')).read_text()
    for line in text.splitlines():
        if line.startswith('{'):
            value=json.loads(line)
            if 'runs' in value and 'iteration' in value:
                # Includes full precision costs, uncertainties, scores and settings;
                # excludes timestamps, file paths and collection-only bookkeeping.
                row={key:value.get(key) for key in ('iteration','runs','selected_run','cost',
                    'score_ui','raw_fit_score_ui','history_rank_score','selection_rank_score')}
                if not include_settings:
                    row['runs']=[{key:run.get(key) for key in ('id','metrics')} for run in row['runs']]
                rows.append(row)
    assert len(rows)==5
    return rows

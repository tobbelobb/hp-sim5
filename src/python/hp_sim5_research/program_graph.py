"""Typed, citation-preserving program memory. Structural support is not admission."""
from .program import ProgramError, _identity, _json, _object, _text


GRAPH_KINDS = ('claim', 'derivation', 'relation', 'assessment', 'review_event')
RELATIONS = ('support', 'challenge', 'scope_limit', 'provenance', 'supersedes', 'decision')
EVENTS = ('withdrawal', 'applicability', 'supersession')


def identities(value, label, *, nonempty=False):
    if not isinstance(value, list) or (nonempty and not value):
        raise ProgramError(f'{label} must be a list' + (' with at least one ID' if nonempty else ''))
    for identity in value:
        _text(identity, label)
    if len(value) != len(set(value)):
        raise ProgramError(f'{label} contains duplicate input identities')
    return value


def validate_artifact(details):
    provenance = _object(details.get('provenance'), 'artifact provenance')
    if 'config' in provenance:
        _object(provenance['config'], 'artifact config')
    if 'input_identities' in provenance:
        identities(provenance['input_identities'], 'artifact input_identities')


def validate_graph(records):
    """Validate a whole candidate graph before writing, including snapshot imports."""
    def require(identity, kinds=None):
        _text(identity, 'reference ID')
        record = records.get(identity)
        if record is None:
            raise ProgramError(f'Missing graph reference: {identity}')
        if kinds and record['kind'] not in kinds:
            raise ProgramError(f'{identity} must identify {"/".join(kinds)}')
        return record

    versions, dependencies = set(), {}
    for record in records.values():
        kind, d = record['kind'], record['details']
        if kind not in GRAPH_KINDS:
            continue
        if kind == 'claim':
            _object(d.get('scope'), 'claim scope')
            _text(d.get('claim_key'), 'claim_key')
            if type(d.get('version')) is not int or d['version'] < 1:
                raise ProgramError('Claim version must be a positive integer')
            key = (d['claim_key'], d['version'])
            if key in versions:
                raise ProgramError(f'Duplicate claim version: {key}')
            versions.add(key)
            if d.get('knowledge_state') != 'unassessed':
                raise ProgramError('Claims remain unassessed until the reviewed admission path')
        elif kind == 'derivation':
            require(d.get('conclusion_id'), ('claim',))
            premises = identities(d.get('premise_ids'), 'premise_ids')
            evidence = identities(d.get('evidence_ids'), 'evidence_ids')
            if not premises and not evidence:
                raise ProgramError('A derivation requires premises or evidence')
            if set(premises) & set(evidence):
                raise ProgramError('Duplicate input identities across premises and evidence')
            for identity in premises:
                premise = require(identity, ('claim', 'artifact'))
                if premise['kind'] == 'claim':
                    dependencies.setdefault(d['conclusion_id'], set()).add(identity)
            for identity in evidence:
                require(identity, ('artifact',))
            _text(d.get('reason'), 'derivation reason')
        elif kind == 'relation':
            relation = d.get('relation')
            if relation not in RELATIONS:
                raise ProgramError('Unknown relation; required_premise edges belong inside derivations')
            source, target = require(d.get('source_id')), require(d.get('target_id'))
            _text(d.get('reason'), 'relation reason')
            if relation == 'provenance' and target['kind'] != 'artifact':
                raise ProgramError('Provenance target must be an artifact')
            if relation in ('support', 'challenge', 'scope_limit') and (
                    source['kind'] not in ('claim', 'artifact', 'assessment') or
                    target['kind'] not in ('claim', 'question', 'approach', 'decision')):
                raise ProgramError('Evidence relations require claim/artifact/assessment to claim/question/approach/decision')
            if relation == 'decision' and (source['kind'] != 'decision' or target['kind'] not in ('approach', 'question')):
                raise ProgramError('Decision relations require a decision and an approach/question')
            if relation == 'supersedes':
                if source['kind'] != 'claim' or target['kind'] != 'claim':
                    raise ProgramError('Supersession requires two claim versions')
                new, old = source['details'], target['details']
                if (not new.get('claim_key') or new.get('claim_key') != old.get('claim_key') or
                        type(new.get('version')) is not int or type(old.get('version')) is not int or
                        new['version'] <= old['version']):
                    raise ProgramError('Supersession requires a later version of the same claim_key')
        elif kind == 'assessment':
            require(d.get('subject_id'), ('claim', 'derivation', 'question'))
            if d.get('outcome') not in ('supported', 'rejected', 'mixed', 'inconclusive', 'not_tested'):
                raise ProgramError('Unknown reported assessment outcome')
            if d.get('review_state') != 'unreviewed' or d.get('admission_state') != 'unassessed':
                raise ProgramError('Reported assessments cannot admit knowledge; reviewed admission is required')
            _text(d.get('actor'), 'assessment actor')
            for identity in identities(d.get('evidence_ids'), 'assessment evidence_ids', nonempty=True):
                require(identity, ('artifact',))
        else:
            require(d.get('subject_id'), ('claim', 'artifact', 'derivation'))
            if d.get('event') not in EVENTS:
                raise ProgramError('Unknown graph review event')
            _text(d.get('actor'), 'event actor')
            _object(d.get('context'), 'event context')
            for key in ('affected_claim_ids', 'affected_derivation_ids'):
                for identity in identities(d.get(key), key):
                    require(identity, ('claim',) if key == 'affected_claim_ids' else ('derivation',))
    active, visited = set(), set()
    def visit(identity):
        if identity in active:
            raise ProgramError(f'Cyclic required premises at claim {identity}')
        if identity in visited:
            return
        active.add(identity)
        for premise in dependencies.get(identity, ()):
            visit(premise)
        active.remove(identity)
        visited.add(identity)
    for identity in dependencies:
        visit(identity)


class EvidenceGraph:
    """Append-only records on ProgramStore; all public mutations are atomic."""
    def __init__(self, store):
        self.store = store

    def _records(self, program_id):
        self.store.get_program(program_id)
        return {r['id']: r for r in (self.store._decode(row) for row in self.store.connection.execute(
            'SELECT * FROM records WHERE program_id=? ORDER BY rowid', (program_id,)))}

    def _insert(self, program_id, kind, title, details, record_id=None):
        identity = _identity(kind, record_id)
        records = self._records(program_id)
        if identity in records or self.store.connection.execute('SELECT 1 FROM programs WHERE id=?', (identity,)).fetchone():
            raise ProgramError(f'Duplicate graph identity: {identity}')
        records[identity] = {'id': identity, 'kind': kind, 'details': details}
        validate_graph(records)
        return self.store._insert(program_id, kind, title, details, identity)

    def add_claim(self, program_id, title, scope, *, prior_id=None, record_id=None, expected_revision=None):
        scope = _object(scope, 'claim scope')
        with self.store.transaction():
            self.store._touch(program_id, expected_revision)
            identity = _identity('claim', record_id)
            prior = self.store.get_record(program_id, prior_id, 'claim') if prior_id else None
            details = {'scope': scope, 'claim_key': prior['details']['claim_key'] if prior else identity,
                       'version': prior['details']['version'] + 1 if prior else 1, 'knowledge_state': 'unassessed'}
            result = self._insert(program_id, 'claim', title, details, identity)
            if prior:
                self._insert(program_id, 'relation', 'New claim version',
                             {'relation': 'supersedes', 'source_id': identity, 'target_id': prior_id,
                              'reason': 'Later scoped version; historical measurement is preserved'})
                self._event(program_id, prior_id, 'supersession', 'New claim version requires review',
                            {'new_claim_id': identity}, 'manager')
        return result

    def add_derivation(self, program_id, conclusion_id, premise_ids, evidence_ids, reason, *,
                       record_id=None, expected_revision=None):
        with self.store.transaction():
            self.store._touch(program_id, expected_revision)
            return self._insert(program_id, 'derivation', _text(reason, 'reason'),
                                {'conclusion_id': conclusion_id, 'premise_ids': premise_ids,
                                 'evidence_ids': evidence_ids, 'reason': reason}, record_id)

    def add_relation(self, program_id, relation, source_id, target_id, reason, *,
                     record_id=None, expected_revision=None):
        with self.store.transaction():
            self.store._touch(program_id, expected_revision)
            return self._insert(program_id, 'relation', _text(reason, 'reason'),
                                {'relation': relation, 'source_id': source_id, 'target_id': target_id,
                                 'reason': reason}, record_id)

    def add_assessment(self, program_id, subject_id, outcome, evidence_ids, reason, *, actor='importer',
                       record_id=None, expected_revision=None):
        with self.store.transaction():
            self.store._touch(program_id, expected_revision)
            return self._insert(program_id, 'assessment', _text(reason, 'reason'),
                                {'subject_id': subject_id, 'outcome': outcome, 'evidence_ids': evidence_ids,
                                 'actor': actor, 'review_state': 'unreviewed', 'admission_state': 'unassessed'}, record_id)

    @staticmethod
    def _affected(records, subject_id):
        affected, derivations = {subject_id}, set()
        changed = True
        while changed:
            changed = False
            for r in records.values():
                d = r['details']
                if r['kind'] == 'derivation' and (r['id'] in affected or
                        affected.intersection(d['premise_ids'] + d['evidence_ids'])):
                    derivations.add(r['id'])
                    if d['conclusion_id'] not in affected:
                        affected.add(d['conclusion_id'])
                        changed = True
        return sorted(i for i in affected if records[i]['kind'] == 'claim'), sorted(derivations)

    def _event(self, program_id, subject_id, event, reason, context, actor, record_id=None):
        records = self._records(program_id)
        self.store.get_record(program_id, subject_id)
        claims, derivations = self._affected(records, subject_id)
        return self._insert(program_id, 'review_event', _text(reason, 'reason'),
                            {'subject_id': subject_id, 'event': event, 'context': context, 'actor': actor,
                             'affected_claim_ids': claims, 'affected_derivation_ids': derivations}, record_id)

    def review_event(self, program_id, subject_id, event, reason, *, context=None, actor='manager',
                     record_id=None, expected_revision=None):
        with self.store.transaction():
            self.store._touch(program_id, expected_revision)
            return self._event(program_id, subject_id, event, reason,
                               {} if context is None else context, actor, record_id)

    @staticmethod
    def _evaluation(records):
        withdrawn = {r['details']['subject_id'] for r in records.values()
                     if r['kind'] == 'review_event' and r['details']['event'] == 'withdrawal'}
        derivations = [r for r in records.values() if r['kind'] == 'derivation']
        supported = {i for i, r in records.items() if r['kind'] == 'artifact' and i not in withdrawn}
        valid = set()
        # Least fixed point: a premise claim needs a surviving grounded derivation.
        for _ in range(len(records) + 1):
            added = False
            for r in derivations:
                d = r['details']
                if r['id'] not in withdrawn and set(d['premise_ids'] + d['evidence_ids']) <= supported:
                    valid.add(r['id'])
                    if d['conclusion_id'] not in withdrawn and d['conclusion_id'] not in supported:
                        supported.add(d['conclusion_id'])
                        added = True
            if not added:
                break
        return withdrawn, supported, valid

    @staticmethod
    def _edges(records):
        edges = []
        for r in records.values():
            d = r['details']
            if r['kind'] == 'relation':
                edges.append(dict(d, id=r['id']))
            elif r['kind'] == 'derivation':
                for identity in d['premise_ids'] + d['evidence_ids']:
                    edges.append({'relation': 'required_premise', 'source_id': identity,
                                  'target_id': d['conclusion_id'], 'derivation_id': r['id'], 'reason': d['reason']})
        return edges

    def query(self, program_id, query, subject_id=None, *, text='', scope=None):
        """Return full records/citations from one consistent program snapshot."""
        snapshot = self.store.snapshot(program_id)
        records = {r['id']: r for r in snapshot['records']}
        if subject_id is not None and subject_id not in records:
            raise ProgramError(f'Missing graph reference: {subject_id}')
        edges = self._edges(records)
        selected, selected_edges = set(), []
        if query == 'search':
            selected = {i for i, r in records.items() if text.casefold() in _json(r).casefold()}
        elif query == 'open_questions':
            selected = {i for i, r in records.items() if r['kind'] == 'question'
                        and r['details'].get('knowledge_state') == 'unassessed'}
        elif query in ('support', 'upstream', 'downstream', 'rationale'):
            if subject_id is None:
                raise ProgramError(f'{query} requires a subject ID')
            selected = {subject_id}
            if query in ('upstream', 'downstream'):
                changed = True
                while changed:
                    changed = False
                    for edge in edges:
                        if edge['relation'] != 'required_premise':
                            continue
                        start, end = (edge['target_id'], edge['source_id']) if query == 'upstream' else (edge['source_id'], edge['target_id'])
                        if start in selected:
                            if edge not in selected_edges:
                                selected_edges.append(edge)
                            if end not in selected:
                                selected.add(end)
                                changed = True
            else:
                selected_edges = [e for e in edges if e['target_id'] == subject_id or e['source_id'] == subject_id]
                for edge in selected_edges:
                    selected.update((edge['source_id'], edge['target_id']))
                if query == 'rationale':
                    selected.update(i for i, r in records.items() if r['kind'] == 'decision'
                                    and r['details'].get('subject_id') == subject_id)
        else:
            raise ProgramError(f'Unknown graph query: {query}')
        if scope is not None:
            _object(scope, 'scope filter')
            selected = {i for i in selected if all(records[i]['details'].get('scope', {}).get(k) == v
                                                  for k, v in scope.items())}
        primary = set(selected)
        # Citation closure is distinct from required-premise traversal: debate cycles
        # cannot create support but their source artifacts remain visible.
        changed = True
        while changed:
            before = set(selected)
            for i, r in records.items():
                d = r['details']
                if r['kind'] in ('route_plan', 'strategy_event') and (
                        d.get('approach_id') in selected or set(d.get('decision_ids', [])) & selected):
                    selected.add(i)
                if (r['kind'] == 'derivation' and d['conclusion_id'] in selected or
                        r['kind'] == 'assessment' and d['subject_id'] in selected or
                        r['kind'] == 'review_event' and (d['subject_id'] in selected or
                                                       set(d['affected_claim_ids']) & selected)):
                    selected.add(i)
                    selected.update(d.get('premise_ids', []) + d.get('evidence_ids', []))
                if r['kind'] == 'submission' and d['manifest']['claim']['id'] in selected:
                    selected.add(i)
                    selected.update(a['artifact_id'] for a in d['manifest']['assets'] if a['artifact_id'])
                if r['kind'] in ('submission_event', 'review_run', 'review', 'admission') and d['submission_id'] in selected:
                    selected.add(i)
                if r['kind'] == 'submission' and i in selected:
                    selected.add(d['manifest']['claim']['id'])
                if i in selected:
                    selected.update(d.get('evidence_ids', []))
                    selected.update(d.get('citation_ids', []))
                    if r['kind'] == 'investigation':
                        selected.update(d['route_plan_ids'])
                    if r['kind'] == 'route_plan':
                        selected.update(d['dependency_ids'])
                    if r['kind'] in ('submission_event', 'review_run', 'review', 'admission'):
                        selected.add(d['submission_id'])
                    if r['kind'] in ('assessment', 'review_event'):
                        selected.add(d['subject_id'])
                    if r['kind'] == 'relation' and d['relation'] == 'provenance':
                        selected.add(d['target_id'])
                if r['kind'] == 'relation' and d['target_id'] in selected:
                    selected.add(i)
                    selected.add(d['source_id'])
                    edge = dict(d, id=i)
                    if edge not in selected_edges:
                        selected_edges.append(edge)
                if r['kind'] == 'relation' and d['relation'] == 'provenance' and d['source_id'] in selected:
                    selected.update((i, d['target_id']))
            changed = selected != before
        withdrawn, supported, valid = self._evaluation(records)
        events = [r for r in records.values() if r['kind'] == 'review_event']
        affected_now = {}
        for event in events:
            claims, derivations = self._affected(records, event['details']['subject_id'])
            affected_now[event['id']] = {event['details']['subject_id'], *claims, *derivations}
        states = {}
        for i in selected:
            r = records[i]
            if r['kind'] not in ('claim', 'derivation', 'artifact'):
                continue
            states[i] = {'availability': 'withdrawn' if i in withdrawn else 'recorded',
                         'structural_support': i in (valid if r['kind'] == 'derivation' else supported),
                         'assessment_state': r['details'].get('knowledge_state', 'unassessed'),
                         'review_event_ids': [e['id'] for e in events if i in affected_now[e['id']]]}
            states[i]['review_required'] = bool(states[i]['review_event_ids'])
            states[i]['new_applicability'] = [
                {'context': e['details']['context'], 'state': 'unestablished'} for e in events
                if e['details']['event'] == 'applicability' and e['id'] in states[i]['review_event_ids']]
            if r['kind'] in ('claim', 'artifact'):
                submissions = [s for s in records.values() if s['kind'] == 'submission'
                               and (s['details']['manifest']['claim']['id'] == i or
                                    any(a['artifact_id'] == i for a in s['details']['manifest']['assets']))]
                submission_ids = {s['id'] for s in submissions}
                history = [a for a in records.values() if a['kind'] == 'admission'
                           and a['details']['submission_id'] in submission_ids]
                latest = {}
                for admission in sorted(history, key=lambda a: (a['created_at'], a['id'])):
                    latest[admission['details']['submission_id']] = admission['id']
                invalid = {e['details']['submission_id'] for e in records.values() if e['kind'] == 'submission_event'}
                live, review_history = [], []
                for admission in history:
                    d = admission['details']
                    submission = records[d['submission_id']]['details']['manifest']
                    evidence = {i, *(a['artifact_id'] for a in submission['assets'] if a['artifact_id'])}
                    withdrawn_review = any(e['details']['event'] == 'withdrawal' and
                                           evidence.intersection(affected_now[e['id']]) for e in events)
                    state = ('invalidated' if d['submission_id'] in invalid else
                             'withdrawn' if withdrawn_review else
                             'superseded' if latest[d['submission_id']] != admission['id'] else d['knowledge_state'])
                    review_history.append({'admission_id': admission['id'], 'state': state,
                                           'finding': d['finding'], 'decision': d['decision']})
                    if state == 'reviewed':
                        live.append(admission)
                states[i].update(submission_ids=[s['id'] for s in submissions],
                                 admission_ids=[a['id'] for a in history], review_history=review_history,
                                 reviewed_findings=[{'admission_id': a['id'], 'finding': a['details']['finding'],
                                                     'claim_id': a['details']['claim_id'],
                                                     'submission_id': a['details']['submission_id']} for a in live])
                if submissions:
                    states[i]['assessment_state'] = 'reviewed' if live else 'pending_review'
                reviewed_ids = {a['details']['submission_id'] for a in live}
                states[i]['pending_submission_ids'] = sorted(submission_ids - reviewed_ids)
                states[i]['review_required'] |= bool(submission_ids - reviewed_ids)
        for i in selected:
            r = records[i]
            if r['kind'] != 'submission':
                continue
            manifest = r['details']['manifest']
            reviews = [v for v in records.values() if v['kind'] == 'review' and v['details']['submission_id'] == i]
            admissions = sorted((a for a in records.values() if a['kind'] == 'admission' and a['details']['submission_id'] == i),
                                key=lambda a: (a['created_at'], a['id']))
            invalid = any(e['kind'] == 'submission_event' and e['details']['submission_id'] == i for e in records.values())
            evidence = {manifest['claim']['id'], *(a['artifact_id'] for a in manifest['assets'] if a['artifact_id'])}
            withdrawn_submission = any(e['details']['event'] == 'withdrawal' and
                                       evidence.intersection(affected_now[e['id']]) for e in events)
            states[i] = {'assessment_state': 'reviewed' if reviews else 'pending_review',
                         'review_ids': [v['id'] for v in reviews], 'admission_ids': [a['id'] for a in admissions],
                         'admission_state': ('invalidated' if invalid else 'withdrawn' if withdrawn_submission else
                                             admissions[-1]['details']['knowledge_state'] if admissions else 'pending'),
                         'candidate_revision': manifest['candidate_revision'], 'submission_sha256': r['details']['sha256']}
        artifacts = [records[i] for i in sorted(selected) if records[i]['kind'] == 'artifact']
        groups = {}
        for r in artifacts:
            d = r['details']
            # Hashes detect copied bytes; declared lineage catches different reports
            # over the same measurements. Unknown lineage proves no independence.
            for identity in ['sha256:' + d['sha256'], *d['provenance'].get('input_identities', [])]:
                groups.setdefault(identity, []).append(r['id'])
        return {'program_id': program_id, 'program_revision': snapshot['program']['revision'], 'query': query,
                'subject_id': subject_id, 'matches': sorted(primary),
                'records': [r for i, r in records.items() if i in selected],
                'relations': selected_edges, 'states': states, 'sources': artifacts,
                'shared_inputs': {k: v for k, v in groups.items() if len(v) > 1},
                'independence': 'Shared inputs are correlated; absent lineage does not establish independent confirmation.',
                'admission': 'Only explicit controller admission history records reviewed findings for fixed submissions; structural support is separate.'}

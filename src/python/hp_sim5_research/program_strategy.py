"""Recorded strategy actions for the existing manager conversation, with cited history."""
import hashlib

from .program import ProgramError, TRANSITIONS, _allocation, _json, _now, _object, _text
from .program_graph import identities

KINDS = ('route_plan', 'investigation', 'strategy_event', 'human_input')
FINDINGS = ('supported', 'rejected', 'mixed', 'unexpected', 'inconclusive', 'tool_failure')
WORK = ('experiment', 'scouting', 'literature', 'tooling')


def texts(value, label):
    if not isinstance(value, list):
        raise ProgramError(f'{label} must be a list of text')
    for item in value:
        _text(item, label)
    return value


def validate_strategy(records):
    """Validate references and recorded actions on mutation and portable restore."""
    def require(identity, kinds=None):
        _text(identity, 'strategy reference')
        record = records.get(identity)
        if record is None or kinds and record['kind'] not in kinds:
            raise ProgramError(f'Missing or mistyped strategy reference: {identity}')
        return record

    def refs(value, label, kinds=None, nonempty=False):
        for identity in identities(value, label, nonempty=nonempty):
            require(identity, kinds)

    for r in records.values():
        d, kind = r['details'], r['kind']
        if kind not in KINDS:
            continue
        refs(d.get('citation_ids'), 'citation_ids')
        if kind == 'route_plan':
            require(d.get('approach_id'), ('approach',))
            for key in ('hypothesis', 'uncertainty', 'decision_value', 'novelty', 'observation', 'reopening_condition', 'reason', 'retry_key'):
                _text(d.get(key), key)
            for key in ('tradeoffs', 'objections'):
                texts(d.get(key), key)
            _object(d.get('scope'), 'route scope')
            _allocation(d.get('estimated_cost'))
            if type(d.get('priority')) is not int or d['priority'] < 0:
                raise ProgramError('priority must be a nonnegative integer, lower is earlier')
            if type(d.get('retry_limit')) is not int or d['retry_limit'] < 1:
                raise ProgramError('retry_limit must be a positive integer')
            refs(d.get('dependency_ids'), 'dependency_ids', ('claim', 'artifact'))
        elif kind == 'investigation':
            assignment = require(d.get('assignment_id'), ('assignment',))
            if d.get('kind') not in WORK:
                raise ProgramError('Unknown investigation kind')
            route = assignment['details']['approach_id']
            plans = d.get('route_plan_ids')
            refs(plans, 'route_plan_ids', ('route_plan',), nonempty=True)
            if route not in [records[i]['details']['approach_id'] for i in plans]:
                raise ProgramError('Funded route requires its recorded route plan')
            explanations = d.get('explanations')
            if not isinstance(explanations, list) or len(explanations) < 2:
                raise ProgramError('An investigation requires at least two rival explanations')
            rivals = []
            for rival in explanations:
                _object(rival, 'rival explanation')
                require(rival.get('approach_id'), ('approach',))
                _text(rival.get('explanation'), 'rival explanation')
                rivals.append(rival['approach_id'])
            if len(set(rivals)) != len(rivals) or route not in rivals:
                raise ProgramError('Rivals must be distinct and include the funded route')
            if set(rivals) != {records[i]['details']['approach_id'] for i in plans}:
                raise ProgramError('Each rival requires one recorded route plan')
            outcomes = _object(d.get('outcomes'), 'outcomes')
            if not outcomes:
                raise ProgramError('Record outcome-dependent decisions before funding')
            for name, outcome in outcomes.items():
                _text(name, 'outcome name')
                _object(outcome, 'outcome')
                _text(outcome.get('observation'), 'discriminating observation')
                if outcome.get('finding') not in FINDINGS:
                    raise ProgramError('Unknown outcome finding')
                if outcome['finding'] == 'unexpected' and outcome.get('review_finding') not in ('supported', 'rejected', 'mixed', 'inconclusive'):
                    raise ProgramError('An unexpected observation requires its separate expected review_finding')
                actions = outcome.get('actions')
                if not isinstance(actions, list):
                    raise ProgramError('outcome actions must be a list')
                if outcome['finding'] == 'tool_failure' and actions:
                    raise ProgramError('A tool failure cannot reject or promote a research route')
                targets = set()
                for action in actions:
                    require(action.get('approach_id'), ('approach',))
                    if action['approach_id'] not in rivals or action['approach_id'] in targets:
                        raise ProgramError('Actions must address distinct recorded rivals')
                    targets.add(action['approach_id'])
                    if action.get('target') not in TRANSITIONS['approach']:
                        raise ProgramError('Unknown route target state')
                    _text(action.get('reason'), 'action reason')
                    _text(action.get('reopening_condition'), 'action reopening_condition')
                    _object(action.get('scope'), 'action scope')
                    plan = next(records[i]['details'] for i in plans if records[i]['details']['approach_id'] == action['approach_id'])
                    if action['scope'] != plan['scope']:
                        raise ProgramError('An outcome action must retain the exact funded route scope')
            execution = _object(d.get('execution'), 'execution guidance')
            if 'recording_mode' in execution and execution['recording_mode'] not in ('none', 'events', 'sampled', 'full'):
                raise ProgramError('Unsupported recording_mode')
            for key in ('source_limit', 'simulated_autocal_speedup'):
                if key in execution:
                    _allocation({'wall_minutes': execution[key]})
        elif kind == 'strategy_event':
            if d.get('event') not in ('decision', 'result', 'dependency_review', 'steering'):
                raise ProgramError('Unknown strategy event')
            for key in ('reason', 'basis'):
                _text(d.get(key), key)
            for identity in identities(d.get('approach_ids'), 'approach_ids'):
                require(identity, ('approach',))
            refs(d.get('review_event_ids'), 'review_event_ids', ('review_event', 'submission_event', 'admission'))
            if d['event'] == 'result':
                investigation = require(d.get('investigation_id'), ('investigation',))
                outcome = investigation['details']['outcomes'].get(d.get('outcome'))
                if outcome is None or d.get('finding') != outcome['finding'] or d.get('actions') != outcome['actions']:
                    raise ProgramError('Recorded result differs from its funded outcome actions')
            refs(d.get('decision_ids'), 'decision_ids', ('decision',))
            if d['event'] == 'decision':
                if d.get('target') not in TRANSITIONS['approach']:
                    raise ProgramError('Unknown decision target state')
                _object(d.get('scope'), 'decision scope')
                _text(d.get('reopening_condition'), 'decision reopening_condition')
        elif kind == 'human_input':
            require(d.get('steering_id'), ('steering',))
            if d.get('intent') not in ('note', 'hunch', 'instruction'):
                raise ProgramError('Human intent must be note, hunch or instruction')
            texts(d.get('links'), 'human links')
            _object(d.get('provenance'), 'human provenance')
            if d.get('next_assignment_id') is not None:
                require(d['next_assignment_id'], ('assignment',))
                if d['intent'] != 'instruction':
                    raise ProgramError('Only explicit instructions change the next allocation')


class ProgramStrategy:
    def __init__(self, store):
        self.store = store

    def records(self, program_id):
        return self.store.graph()._records(program_id)

    def _validate(self, program_id):
        validate_strategy(self.records(program_id))

    def _write(self, record):
        self.store.connection.execute('UPDATE records SET details_json=?, revision=revision+1, updated_at=? WHERE id=?',
                                      (_json(record['details']), _now(), record['id']))

    def _plans(self, records):
        plans = {}
        # _records reads SQLite rowid order; restore preserves the snapshot order.
        for r in records.values():
            if r['kind'] == 'route_plan':
                plans[r['details']['approach_id']] = r
        return plans

    def route(self, program_id, approach_id, hypothesis, scope, priority, estimated_cost, uncertainty,
              decision_value, novelty, observation, reopening_condition, reason, *, tradeoffs=None,
              objections=None, dependency_ids=None, citation_ids=None, retry_limit=2,
              record_id=None, expected_revision=None):
        """Record a versioned route rationale; lower priority numbers allocate first."""
        details = dict(approach_id=approach_id, hypothesis=hypothesis, scope=scope, priority=priority,
                       estimated_cost=estimated_cost, uncertainty=uncertainty, decision_value=decision_value,
                       novelty=novelty, observation=observation, reopening_condition=reopening_condition,
                       reason=reason, tradeoffs=[] if tradeoffs is None else tradeoffs,
                       objections=[] if objections is None else objections,
                       dependency_ids=[] if dependency_ids is None else dependency_ids,
                       citation_ids=[] if citation_ids is None else citation_ids, retry_limit=retry_limit,
                       retry_key=hashlib.sha256(_json([approach_id, hypothesis, scope, observation]).encode()).hexdigest())
        with self.store.transaction():
            self.store._touch(program_id, expected_revision)
            self.store.get_record(program_id, approach_id, 'approach')
            result = self.store._insert(program_id, 'route_plan', _text(reason, 'reason'), details, record_id)
            self._validate(program_id)
        return result

    def _event(self, program_id, event, reason, approach_ids, citation_ids, *, basis, review_event_ids=None, **extra):
        details = dict(event=event, reason=_text(reason, 'reason'), approach_ids=approach_ids,
                       citation_ids=list(dict.fromkeys(citation_ids)), review_event_ids=review_event_ids or [], basis=basis,
                       decision_ids=[], **extra)
        # Ordinary graph rationale queries reach the full strategy and review history.
        for approach_id in approach_ids:
            note = self.store._note(program_id, 'decision', reason, approach_id, [], 'manager')
            self.store.graph()._insert(program_id, 'relation', reason,
                                      dict(relation='decision', source_id=note['id'], target_id=approach_id, reason=reason))
            details['decision_ids'].append(note['id'])
        return self.store._insert(program_id, 'strategy_event', reason, details)

    def _change(self, program_id, approach_id, target, reason):
        route = self.store.get_record(program_id, approach_id, 'approach')
        current = route['details']['state']
        if target != current and target not in TRANSITIONS['approach'][current]:
            raise ProgramError(f'Invalid approach transition: {current} -> {target}')
        route['details'].update(state=target, reason=reason)
        self._write(route)

    def decide(self, program_id, approach_id, target, reason, scope, reopening_condition, citation_ids, *, expected_revision=None):
        """Change one route with scope and reopening reasons, preserving every prior decision."""
        with self.store.transaction():
            self.store._touch(program_id, expected_revision)
            records = self.records(program_id)
            plan = self._plans(records).get(approach_id)
            if plan is None or scope != plan['details']['scope']:
                raise ProgramError('Decision requires a recorded route plan and its exact scope')
            _text(reopening_condition, 'reopening_condition')
            identities(citation_ids, 'citation_ids', nonempty=True)
            self._change(program_id, approach_id, target, reason)
            event_ids = [r['id'] for r in records.values() if r['kind'] in ('review_event', 'submission_event', 'admission') and r['id'] in citation_ids]
            result = self._event(program_id, 'decision', reason, [approach_id], [plan['id'], *citation_ids],
                                 basis='Manager decision; scientific admission is separate', review_event_ids=event_ids,
                                 target=target, scope=scope, reopening_condition=reopening_condition)
            self._validate(program_id)
        return result

    def commission(self, program_id, approach_id, objective, allocation, explanations, outcomes, *, kind='experiment',
                   execution=None, citation_ids=None, record_id=None, expected_revision=None):
        """Fund a rival discriminator, scouting or tooling through the existing assignment path."""
        _allocation(allocation)
        with self.store.transaction():
            program = self.store._touch(program_id, expected_revision)
            board = self.board(program_id)
            records = self.records(program_id)
            plans = self._plans(records)
            route = self.store.get_record(program_id, approach_id, 'approach')
            entry = next(r for r in board['routes'] if r['id'] == approach_id)
            if program['state'] != 'active' or route['details']['state'] not in ('live', 'promising'):
                raise ProgramError('Funding requires an active program and live/promising route')
            if entry['review_required'] or entry['futile']:
                raise ProgramError('Review dependencies or reframe the exhausted route before funding')
            if not isinstance(explanations, list):
                raise ProgramError('explanations must be a list')
            rivals = [_object(r, 'rival explanation').get('approach_id') for r in explanations]
            if any(i not in plans for i in rivals):
                raise ProgramError('Record a credible route plan for every rival before funding')
            assignment = self.store._insert(program_id, 'assignment', objective,
                dict(approach_id=approach_id, question_id=route['details']['question_id'], allocation=allocation,
                     execution_state='ready', review_state='unreviewed', knowledge_state='unassessed'), record_id)
            guidance = dict(_object({} if execution is None else execution, 'execution guidance'))
            if allocation.get('collections', 0) > 0:
                guidance.setdefault('simulated_autocal_speedup', 25)
                guidance.setdefault('recording_mode', 'events')
            investigation = self.store._insert(program_id, 'investigation', objective,
                dict(assignment_id=assignment['id'], kind=kind, explanations=explanations, outcomes=outcomes,
                     route_plan_ids=[plans[i]['id'] for i in rivals], execution=guidance, citation_ids=citation_ids or []))
            self._validate(program_id)
        return {'assignment': assignment, 'investigation': investigation}

    def result(self, program_id, assignment_id, outcome, citation_ids, reason, *, expected_revision=None):
        """Apply a precommitted observation. A failed command is never a negative finding."""
        with self.store.transaction():
            self.store._touch(program_id, expected_revision)
            records = self.records(program_id)
            investigations = [r for r in records.values() if r['kind'] == 'investigation' and r['details']['assignment_id'] == assignment_id]
            if not investigations:
                raise ProgramError('No funded investigation for this assignment')
            investigation = investigations[-1]
            if any(r['kind'] == 'strategy_event' and r['details'].get('investigation_id') == investigation['id'] for r in records.values()):
                raise ProgramError('This investigation already has a recorded result')
            selected = investigation['details']['outcomes'].get(outcome)
            if selected is None:
                raise ProgramError('Choose a recorded outcome or reframe the investigation')
            assignment = self.store.get_record(program_id, assignment_id, 'assignment')
            if assignment['details']['execution_state'] in ('planned', 'ready', 'running'):
                raise ProgramError('End or cancel the worker before interpreting its result')
            identities(citation_ids, 'citation_ids', nonempty=True)
            cited = [self.store.get_record(program_id, i) for i in citation_ids]
            finding = selected['finding']
            if finding != 'tool_failure':
                required = selected['review_finding'] if finding == 'unexpected' else finding
                effective = self._effective_admissions(program_id, records, assignment_id, required)
                if not any(r['id'] in effective or r['kind'] == 'review' and any(
                        records[i]['details']['review_id'] == r['id'] for i in effective) for r in cited):
                    raise ProgramError('Scientific route decisions require an effective reviewed admission for this assignment and matching finding')
            approach_id = assignment['details']['approach_id']
            actions = selected['actions']
            current_plans = self._plans(records)
            for action in actions:
                funded_plan = next(records[i] for i in investigation['details']['route_plan_ids']
                                   if records[i]['details']['approach_id'] == action['approach_id'])
                if current_plans[action['approach_id']]['details']['retry_key'] != funded_plan['details']['retry_key']:
                    raise ProgramError('This outcome belongs to a prior route framing; record a scoped manager decision before changing the new route')
                self._change(program_id, action['approach_id'], action['target'], action['reason'])
            result = self._event(program_id, 'result', reason, sorted({approach_id, *(a['approach_id'] for a in actions)}),
                                 [investigation['id'], *citation_ids], basis=('Execution failure; no scientific finding' if finding == 'tool_failure' else
                                                                          'Effective reviewed finding for this fixed assignment and submission scope'),
                                 investigation_id=investigation['id'], finding=finding, outcome=outcome, actions=actions)
            # Futility counts inconclusive observations, never launch/tool failures.
            if finding == 'inconclusive':
                plan = next(records[i] for i in investigation['details']['route_plan_ids'] if records[i]['details']['approach_id'] == approach_id)
                if self._retries(self.records(program_id), approach_id, plan['details']['retry_key']) >= plan['details']['retry_limit']:
                    self._change(program_id, approach_id, 'parked', 'Inconclusive retry limit reached; reframe before repeating')
                    self._event(program_id, 'decision', 'Inconclusive retry limit reached; reframe before repeating', [approach_id],
                                [result['id'], plan['id']], basis='Futility limit; hypothesis remains unresolved',
                                target='parked', scope=plan['details']['scope'], reopening_condition=plan['details']['reopening_condition'])
            self._validate(program_id)
        return result

    def _effective_admissions(self, program_id, records, assignment_id, finding):
        latest = {}
        for r in records.values():
            if r['kind'] == 'admission':
                latest[r['details']['submission_id']] = r
        effective = set()
        for submission_id, admission in latest.items():
            d = admission['details']
            submission = records[submission_id]['details']['manifest']
            if (submission['assignment_id'] != assignment_id or d['finding'] != finding or
                    d['decision'] != 'keep' or d['knowledge_state'] != 'reviewed'):
                continue
            state = self.store.graph().query(program_id, 'support', d['claim_id'])['states'][d['claim_id']]
            if admission['id'] in {a['admission_id'] for a in state['reviewed_findings']}:
                effective.add(admission['id'])
        return effective

    @staticmethod
    def _retries(records, approach_id, retry_key):
        count = 0
        for r in records.values():
            if r['kind'] != 'strategy_event' or r['details'].get('finding') != 'inconclusive':
                continue
            investigation = records[r['details']['investigation_id']]['details']
            if records[investigation['assignment_id']]['details']['approach_id'] == approach_id and any(
                    records[i]['details']['retry_key'] == retry_key for i in investigation['route_plan_ids']):
                count += 1
        return count

    def board(self, program_id):
        """Reconstruct priorities and pending dependency review from durable graph history."""
        records = self.records(program_id)
        plans = self._plans(records)
        routes = []
        for route in records.values():
            if route['kind'] != 'approach':
                continue
            plan = plans.get(route['id'])
            dependencies, event_ids, surviving = {}, set(), []
            if plan:
                for identity in plan['details']['dependency_ids']:
                    query = self.store.graph().query(program_id, 'support', identity)
                    state = query['states'][identity]
                    dependencies[identity] = {'record': records[identity], 'state': state, 'sources': query['sources']}
                    event_ids.update(state['review_event_ids'])
                    latest_reviews = {}
                    for record in query['records']:
                        if record['kind'] == 'admission':
                            latest_reviews[record['details']['submission_id']] = record
                        elif record['kind'] == 'submission_event':
                            event_ids.add(record['id'])
                    event_ids.update(r['id'] for r in latest_reviews.values() if r['details']['knowledge_state'] != 'reviewed')
                    surviving += [i for i, s in query['states'].items() if records[i]['kind'] == 'derivation' and s['structural_support']]
            history = [r for r in records.values() if r['kind'] == 'strategy_event' and route['id'] in r['details']['approach_ids']]
            resolved = {i for r in history if r['details']['event'] == 'decision' for i in r['details']['review_event_ids']}
            pending = sorted(event_ids - resolved)
            count = self._retries(records, route['id'], plan['details']['retry_key']) if plan else 0
            routes.append(dict(id=route['id'], title=route['title'], state=route['details']['state'],
                               reason=route['details']['reason'], plan=plan, priority=plan['details']['priority'] if plan else 1000,
                               priority_reason=plan['details']['reason'] if plan else route['details']['reason'],
                               dependencies=dependencies, surviving_derivation_ids=sorted(set(surviving)),
                               review_event_ids=sorted(event_ids), pending_review_event_ids=pending,
                               review_required=bool(pending), inconclusive_count=count,
                               futile=bool(plan and count >= plan['details']['retry_limit']), history=history))
        program = self.store.get_program(program_id)
        return {'program_id': program_id, 'question': program['goal'], 'constraints': program['constraints'],
                'open_questions': [r for r in records.values() if r['kind'] == 'question' and r['details']['knowledge_state'] == 'unassessed'],
                'routes': sorted(routes, key=lambda r: (r['priority'], r['id'])),
                'human_inputs': [r for r in records.values() if r['kind'] == 'human_input'],
                'evidence_boundary': 'Reported/imported assessments remain unreviewed. Structural support is not scientific admission.'}

    def reconcile(self, program_id):
        """Queue route review after graph changes; retain a surviving alternative derivation."""
        board = self.board(program_id)
        for route in board['routes']:
            pending = route['pending_review_event_ids']
            already = {i for r in route['history'] if r['details']['event'] == 'dependency_review' for i in r['details']['review_event_ids']}
            if not set(pending) - already:
                continue
            supported = all(d['state']['structural_support'] for d in route['dependencies'].values())
            reason = ('Dependency changed; surviving derivations retain the route pending explicit review' if supported else
                      'A required dependency lost support; park pending review')
            with self.store.transaction():
                self.store._touch(program_id)
                if not supported and route['state'] in ('live', 'promising'):
                    self._change(program_id, route['id'], 'parked', reason)
                self._event(program_id, 'dependency_review', reason, [route['id']],
                            [*pending, *(d for d in route['surviving_derivation_ids'])],
                            basis='Graph traversal; historical observations are preserved', review_event_ids=pending)
                self._validate(program_id)
        return self.board(program_id)

    def steering(self, program_id, text, *, subject_id=None, evidence_ids=None, actor='human', record_id=None,
                 expected_revision=None, intent='note', links=None, provenance=None, next_assignment_id=None,
                 allocation=None):
        """Preserve notes and hunches; explicit instructions select or replace the next allocation."""
        with self.store.transaction():
            self.store._touch(program_id, expected_revision)
            if intent != 'instruction' and (next_assignment_id is not None or allocation is not None):
                raise ProgramError('Only explicit instructions change the next allocation')
            if allocation is not None and next_assignment_id is None:
                raise ProgramError('An allocation change requires next_assignment_id')
            note = self.store._note(program_id, 'steering', text, subject_id, evidence_ids or [], actor, record_id)
            next_id = next_assignment_id
            if next_id is not None:
                assignment = self.store.get_record(program_id, next_id, 'assignment')
                if assignment['details']['execution_state'] not in ('planned', 'ready'):
                    raise ProgramError('Steer pending work; running assignments continue until explicitly cancelled')
                if allocation is not None:
                    _allocation(allocation)
                    original = assignment['id']
                    assignment['details']['execution_state'] = 'cancelled'
                    self._write(assignment)
                    details = dict(assignment['details'], allocation=allocation, execution_state='ready')
                    replacement = self.store._insert(program_id, 'assignment', assignment['title'], details)
                    next_id = replacement['id']
                    for investigation in self.records(program_id).values():
                        if investigation['kind'] == 'investigation' and investigation['details']['assignment_id'] == original:
                            self.store._insert(program_id, 'investigation', investigation['title'],
                                               dict(investigation['details'], assignment_id=next_id, citation_ids=[note['id'], investigation['id']]))
            human = self.store._insert(program_id, 'human_input', text, dict(steering_id=note['id'], intent=intent,
                       links=links or [], provenance=provenance or {}, next_assignment_id=next_id,
                       citation_ids=[note['id'], *(evidence_ids or [])]))
            if next_id:
                route_id = self.store.get_record(program_id, next_id)['details']['approach_id']
                self._event(program_id, 'steering', text, [route_id], [human['id'], note['id']],
                            basis='Accepted explicit human instruction', next_assignment_id=next_id)
            self._validate(program_id)
        return dict(note, accepted_input=human)

    def next_allocation(self, program_id, board):
        records = self.records(program_id)
        routes = {r['id']: r for r in board['routes']}
        preferred = [r['details']['next_assignment_id'] for r in records.values() if r['kind'] == 'human_input'
                     and r['details']['intent'] == 'instruction' and r['details']['next_assignment_id']]
        pending = [r for r in records.values() if r['kind'] == 'assignment' and r['details']['execution_state'] in ('planned', 'ready')
                   and routes[r['details']['approach_id']]['state'] in ('live', 'promising')
                   and not routes[r['details']['approach_id']]['review_required'] and not routes[r['details']['approach_id']]['futile']]
        pending.sort(key=lambda r: (0 if preferred and r['id'] == preferred[-1] else 1,
                                    routes[r['details']['approach_id']]['priority'], r['created_at'], r['id']))
        if not pending:
            return dict(state='unallocated', allocation=None, reason='Choose a finite discriminator or review/reframe pending dependencies')
        r = pending[0]
        return dict(assignment_id=r['id'], state=r['details']['execution_state'], allocation=r['details']['allocation'],
                    reason=routes[r['details']['approach_id']]['reason'], approach_id=r['details']['approach_id'])

    @staticmethod
    def brief(board, allocation):
        lines = [board['question']]
        for route in board['routes']:
            lines.append('')
            plan = route['plan']
            refs = [plan['id']] if plan else []
            refs += [r['id'] for r in route['history'][-2:]]
            lines.append(f"{route['title']}: {route['state']}, priority {route['priority']}. {route['reason']} [{', '.join(refs) or route['id']}]")
            if plan:
                d = plan['details']
                lines.append(f"Priority reason: {route['priority_reason'].rstrip('.')}. Unknown: {d['uncertainty'].rstrip('.')}. Next observation: {d['observation']} [{plan['id']}]")
            for identity, dependency in route['dependencies'].items():
                state = dependency['state']
                lines.append(f"Recorded evidence: {dependency['record']['title']} Support survives: {state['structural_support']}; admission: {state['assessment_state']}. [{identity}]")
            if route['review_required']:
                lines.append('Review dependencies: ' + ', '.join(route['pending_review_event_ids']))
        lines.append('\nNext allocation: ' + _json(allocation))
        lines.append(board['evidence_boundary'])
        return '\n'.join(lines)

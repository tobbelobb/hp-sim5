"""Research missions and the canonical Research Program role files."""
from datetime import date
import hashlib
import json
from pathlib import Path
import re

from .store import ProgramError, _object, _text


ROLE_FILES = {
    'manager': 'MANAGER.md',
    'researcher': 'RESEARCHER.md',
    'reviewer': 'REVIEWER.md',
}
PORTFOLIO_VERSION = 1
PROMPT_KINDS = ('experiment', 'scouting', 'literature', 'tooling')
PROMPT_INTENTS = ('implementation', 'diagnosis', 'exploration', 'tooling')
MACHINES = ('hp3', 'hp4', 'any')


def role_instructions(role):
    """Load the role file directly; there are no alternate role prompts."""
    filename = ROLE_FILES.get(role)
    if filename is None:
        raise ProgramError(f'Unknown Research Program instruction role: {role}')
    return (Path(__file__).resolve().parents[1] / 'role-instructions' / filename).read_text()


def _text_list(value, label):
    if not isinstance(value, list) or not value:
        raise ProgramError(f'{label} must be a nonempty list of strings')
    for item in value:
        _text(item, label)
    return value


class PromptPortfolio:
    """Read-only registry of reusable research missions under research/prompts."""

    def __init__(self, root):
        self.root = Path(root).resolve()
        self.directory = self.root / 'research/prompts'
        self.path = self.directory / 'portfolio.json'

    def _load(self):
        try:
            data = json.loads(self.path.read_text())
        except json.JSONDecodeError as error:
            raise ProgramError(f'Invalid prompt portfolio JSON: {error}') from error
        _object(data, 'prompt portfolio')
        if data.get('version') != PORTFOLIO_VERSION:
            raise ProgramError(f'Prompt portfolio version must be {PORTFOLIO_VERSION}')
        policy = _text(data.get('selection_policy'), 'selection_policy')
        entries = data.get('prompts')
        if not isinstance(entries, list) or not entries:
            raise ProgramError('prompts must be a nonempty list')

        records, ids, filenames = [], set(), set()
        for entry in entries:
            _object(entry, 'prompt portfolio entry')
            identity = _text(entry.get('id'), 'prompt id')
            if not re.fullmatch(r'[a-z0-9][a-z0-9-]{0,63}', identity):
                raise ProgramError('Prompt ids must use lowercase letters, digits and hyphens')
            if identity in ids:
                raise ProgramError(f'Duplicate prompt id: {identity}')
            filename = _text(entry.get('path'), 'prompt path')
            path = Path(filename)
            if path.is_absolute() or len(path.parts) != 1 or path.name != filename or path.suffix not in ('.md', '.txt'):
                raise ProgramError('Prompt paths must name one .md or .txt file directly under research/prompts')
            if filename in filenames:
                raise ProgramError(f'Duplicate prompt path: {filename}')
            source = self.directory / filename
            if not source.is_file():
                raise ProgramError(f'Missing research prompt: {filename}')
            if entry.get('kind') not in PROMPT_KINDS:
                raise ProgramError(f'Unknown prompt kind for {identity}')
            if entry.get('intent') not in PROMPT_INTENTS:
                raise ProgramError(f'Unknown prompt intent for {identity}')
            machines = _text_list(entry.get('machines'), f'{identity} machines')
            if any(machine not in MACHINES for machine in machines):
                raise ProgramError(f'Unknown machine for {identity}')
            _text_list(entry.get('expected_deliverables'), f'{identity} expected_deliverables')
            _text_list(entry.get('related_evidence'), f'{identity} related_evidence')
            for key in ('title', 'decision_value', 'novelty', 'readiness', 'last_reviewed'):
                _text(entry.get(key), f'{identity} {key}')
            try:
                date.fromisoformat(entry['last_reviewed'])
            except ValueError as error:
                raise ProgramError(f'{identity} last_reviewed must be YYYY-MM-DD') from error
            record = dict(entry, path=f'research/prompts/{filename}',
                          sha256=hashlib.sha256(source.read_bytes()).hexdigest())
            records.append(record)
            ids.add(identity)
            filenames.add(filename)

        present = {path.name for path in self.directory.iterdir()
                   if path.is_file() and path.suffix in ('.md', '.txt')}
        unregistered = sorted(present - filenames)
        if unregistered:
            raise ProgramError('Unregistered research prompt(s): ' + ', '.join(unregistered) +
                               '; add mission metadata to portfolio.json or move role/plumbing instructions')
        return policy, records

    def inventory(self):
        """Return the complete mission portfolio without copying prompt bodies."""
        policy, records = self._load()
        return {'portfolio': str(self.path.relative_to(self.root)), 'selection_policy': policy,
                'prompts': records}

    def resolve(self, prompt_id, *, include_text=False):
        """Load one prompt selected during triage, optionally with exact instruction text."""
        if not isinstance(prompt_id, str) or not prompt_id:
            raise ProgramError('prompt_id must be a nonempty string')
        _, records = self._load()
        record = next((r for r in records if r['id'] == prompt_id), None)
        if record is None:
            raise ProgramError(f'Unknown prompt id: {prompt_id}')
        result = {'portfolio': str(self.path.relative_to(self.root)), 'prompt': record}
        if include_text:
            result['prompt'] = dict(record, text=(self.root / record['path']).read_text())
        return result

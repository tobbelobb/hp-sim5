"""Human-readable session names and a catalog built from short research abstracts."""
from datetime import datetime, timezone
import os
from pathlib import Path
import re
import tempfile
import unicodedata
from urllib.parse import quote
import uuid


def session_name(topic):
    ascii_topic = unicodedata.normalize('NFKD', topic).encode('ascii', 'ignore').decode()
    slug = re.sub(r'[^a-z0-9]+', '-', ascii_topic.lower()).strip('-')[:64].rstrip('-') or 'research'
    date = datetime.now(timezone.utc).strftime('%Y-%m-%d')
    return f'{date}-{slug}-{uuid.uuid4().hex[:8]}'


def initial_abstract(topic):
    title = ' '.join(topic.split())[:120]
    return (f'# {title}\n\n'
            'Status: not yet summarized.\n\n'
            '## Abstract\n\n'
            'The session has started. No research result has been summarized yet.\n\n'
            '## Topics\n\n'
            f'{title}\n\n'
            '## Tested hypotheses\n\n'
            '| Hypothesis | Experiment IDs | Outcome | Evidence |\n'
            '| --- | --- | --- | --- |\n'
            '| None recorded yet | None | Not tested | None |\n\n'
            '## References\n\n'
            '- [Original task](prompt.txt)\n'
            '- [Research notebook](research.md)\n')


def catalog_entries(root):
    root = Path(root)
    candidates = [p for parent in (root, root / 'sessions') if parent.is_dir()
                  for p in parent.iterdir() if p.is_dir()]
    # Prefer a descriptive alias over its old hash, and include each session once.
    candidates.sort(key=lambda p: (bool(re.fullmatch(r'[0-9a-f]{32}', p.name)), p.name))
    seen = set()
    for directory in candidates:
        resolved = directory.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        if not any((directory / name).is_file() for name in
                   ('abstract.md', 'report.md', 'research.md', 'prompt.txt', 'launch.json', 'exit.json')):
            continue
        abstract = directory / 'abstract.md'
        summary = (abstract.read_text() if abstract.is_file() else
                   f'# {directory.name}\n\nStatus: abstract missing. '
                   'Consult the original artifacts; tested hypotheses have not been cataloged.\n')
        yield directory, summary


def render_index(root, search=''):
    root = Path(root)
    parts = ['# Research session catalog\n\n'
             'Short abstracts and tested hypotheses from local sessions. '
             'Missing or pending summaries are not evidence of completed experiments.\n\n'
             'Regenerate with `.venv/bin/python scripts/research_index.py`.\n']
    for directory, summary in catalog_entries(root):
        if search.casefold() not in (directory.name + '\n' + summary).casefold():
            continue
        relative = directory.relative_to(root).as_posix()
        parts.append(f'## [{directory.name}]({quote(relative)}/)\n')
        links = [f'[{name}]({quote(relative)}/{name})' for name in
                 ('abstract.md', 'report.md', 'research.md') if (directory / name).is_file()]
        if links:
            parts.append(' · '.join(links) + '\n')
        # Anchor relative evidence links at their session while embedding its abstract.
        summary = re.sub(r'\]\(([^)]+)\)', lambda m: '](' + (
            quote(relative) + '/' + m[1] if not m[1].startswith(('/', '#')) and ':' not in m[1]
            else m[1]) + ')', summary)
        summary = re.sub(r'^(#+) ', r'##\1 ', summary, flags=re.MULTILINE)
        parts.append(summary)
    return '\n'.join(parts).rstrip() + '\n'


def write_index(root):
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    content = render_index(root)
    # Concurrent launchers must not leave a partly written catalog.
    with tempfile.NamedTemporaryFile(mode='w', dir=root, prefix='.index-', delete=False) as temporary:
        temporary.write(content)
    os.replace(temporary.name, root / 'index.md')

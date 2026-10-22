"""Static CLI inventory and literal evidence audit; no calibration is run."""
import ast
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
original = (ROOT / 'autocal-flags.md').read_text()
keep = {flag for line in original.splitlines() if line.startswith('KEEP')
        for flag in re.findall(r'--[\w-]+', line)}
flags = set(re.findall(r'--[a-z][a-z0-9-]*', original.split('Non-interactive')[1].split('\n## Investigation')[0]))
rows = {}
for filename in ['autocal/_autocal_common.py', 'autocal/autocal.py']:
    tree = ast.parse((ROOT / filename).read_text())
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute) or node.func.attr != 'add_argument':
            continue
        names = [x.value for x in node.args if isinstance(x, ast.Constant) and isinstance(x.value, str) and x.value.startswith('--')]
        if not names:
            continue
        kws = {x.arg: ast.unparse(x.value) for x in node.keywords}
        for name in names:
            if name in flags:
                rows.setdefault(name, []).append({'source': f'{filename}:{node.lineno}', **kws})
# Include ignored local evidence. Restrict extensions and size explicitly; never read credentials.
roots = ['autocal/data', 'output/research', 'research', 'autocal/tests', 'autocal/tools']
files = []
for root in roots:
    for p in (ROOT / root).rglob('*'):
        if p.is_file() and p.suffix in {'.log', '.jsonl', '.md', '.py', '.conf'} and 'autocal-flag-audit' not in p.parts:
            if p.stat().st_size <= 8_000_000:
                files.append(p)
files.append(ROOT / 'autocal/shotgun.conf')
files = sorted(set(files))
found = {flag: {'log_files': 0, 'other_files': 0, 'examples': []} for flag in flags}
pattern = re.compile(r'(?<![\w-])--[a-z][a-z0-9-]*(?![\w-])')
for p in files:
    seen = set()
    for n, line in enumerate(p.read_text(errors='replace').splitlines(), 1):
        for flag in set(pattern.findall(line)) & flags:
            row = found[flag]
            if flag not in seen:
                row['log_files' if p.suffix in {'.log', '.jsonl'} else 'other_files'] += 1
                seen.add(flag)
            if len(row['examples']) < 4:
                row['examples'].append({'source': f'{p.relative_to(ROOT)}:{n}', 'excerpt': line[:350]})
result = {'revision': subprocess.check_output(['git','rev-parse','HEAD'], cwd=ROOT, text=True).strip(),
          'method': 'Literal option spellings, file counts not invocations; includes help/comments. Absence is not non-use. Defaults/JSON keys not counted. Files >8 MB excluded.',
          'scanned_files': len(files), 'roots': roots,
          'flags': {f: {'keep': f in keep, 'definitions': rows.get(f, []), **found[f]} for f in sorted(flags)}}
(OUT / 'usage.json').write_text(json.dumps(result, indent=2) + '\n')
lines = ['# Flag inventory and literal usage evidence', '', result['method'], '',
         f"Revision `{result['revision']}`. Scanned {len(files)} files. See [raw evidence](usage.json) and [audit script](audit_usage.py).", '',
         'Definitions can include the variant parser, whose `None` means inherit. The first definition is generally the main parser; consult its source before interpreting defaults.', '',
         '| Flag | User KEEP | Definition / default expression | Log files | Other files | Example |',
         '| --- | --- | --- | ---: | ---: | --- |']
for f, r in result['flags'].items():
    defs = '; '.join(f"`{d['source']}`: `{d.get('default', '(argparse implicit)')}`" for d in r['definitions'])
    example = r['examples'][0]['source'] if r['examples'] else 'No literal match'
    lines.append(f"| `{f}` | {'KEEP' if r['keep'] else ''} | {defs} | {r['log_files']} | {r['other_files']} | `{example}` |")
(OUT / 'inventory.md').write_text('\n'.join(lines) + '\n')
print(json.dumps({'flags': len(flags), 'scanned_files': len(files), 'output': str(OUT / 'inventory.md')}))

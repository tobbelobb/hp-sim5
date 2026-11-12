"""Machine catalogue: which scene, firmware configs and compiled commands belong together.

machines/catalog.json is the single source of truth. Paths are relative to the repository root.
"""
import json
from pathlib import Path

CATALOG_PATH = Path(__file__).with_name('catalog.json')
CATALOG = json.loads(CATALOG_PATH.read_text())


def machine(name):
    """Return the machine whose canonical name or nickname matches name (case-insensitive)."""
    key = str(name).strip().lower()
    for entry in CATALOG['machines']:
        if key == entry['name'].lower() or key in (nickname.lower() for nickname in entry['nicknames']):
            return entry
    raise ValueError(f'Unknown machine {name!r}; see {CATALOG_PATH.relative_to(CATALOG_PATH.parents[1])}')


def line_layering_variant(entry, line_layering):
    """Return the machine with the same scene as entry and the requested line layering."""
    for candidate in CATALOG['machines']:
        if candidate['scene'] == entry['scene'] and candidate['line_layering'] == line_layering:
            return candidate
    raise ValueError(f'{entry["name"]} has no variant with line_layering={line_layering}')


def scene(entry):
    """Return the catalogue's description of entry's scene."""
    return CATALOG['scenes'][entry['scene']]

#!/usr/bin/env python3
"""Restore a pinned DaRUS dataset to a local data directory."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
import time
from pathlib import Path
import urllib.parse
import urllib.request

DATASETS = {
    '3600': ('doi:10.18419/darus-3600', 'Cable_Actuator_Test_Rig_Stuttgart_2023_DaRUS-3600'),
    '4614': ('doi:10.18419/DARUS-4614', 'Experimental_Investigation_2025_DaRUS-4614'),
    '5171': ('doi:10.18419/DARUS-5171', 'Experimental_modelling_of_synthetic_fibre_ropes_for_compensation_2026_DaRUS-5171'),
}
BASE = 'https://darus.uni-stuttgart.de/api/'

def digest(path, algorithm):
    h = hashlib.new(algorithm.lower().replace('-', ''))
    with path.open('rb') as f:
        while chunk := f.read(1024 * 1024):
            h.update(chunk)
    return h.hexdigest()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', choices=DATASETS)
    parser.add_argument('--workers', type=int, default=4, help='Concurrent downloads (default: 4)')
    parser.add_argument('--output', type=Path, help='Data directory (default: data/ under the dataset directory)')
    args = parser.parse_args()
    doi, dirname = DATASETS[args.dataset]
    root = (args.output or Path(__file__).resolve().parent / dirname / 'data').resolve()
    url = BASE + 'datasets/:persistentId/versions/1.0?' + urllib.parse.urlencode({'persistentId': doi})
    with urllib.request.urlopen(url, timeout=60) as response:
        files = json.load(response)['data']['files']
    print(f'{doi}: {len(files)} files, {sum(f["dataFile"].get("originalFileSize", f["dataFile"]["filesize"]) for f in files):,} bytes -> {root}', flush=True)
    if args.workers < 1:
        parser.error('--workers must be positive')

    def download(item):
        i, entry = item
        data = entry['dataFile']
        relative = Path(entry.get('directoryLabel', '')) / data.get('originalFileName', data['filename'])
        expected_size = data.get('originalFileSize', data['filesize'])
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError(f'Unsafe path: {relative}')
        target = root / relative
        checksum = data['checksum']
        if target.exists():
            if target.stat().st_size == expected_size and digest(target, checksum['type']) == checksum['value'].lower():
                print(f'[{i}/{len(files)}] verified {relative}', flush=True)
                return
            raise FileExistsError(f'Existing file differs; inspect it before retrying: {target}')
        target.parent.mkdir(parents=True, exist_ok=True)
        temp = target.with_name(target.name + '.download')
        print(f'[{i}/{len(files)}] downloading {relative}', flush=True)
        for attempt in range(5):
            try:
                h = hashlib.new(checksum['type'].lower().replace('-', ''))
                with urllib.request.urlopen(BASE + 'access/datafile/' + str(data['id']) + '?format=original', timeout=120) as response, temp.open('wb') as out:
                    while chunk := response.read(1024 * 1024):
                        out.write(chunk)
                        h.update(chunk)
                if temp.stat().st_size != expected_size or h.hexdigest() != checksum['value'].lower():
                    raise ValueError(f'Checksum or size mismatch: {relative}')
                os.replace(temp, target)
                return
            except (OSError, ValueError) as exc:
                temp.unlink(missing_ok=True)
                if attempt == 4:
                    raise
                print(f'Retrying {relative}: {exc}', flush=True)
                time.sleep(2 ** attempt)

    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        for _ in executor.map(download, enumerate(files, 1)):
            pass
    print('All files verified.')

if __name__ == '__main__':
    main()

"""Build or search the local research catalog without reading long reports."""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/python'))
from hp_sim5_research.archive import render_index, write_index


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT / 'output/research')
    parser.add_argument('--search', help='Print abstracts matching this phrase, including hypotheses and references')
    args = parser.parse_args()
    if args.search is not None:
        print(render_index(args.root, args.search), end='')
    else:
        write_index(args.root)
        print(args.root / 'index.md')


if __name__ == '__main__':
    main()

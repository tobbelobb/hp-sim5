"""Internal manager file-queue host. Ordinary chats start it through the CLI."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/python'))
from hp_sim5_research.program_host import serve

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--store', required=True, type=Path)
    parser.add_argument('--root', required=True, type=Path)
    args = parser.parse_args()
    from hp_sim5_research.cloud_environment import activate
    activate(args.root)
    raise SystemExit(serve(args.store, args.root))

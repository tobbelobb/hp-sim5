from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'src/python'))
sys.path.insert(0, str(ROOT / 'tests'))
sys.path.insert(0, str(ROOT / '.agents/skills/cloud-research-client/scripts'))
sys.path.insert(0, str(ROOT / 'research-program-feature'))
sys.path.insert(0, str(ROOT / 'research-program-feature/tests'))

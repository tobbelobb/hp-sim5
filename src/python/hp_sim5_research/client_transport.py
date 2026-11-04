"""Explicit client selection and the optional platform adapter boundary.

Local research never imports or needs the Codex Cloud adapter.
"""
import os
from pathlib import Path
import sys


def research_client():
    selected = os.environ.get('HP_SIM5_RESEARCH_CLIENT', 'local')
    if selected not in ('local', 'cloud'):
        raise ValueError('HP_SIM5_RESEARCH_CLIENT must be local or cloud')
    return selected


def client_adapter():
    if research_client() == 'cloud':
        scripts = Path(__file__).resolve().parents[3] / '.agents/skills/cloud-research-client/scripts'
        if not (scripts / 'codex_cloud/adapter.py').is_file():
            raise ValueError('Cloud research requires the cloud-research-client skill in .agents/skills')
        if str(scripts) not in sys.path:
            sys.path.insert(0, str(scripts))
        from codex_cloud import adapter
        return adapter
    return None


def activate(root):
    adapter = client_adapter()
    if adapter:
        adapter.activate(root)

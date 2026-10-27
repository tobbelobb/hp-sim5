"""Explicit client selection and the optional platform adapter boundary.

Local research never imports or needs the Codex Cloud adapter.
"""
import os


def research_client():
    selected = os.environ.get('HP_SIM5_RESEARCH_CLIENT', 'local')
    if selected not in ('local', 'cloud'):
        raise ValueError('HP_SIM5_RESEARCH_CLIENT must be local or cloud')
    return selected


def client_adapter():
    if research_client() == 'cloud':
        from .codex_cloud import adapter
        return adapter
    return None


def activate(root):
    adapter = client_adapter()
    if adapter:
        adapter.activate(root)

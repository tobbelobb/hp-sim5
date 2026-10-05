"""Native simulation MCP; stdout is reserved for the stdio protocol."""
from pathlib import Path
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/python'))

from mcp.server import MCPServer
from mcp_types import ToolAnnotations
from hp_sim5_research.experiments import DEFAULT_SCENE, MAX_STEPS, compare_runs, read_run, run_experiment as run

mcp = MCPServer('hp-sim5', instructions=(
    'Run bounded native Hangprinter experiments, read numeric telemetry, and compare trials. '
    'Each run starts from authored initial state and writes immutable inputs, JSON telemetry and an optional RRD. '
    'Use Rerun MCP for Viewer inspection. Motor angles are radians, geometry metres, forces newtons. '
    'Read research/AGENTS.md for the experiment and autocal workflow.'
))
READ_ONLY = ToolAnnotations(read_only_hint=True, open_world_hint=False)


@mcp.tool(annotations=READ_ONLY)
def capabilities() -> dict[str, Any]:
    """Discover native scenes, command units, artifact contracts and limits."""
    return {'scenes': [str(path.relative_to(ROOT)) for path in sorted((ROOT / 'public/usd_scenes').glob('*rigid_body.usda'))],
            'default_scene': DEFAULT_SCENE, 'max_steps': MAX_STEPS,
            'commands': {'Move': 'absolute motor angles in rad; axes object or top-level axis keys',
                         'Add to reference': 'reference angle increments in rad',
                         'SetTorqueMode': 'axis and torqueNm', 'SetPositionMode': 'axis',
                         '{}': 'hold current targets for one fixed timestep', 'E': 'deposited length in m'},
            'telemetry': 'Every step including zero: effectors, encoders, tracking errors, missed steps, lengths and forces',
            'limitations': ['Native commands are timestep-scheduled motor records, not G-code',
                            'Full-auto autocal collection still requires the browser/firmware bridge',
                            'Final snapshots are observations, not resumable physics checkpoints',
                            'Timings include telemetry and optional recording overhead']}


@mcp.tool(annotations=ToolAnnotations(read_only_hint=False, destructive_hint=False, open_world_hint=False))
def run_experiment(scene: str = DEFAULT_SCENE, steps: int = 200, dt: float | None = None,
                   commands: list[dict] | None = None, commands_file: str | None = None,
                   label: str = '', record: bool = True) -> dict[str, Any]:
    """Run fresh native physics. One command per step; return measurements and saved artifact paths.

    Use commands_file for large JSON arrays inside the repo. Do not supply both forms.
    {} holds targets. RRD opens in Rerun; read_run gives numeric data without a Viewer.
    """
    if commands_file is not None:
        if commands is not None:
            raise ValueError('Supply commands or commands_file, not both')
        from hp_sim5_research.experiments import repo_path
        import json
        commands = json.loads(repo_path(ROOT, commands_file).read_text())
    return run(ROOT, scene, steps=steps, dt=dt, commands=commands, label=label, record=record)


@mcp.tool(annotations=READ_ONLY)
def read_experiment(run_id: str, start_step: int | None = None, limit: int = 20) -> dict[str, Any]:
    """Read a manifest or at most 100 consecutive numeric samples starting at start_step."""
    return read_run(ROOT, run_id, start_step, limit)


@mcp.tool(annotations=READ_ONLY)
def compare_experiments(baseline_id: str, candidate_id: str) -> dict[str, Any]:
    """Compare complete trials with matching timelines; report changed inputs and physical differences."""
    return compare_runs(ROOT, baseline_id, candidate_id)


if __name__ == '__main__':
    mcp.run(transport='stdio')

"""Native simulation MCP; stdout is reserved for the stdio protocol."""
from pathlib import Path
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/python'))

from mcp.server import MCPServer
from mcp_types import ToolAnnotations
from hp_sim5_research.experiments import DEFAULT_SCENE, MAX_STEPS, compare_runs, read_run, run_experiment as run
from hp_sim5_research.services import supervised_request

mcp = MCPServer('hp-sim5', instructions=(
    'Run bounded native Hangprinter experiments, read numeric telemetry, and compare trials. '
    'run_experiment starts fresh; collect_sweeps and G-code use the launcher-owned persistent session. '
    'Experiments write immutable inputs, JSON telemetry and Rerun recordings. '
    'Use Rerun MCP for Viewer inspection. Motor angles are radians, geometry metres, forces newtons. '
    'Read research/AGENTS.md for the experiment and autocal workflow.'
))
READ_ONLY = ToolAnnotations(read_only_hint=True, open_world_hint=False)
MUTATION = ToolAnnotations(read_only_hint=False, destructive_hint=False, open_world_hint=False)


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
            'native_collection': {'machine': 'HP4', 'firmware': 'RRF',
                                  'operations': ['runtime_status', 'send_gcode', 'collect_sweeps', 'step_physics', 'reset_session', 'start_browser_service'],
                                  'encoders': 'Raw unwrapped degrees; A/B/C/D mapped to CAN 40/41/42/43',
                                  'clock': 'Persistent fixed-step world; collector delays advance physics; reads drain motion',
                                  'settling': 'Bounded in simulation time; default 30 seconds',
                                  'services': 'Launcher-owned runtime, firmware and bridge outside the command sandbox'},
            'limitations': ['Klipper streamed motion is not supported by native collection',
                            'Final snapshots are observations, not resumable physics checkpoints',
                            'Timings include telemetry and optional recording overhead']}


@mcp.tool(annotations=READ_ONLY)
def runtime_status() -> dict[str, Any]:
    """Inspect persistent native world, clock, command queue and supervised service health."""
    return supervised_request('status')


@mcp.tool(annotations=MUTATION)
def send_gcode(line: str) -> dict[str, Any]:
    """Send one G-code line through real RRF planning to the continuing native session.

    M569.3 reads complete queued motor motion. Use it before interpreting a movement result.
    The returned firmware reply alone does not establish physical settling.
    """
    return supervised_request('gcode', line=line)


@mcp.tool(annotations=MUTATION)
def collect_sweeps(configs: list[dict] | None = None, options: dict | None = None,
                   settling_timeout_s: float = 30) -> dict[str, Any]:
    """Collect real version-2 HP4/RRF sweep records against one persistent native world.

    Config: {fixed:[2,3], drive:0, sensor:1}. Options include sweepPoints, fixedTargets
    (comma-separated mm), feed, forceLow/Mid/Max (N), sensorCollectionForce, noiseSamples,
    returnToOrigin, projectZeroTension and preserveBuildupFactor. Default is six points per direction.
    Returns collector JSON, RRD, scene, command/sensor events and immutable per-collection provenance.
    Subsequent calls keep the world and firmware references; reset_session starts both afresh.
    """
    args = {'options': options or {}, 'settling_timeout_s': settling_timeout_s}
    if configs is not None:
        args['configs'] = configs
    return supervised_request('collect', **args)


@mcp.tool(annotations=MUTATION)
def reset_session() -> dict[str, Any]:
    """Archive the recording and restart the native world, RRF and bridge references together."""
    return supervised_request('reset')


@mcp.tool(annotations=MUTATION)
def step_physics(steps: int = 1) -> dict[str, Any]:
    """Advance 1–10,000 fixed physics steps in the continuing native session."""
    return supervised_request('step', steps=steps)


@mcp.tool(annotations=MUTATION)
def start_browser_service() -> dict[str, Any]:
    """Start launcher-owned Vite outside the agent sandbox and return its ready 3D page URL.

    This browser scene is independent of the native world. Inspect native RRDs with Rerun.
    """
    return supervised_request('browser')


@mcp.tool(annotations=MUTATION)
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

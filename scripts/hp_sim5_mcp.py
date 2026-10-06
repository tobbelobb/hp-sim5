"""Native simulation MCP; stdout is reserved for the stdio protocol."""
from pathlib import Path
import os
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
    'run_experiment starts fresh; collection jobs and G-code use the launcher-owned persistent session. '
    'Experiments write immutable inputs, JSON telemetry and Rerun recordings. '
    'Use Rerun MCP for Viewer inspection. Motor angles are radians, geometry metres, forces newtons. '
    + (ROOT / 'research/AGENTS.md').read_text()
    + f'\nSession artifacts: {os.environ.get("HP_SIM5_SESSION_DIR", ROOT / "output/research")}. Maintain research.md and report.md there.'
))
READ_ONLY = ToolAnnotations(read_only_hint=True, open_world_hint=False)
MUTATION = ToolAnnotations(read_only_hint=False, destructive_hint=False, open_world_hint=False)


@mcp.tool(annotations=READ_ONLY)
def capabilities() -> dict[str, Any]:
    """Discover native scenes, command units, artifact contracts and limits."""
    return {'backends': {'native-python': 'Fresh trials and continuing HP4/RRF collection',
                         'browser-js': 'The exact open 3D page via browser_status/browser_action',
                         'standalone-js': 'Production JS parity harness in tests/parity3d'},
            'session_artifacts': os.environ.get('HP_SIM5_SESSION_DIR'), 'scenes': [str(path.relative_to(ROOT)) for path in sorted((ROOT / 'public/usd_scenes').glob('*rigid_body.usda'))],
            'default_scene': DEFAULT_SCENE, 'max_steps': MAX_STEPS,
            'commands': {'Move': 'absolute motor angles in rad; axes object or top-level axis keys',
                         'Add to reference': 'reference angle increments in rad',
                         'SetTorqueMode': 'axis and torqueNm', 'SetPositionMode': 'axis',
                         '{}': 'hold current targets for one fixed timestep', 'E': 'deposited length in m'},
            'telemetry': 'Every step including zero: effectors, encoders, tracking errors, missed steps, lengths and forces',
            'native_collection': {'machine': 'HP4', 'firmware': 'RRF',
                                  'operations': ['start_collection', 'collection_status', 'cancel_collection', 'runtime_status', 'send_gcode', 'step_physics', 'reset_session', 'start_browser_service'],
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
def start_collection(configs: list[dict] | None = None, options: dict | None = None,
                     settling_timeout_s: float = 30) -> dict[str, Any]:
    """Start a long HP4/RRF collection job; poll collection_status and explicitly cancel_collection to stop movement.

    Config: {fixed:[2,3], drive:0, sensor:1}. Options: sweepPoints (3–100), fixedTargets
    (comma-separated mm), feed, forceLow/Mid/Max (N), sensorCollectionForce, noiseSamples,
    returnToOrigin, projectZeroTension, preserveBuildupFactor. Defaults: six points in each direction.
    Completed jobs return production JSON, RRD, commands/sensor events and provenance.
    """
    args = {'options': options or {}, 'settling_timeout_s': settling_timeout_s}
    if configs is not None:
        args['configs'] = configs
    return supervised_request('start_collection', **args)


@mcp.tool(annotations=READ_ONLY)
def collection_status(job_id: str) -> dict[str, Any]:
    """Read collection progress or its final manifest, including cancellation evidence."""
    return supervised_request('job_status', job_id=job_id)


@mcp.tool(annotations=MUTATION)
def cancel_collection(job_id: str) -> dict[str, Any]:
    """Stop at the next native step boundary, retire firmware queues, finalize partial evidence; requires reset."""
    return supervised_request('cancel_job', job_id=job_id)


@mcp.tool(annotations=MUTATION)
def reset_session() -> dict[str, Any]:
    """Archive the recording and restart the native world, RRF and bridge references together."""
    return supervised_request('reset')


@mcp.tool(annotations=MUTATION)
def step_physics(steps: int = 1) -> dict[str, Any]:
    """Advance 1–10,000 fixed physics steps in the continuing native session."""
    return supervised_request('step', steps=steps)


@mcp.tool(annotations=MUTATION)
def start_browser_service(record: bool = False) -> dict[str, Any]:
    """Start launcher-owned Vite outside the agent sandbox and return its ready 3D page URL.

    This browser scene is independent of the native world. Inspect native RRDs with Rerun.
    """
    return supervised_request('browser', record=record)


@mcp.tool(annotations=MUTATION)
def capture_native_context(message: str, step: int | None = None, selected_entity: str | None = None,
                           run_id: str | None = None) -> dict[str, Any]:
    """Freeze a research message with native run/session, selected sim_step, entity and actual numerical observation.

    For a saved fresh experiment supply run_id and step. Otherwise select an observed step in the continuing session.
    Capture this when accepting visual steering, before further movement or recording navigation.
    """
    if not message.strip():
        raise ValueError('Supply a research message')
    if run_id is None:
        args = {'message': message, 'selected_entity': selected_entity}
        if step is not None:
            args['step'] = step
        return supervised_request('capture_context', **args)
    manifest = read_run(ROOT, run_id)
    selected_step = manifest['steps'] if step is None else step
    if type(selected_step) is not int or not 0 <= selected_step <= manifest['steps']:
        raise ValueError('Select a step in this experiment')
    import json
    import os
    import uuid
    context = {'capture_id': uuid.uuid4().hex, 'backend': 'native-python', 'run_id': run_id,
               'sim_step': selected_step, 'sim_time_s': selected_step * manifest['dt_s'],
               'timeline': 'sim_step', 'recording': manifest['artifacts'].get('rrd'),
               'scene_generation': manifest.get('scene_generation'), 'scene_sha256': manifest['scene_sha256'],
               'selected_entity': selected_entity, 'message': message,
               'observation': read_run(ROOT, run_id, selected_step, 1)['samples'][0]}
    directory = Path(os.environ.get('HP_SIM5_SESSION_DIR', ROOT / 'output/research'))
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / 'native-context.jsonl').open('a') as output:
        output.write(json.dumps(context, allow_nan=False) + '\n')
    return context


@mcp.tool(annotations=READ_ONLY)
def browser_status() -> dict[str, Any]:
    """Discover the exact connected browser page identity and immutable user-submitted research contexts."""
    return supervised_request('browser_status')


@mcp.tool(annotations=MUTATION)
def browser_action(page_id: str, action: str, args: dict | None = None) -> dict[str, Any]:
    """Operate the selected existing browser page: observe, pause, resume, reset, step, commands, load_scene, record, capture_context.

    Obtain page_id from browser_status. This is the browser JS world, independent of native Python.
    Bounded steps require a paused world with no active worker. record requires start_browser_service(record=True).
    capture_context freezes message, selected_entity, backend, page, scene generation, time and numerical observation.
    """
    return supervised_request('browser_action', page_id=page_id, action=action, args=args or {})


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

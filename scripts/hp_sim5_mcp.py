"""Native simulation MCP; stdout is reserved for the stdio protocol."""
from pathlib import Path
import os
import sys
from typing import Annotated, Any, Literal

from pydantic import BaseModel, ConfigDict, Field

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/python'))

from mcp.server import MCPServer
from mcp_types import ToolAnnotations
from hp_sim5_research.experiments import DEFAULT_SCENE, MAX_STEPS, compare_runs, read_run, run_experiment as run
from hp_sim5_research.services import supervised_request, supervised_preflight
from hp_sim5_research.test_jobs import MAX_TIMEOUT_S, SUITES

mcp = MCPServer('hp-sim5', instructions=(
    'Run bounded native Hangprinter experiments, read numeric telemetry, and compare trials. '
    'run_experiment starts fresh; collection jobs and G-code use the launcher-owned persistent session. '
    'Experiments write immutable inputs, JSON telemetry and Rerun recordings. '
    'Use Rerun MCP for Viewer inspection. Motor angles are radians, geometry metres, forces newtons. '
    + (ROOT / 'research/AGENTS.md').read_text()
    + f'\nSession artifacts: {os.environ.get("HP_SIM5_SESSION_DIR", ROOT / "output/research")}. Maintain research.md, report.md and abstract.md there.'
))
READ_ONLY = ToolAnnotations(read_only_hint=True, open_world_hint=False)
MUTATION = ToolAnnotations(read_only_hint=False, destructive_hint=False, open_world_hint=False)


@mcp.tool(annotations=READ_ONLY)
def capabilities() -> dict[str, Any]:
    """Discover native scenes, command units, artifact contracts and limits."""
    return {'backends': {'native-python': 'Explicit independent Python reference for fresh trials and continuing collection (slow)',
                         'native-warp': 'Optional compiled double-precision cable solver on CPU; other physics systems remain Python',
                         'native-warp-cuda': 'Experimental CUDA cable solver; requires CUDA device, GPU parity not yet verified',
                         'headless-js': 'Default native Rust physics hosted in Node; no browser/rendering',
                         'browser-js': 'The exact open 3D page via browser_status/browser_action',
                         'standalone-js': 'Production JS parity harness in tests/parity3d'},
            'session_artifacts': os.environ.get('HP_SIM5_SESSION_DIR'), 'scenes': [str(path.relative_to(ROOT)) for path in sorted((ROOT / 'public/usd_scenes').glob('*rigid_body.usda'))],
            'default_scene': DEFAULT_SCENE, 'max_steps': MAX_STEPS,
            'commands': {'Move': 'absolute motor angles in rad; axes object or top-level axis keys',
                         'Add to reference': 'reference angle increments in rad',
                         'SetTorqueMode': 'axis and torqueNm', 'SetPositionMode': 'axis',
                         '{}': 'hold current targets for one fixed timestep', 'E': 'deposited length in m'},
            'telemetry': 'Every step including zero: effectors, encoders, tracking errors, missed steps, lengths and forces',
            'native_collection': {'machines': ['HP3', 'HP4'], 'firmware': 'RRF',
                                  'operations': ['start_collection', 'collection_status', 'cancel_collection', 'runtime_status', 'send_gcode', 'step_physics', 'reset_session', 'configure_session', 'start_browser_service'],
                                  'encoders': 'Raw unwrapped degrees; A/B/C/D mapped to CAN 40/41/42/43',
                                  'clock': 'Persistent fixed-step world; collector delays advance physics; reads drain motion',
                                  'settling': 'Bounded in simulation time; default 30 seconds',
                                  'services': 'Launcher-owned runtime, firmware and bridge outside the command sandbox',
                                  'configuration': {'tool': 'configure_session', 'immutable_per_generation': True,
                                      'scenes': 'Matching stock HP3/HP4; custom anchor designs unsupported',
                                      'radii': 'Physical base radii: one or four values, 1–200 mm; custom radii require layering off',
                                      'layered_buildup': 'Stock 2/pi only; unlayered buildup is zero',
                                      'reset': 'Retains accepted settings; reloads scene, JS and firmware sources',
                                      'active_jobs': 'Explicit cancellation required before reset or reconfiguration'}},
            'full_autocal': {'operations': ['start_autocal', 'autocal_status', 'cancel_autocal'],
                             'machines': ['hp3', 'hp4'], 'backend': 'headless-js', 'isolated_world': True,
                             'workflow': 'Production one-click CLI: physical collection, fitting, planning, selection, automatic stopping',
                             'statuses': ['running', 'cancelling', 'accepted', 'budget_exhausted', 'cancelled', 'failed'],
                             'limits': {'max_steps': 100, 'sweep_points': 100, 'wall_timeout_s': 14400,
                                        'max_physics_steps': 10000000, 'threads': 8},
                             'artifact_dir': 'New directory within launcher session artifacts; defaults to autocal/job_id'},
            'preflight': {'tool': 'research_preflight', 'timeout_s': 15, 'moves_robot': False,
                          'recorder': 'Isolated tool-check recording; handshake, finalization and readable RRD',
                          'command_network': 'Unknown until scripts/research_socket_probe.py runs in agent command context',
                          'doctor': 'Separate, explicit expensive physical diagnostic'},
            'integration_tests': {'operations': ['start_test', 'test_status', 'cancel_test'],
                                  'suites': SUITES, 'max_timeout_s': MAX_TIMEOUT_S,
                                  'slow_tests': 'Excluded; movement/full-auto smoke tests remain explicit',
                                  'services': 'Supervisor-owned sockets, fixed argv, repo cwd and .venv/bin/python',
                                  'outcomes': ['passed', 'failed', 'blocked'],
                                  'artifacts': 'Isolated tests/job_id directory; stdout/stderr, JUnit, pytest temp and manifest'},
            'limitations': ['Klipper streamed motion is not supported by native collection',
                            'Final snapshots are observations, not resumable physics checkpoints',
                            'Timings include telemetry and optional recording overhead']}


@mcp.tool(annotations=READ_ONLY)
def runtime_status() -> dict[str, Any]:
    """Inspect persistent native world, clock, command queue and supervised service health."""
    return supervised_request('status')


@mcp.tool(annotations=MUTATION)
def research_preflight(expected_configuration: dict | None = None) -> dict[str, Any]:
    """Run a bounded non-moving tool check before long collection, independently of expensive doctor.

    Checks supervised networking, matched world settings, firmware/collector, isolated recorder
    acknowledgment/finalization and available full-auto/configuration/test tools. Optional expected_configuration
    is a complete configure_session request. Never resets or advances the attached world.
    Command networking is reported unknown: separately run .venv/bin/python scripts/research_socket_probe.py
    in the agent command context. A successful supervisor check does not prove command socket access.
    """
    return supervised_preflight(expected_configuration)


@mcp.tool(annotations=MUTATION)
def start_test(suite: Literal['headless-autocal', 'research-services'], timeout_s: int = 300) -> dict[str, Any]:
    """Start a fixed integration suite on the supervisor; return a job ID immediately.

    Uses repository .venv/bin/python, isolated artifacts, and excludes slow physical/full-auto smoke tests.
    timeout_s is bounded to 1–900. No shell or additional argv is accepted. Poll test_status;
    cancel_test retires the entire job process group and preserves failed/partial evidence.
    Existing collection/autocal jobs and the attached world remain live.
    """
    return supervised_request('start_test', suite=suite, timeout_s=timeout_s)


@mcp.tool(annotations=READ_ONLY)
def test_status(job_id: str) -> dict[str, Any]:
    """Read running/cancelling or passed/failed/blocked/cancelled test state, exit code, logs and artifacts."""
    return supervised_request('test_status', job_id=job_id)


@mcp.tool(annotations=MUTATION)
def cancel_test(job_id: str) -> dict[str, Any]:
    """Stop a supervised integration suite and descendants; preserve isolated logs and artifacts."""
    return supervised_request('cancel_test', job_id=job_id)


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
    """Start an HP3/HP4 RRF collection job; poll collection_status and explicitly cancel_collection to stop movement.

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
def start_autocal(machine: str = 'hp3', layering: bool = False,
                  base_radii_mm: list[float] | None = None, buildup_factor: float = 0,
                  find_radii: str = 'off', find_buildup_factor: str = 'off',
                  optimizer: dict | None = None, budgets: dict | None = None,
                  collection: dict | None = None,
                  artifact_dir: str | None = None) -> dict[str, Any]:
    """Return a full-autocal job ID immediately; the supervisor runs an isolated production CLI world.

    base_radii_mm: one or four known/initial radii (default [30]); fitting modes: off/global/per-anchor.
    Nonzero or fitted buildup requires layering. optimizer: mode (fast/fast-fd/legacy), solver
    (lbfgsb/trf), solve_iterations, solve_restarts, spool_outer_iters, spool_inner_iters,
    r0_bounds_mm and b_bounds (two-element lists). budgets: max_steps (default 20),
    sweep_points (20), wall_timeout_s (3600), max_physics_steps (5000000), threads (1).
    collection optionally supplies force_low_n, force_mid_n, force_max_n together (0.001–10 N,
    low < mid <= max) to use the CLI's manual forces, and/or sensor_collection_force_n.
    Omitted forces preserve production autotuning.
    artifact_dir must be new and inside session artifacts. Poll autocal_status; cancel_autocal stops
    physical activity and finalizes partial evidence. Budget exhaustion is not automatic acceptance.
    """
    args = dict(machine=machine, layering=layering, base_radii_mm=base_radii_mm if base_radii_mm is not None else [30],
                buildup_factor=buildup_factor, find_radii=find_radii, find_buildup_factor=find_buildup_factor,
                optimizer=optimizer or {}, budgets=budgets or {}, collection=collection or {})
    if artifact_dir is not None:
        args['artifact_dir'] = artifact_dir
    return supervised_request('start_autocal', **args)


@mcp.tool(annotations=READ_ONLY)
def autocal_status(job_id: str) -> dict[str, Any]:
    """Read full-autocal progress, terminal reason, CLI exit status, settings and complete/partial artifacts."""
    return supervised_request('autocal_status', job_id=job_id)


@mcp.tool(annotations=MUTATION)
def cancel_autocal(job_id: str) -> dict[str, Any]:
    """Stop an isolated autocal world, retire its child process group and finalize partial evidence."""
    return supervised_request('cancel_autocal', job_id=job_id)


class WorldSettings(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    machine: Literal['hp3', 'hp4'] = 'hp4'
    layering: bool = True
    scene: str | None = None
    firmware_config: str | None = None
    base_radii_mm: list[Annotated[float, Field(ge=1, le=200)]] | None = None
    buildup_factor: Annotated[float, Field(ge=0, le=2)] | None = None


@mcp.tool(annotations=MUTATION)
def configure_session(configuration: WorldSettings) -> dict[str, Any]:
    """Archive the previous world and create a fresh configured world, RRF and encoder origin.

    Supply machine hp3/hp4 and layering. Optional scene and firmware_config must select matching
    stock repository files (firmware path relative to RRF/run/vsd). Unlayered worlds accept one
    or four physical base_radii_mm (1–200), with zero buildup. Layered worlds support stock
    30 mm base radii and buildup_factor=2/pi only. Custom anchor layouts are unsupported.
    Settings are immutable until the next configure_session; ordinary reset retains them.
    Explicitly cancel active collection/autocal jobs first. Invalid settings leave the live world
    intact; startup failure preserves evidence and reports reset_required with recovery instructions.
    """
    return supervised_request('configure', **configuration.model_dump(exclude_none=True))


@mcp.tool(annotations=MUTATION)
def reset_session() -> dict[str, Any]:
    """Archive evidence and restart world, RRF and encoder references with the accepted settings.

    Reloads scene/JS/firmware sources. Active jobs require explicit cancellation first."""
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
    context = {'capture_id': uuid.uuid4().hex, 'backend': manifest['backend'],
               'physics_engine': manifest.get('physics_engine', manifest['backend']), 'run_id': run_id,
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
                   label: str = '', record: bool = True, cable_solver_device: str | None = None,
                   physics_backend: str | None = None) -> dict[str, Any]:
    """Run fresh native physics. One command per step; return measurements and saved artifact paths.

    Use commands_file for large JSON arrays inside the repo. Do not supply both forms.
    {} holds targets. RRD opens in Rerun; read_run gives numeric data without a Viewer.
    Defaults to native Rust. physics_backend='native-python' selects the Python reference.
    cable_solver_device='cpu' selects Warp; 'cuda:0' is experimental (remaining systems use Python).
    """
    if commands_file is not None:
        if commands is not None:
            raise ValueError('Supply commands or commands_file, not both')
        from hp_sim5_research.experiments import repo_path
        import json
        commands = json.loads(repo_path(ROOT, commands_file).read_text())
    return run(ROOT, scene, steps=steps, dt=dt, commands=commands, label=label, record=record,
               cable_solver_device=cable_solver_device, physics_backend=physics_backend)


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

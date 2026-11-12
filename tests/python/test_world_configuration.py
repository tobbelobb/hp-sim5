import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess

import pytest

from hp_sim5_research.configuration import prepare_world, world_configuration
from hp_sim5_research.services import RuntimeService
from hp_sim5_research.session import NativeSession
from hp_sim5_research.mirror import CablePathComponent, RadiusComponent

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('args', [
    {}, {'machine': 'hp5'}, {'machine': 'slideprinter'}, {'machine': 'cubecorners'},
    {'machine': 'hp3', 'layering': False},
    {'machine': 'hp3', 'scene': 'public/usd_scenes/hp4.usda'},
    {'machine': 'hp3', 'firmware_config': 'sys/config_hp4.g'},
    {'machine': 'hp3', 'base_radii_mm': [True]},
    {'machine': 'hp3', 'base_radii_mm': [float('nan')]}, {'machine': 'hp3', 'base_radii_mm': [0]},
    {'machine': 'hp3', 'base_radii_mm': [30, 30]}, {'machine': 'hp3', 'buildup_factor': .6},
    {'machine': 'hp3_w_line_layers', 'buildup_factor': .6}, {'machine': 'hp3_w_line_layers', 'base_radii_mm': [31]},
    {'machine': 'hp3', 'backend': 'other'}, {'machine': 'hp3', 'anchors': []},
])
def test_invalid_world_settings(args):
    with pytest.raises(ValueError):
        world_configuration(ROOT, args, backend='headless-js')


@pytest.mark.parametrize('nickname,machine_type,scene,firmware,layering', [
    ('hp3', 'hangprinter_3', 'hp3', 'hp3', False),
    ('HP4_w_line_layers', 'hangprinter_4', 'hp4', 'hp4_w_line_layers', True),
    ('skycam', 'skycam', 'four_high_anchors', 'skycam', False),
])
def test_catalogued_machine_selects_scene_firmware_and_layering(nickname, machine_type, scene, firmware, layering):
    config = world_configuration(ROOT, {'machine': nickname}, backend='headless-js')
    assert (config.machine_type, config.layering) == (machine_type, layering)
    assert (config.scene, config.firmware_config) == (f'public/usd_scenes/{scene}.usda', f'sys/config_{firmware}.g')


def test_mcp_defers_autocal_defaults_to_the_selected_machine(monkeypatch):
    from scripts import hp_sim5_mcp as mcp

    monkeypatch.setattr(mcp, 'supervised_request', lambda operation, **args: args)
    assert 'machine' not in mcp.start_autocal()
    assert 'buildup_factor' not in mcp.start_autocal(machine='hp3_w_line_layers')
    assert mcp.start_autocal(machine='hp3_w_line_layers', buildup_factor=0)['buildup_factor'] == 0


@pytest.mark.slow
@pytest.mark.asyncio
@pytest.mark.parametrize('machine', ['hp3', 'hp4'])
async def test_disabled_layering_bakes_zero_width_and_matches_production(tmp_path, machine):
    config = world_configuration(ROOT, {'machine': machine}, backend='headless-js')
    metadata = prepare_world(ROOT, tmp_path, config)
    # Compare byte-for-byte with the production construction path used by headless autocal.
    script = """
import fs from 'node:fs';
import {createHeadlessWorld} from './hp-sim-3d/app/headlessWorld.js';
const {bakedScene} = createHeadlessWorld(fs.readFileSync(process.argv[1], 'utf8'), {lineLayeringEnabled:false});
process.stdout.write(bakedScene);
"""
    production = subprocess.check_output(['node', '--input-type=module', '-e', script,
                                         str(ROOT / config.scene)], cwd=ROOT, text=True)
    assert hashlib.sha256(production.encode()).hexdigest() == metadata['baked_scene_sha256']
    session = NativeSession(ROOT, tmp_path, backend='headless-js', record=False,
                            frozen_scene=production, layering=False)
    try:
        widths = [session.world.get_component(entity, CablePathComponent).cable_half_width
                  for entity in session.world.query([CablePathComponent])]
        assert widths and set(widths) == {0.}
        assert all(session.world.get_resource(key) is False for key in
                   ('enableLayering', 'layeringFrictionEffectiveRadius', 'layeringRenderWraps'))
        await session.handle({'commands': [{'type': 'Move', 'A': -.001}] * 10})
        await session.handle({'type': 'encoder_request', 'requestId': 1, 'axes': list('ABCD')})
        assert session.step == 10
        assert abs(session.encoder_angles('A')[0]) > .001
    finally:
        session.close()


@pytest.mark.slow
def test_custom_unlayered_radii_reach_geometry_and_frozen_firmware(tmp_path):
    config = world_configuration(ROOT, {'machine': 'hp3',
                                'base_radii_mm': [31, 32, 33, 34]}, backend='headless-js')
    metadata = prepare_world(ROOT, tmp_path, config)
    session = NativeSession(ROOT, tmp_path, record=False,
                            frozen_scene=(tmp_path / 'scene.usda').read_text(), layering=False)
    try:
        assert [session.world.get_component(session.remote.axis_to_entity[a][0], RadiusComponent).radius
                for a in 'ABCD'] == pytest.approx([.031, .032, .033, .034])
        assert 'M666 Q0 R31.0:32.0:33.0:34.0' in (tmp_path / 'firmware-config.g').read_text()
        assert metadata['firmware_configuration']['radii_mm'] == [31, 32, 33, 34]
    finally:
        session.close()


@pytest.mark.slow
def test_preparation_reloads_authored_sources_and_rejects_changed_firmware(tmp_path):
    root = tmp_path / 'repo'
    for source in ('public/usd_scenes/hp3.usda', 'RRF/run/vsd/sys/config_hp3.g'):
        target = root / source
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / source, target)
    (root / 'scripts').symlink_to(ROOT / 'scripts', target_is_directory=True)
    config = world_configuration(root, {'machine': 'hp3'}, backend='headless-js')
    first, second, failed = [tmp_path / name for name in ('first', 'second', 'failed')]
    for directory in (first, second, failed):
        directory.mkdir()
    before = prepare_world(root, first, config)
    scene = root / config.scene
    scene.write_text(scene.read_text().replace('cablePath:stored = [1.0', 'cablePath:stored = [1.1', 1))
    after = prepare_world(root, second, config)
    assert before['authored_scene_sha256'] != after['authored_scene_sha256']
    assert before['baked_scene_sha256'] != after['baked_scene_sha256']
    firmware = root / 'RRF/run/vsd/sys/config_hp3.g'
    firmware.write_text(firmware.read_text().replace('M666 Q0.0', 'M666 Q1.0'))
    with pytest.raises(ValueError, match='incompatible Q/R'):
        prepare_world(root, failed, config)
    assert (first / 'scene.usda').exists()


@pytest.mark.slow
def test_real_reconfiguration_reset_active_job_and_evidence(tmp_path):
    service = RuntimeService(ROOT, tmp_path, record=False).start()
    try:
        empty = service.call('status')
        assert empty['machine_design'] is None and empty['accepted_settings'] is None and empty['reset_required']
        for operation in ('reset', 'gcode'):
            with pytest.raises(RuntimeError, match='configure_session'):
                service.call(operation, line='M114')
        with pytest.raises(RuntimeError, match='machine'):
            service.call('configure')
        hp4 = service.call('configure', machine='hp4_w_line_layers')
        with pytest.raises(RuntimeError, match='machine'):
            service.call('configure', machine='invalid')
        assert service.call('status')['world_id'] == hp4['world_id']
        # Collector start-up replies may still be appending; the world itself must not restart.
        events = Path(hp4['artifacts']['events']).read_text().splitlines()
        assert sum(json.loads(line)['type'] == 'session_started' for line in events) == 1
        hp3 = service.call('configure', machine='hp3', base_radii_mm=[30], buildup_factor=0)
        assert hp3['world_id'] != hp4['world_id'] and hp3['generation'] == hp4['generation'] + 1
        assert hp3['machine_design'] == 'Hangprinter version 3 without line layering' and hp3['step'] == 0
        assert hp3['session_directory'] == hp4['session_directory'] == str(tmp_path)
        assert hp3['world_configuration']['effective_settings']['layering'] is False
        assert json.loads(Path(hp4['artifacts']['world_manifest']).read_text())['status'] == 'archived'
        assert hp3['services']['rrf']['pid'] != hp4['services']['rrf']['pid']
        service.call('gcode', line='G1 H2 X1 F600')
        service.call('gcode', line='M569.3 P40.0:41.0:42.0:43.0')
        reset = service.call('reset')
        assert reset['step'] == 0 and reset['world_id'] != hp3['world_id']
        assert reset['accepted_settings'] == hp3['accepted_settings']
        service.call('gcode', line='M569.3 P40.0:41.0:42.0:43.0')
        events = [json.loads(line) for line in Path(reset['artifacts']['events']).read_text().splitlines()]
        assert next(row for row in reversed(events) if row['type'] == 'encoder_response' and row['axes'])['angles_deg'] == [0., 0., 0., 0.]
        job = service.call('start_collection', options={'sweepPoints': 3})
        for operation in ('configure', 'reset'):
            with pytest.raises(RuntimeError, match='explicitly cancel'):
                service.call(operation, machine='hp4')
        assert service.call('status')['world_id'] == reset['world_id']
        service.call('cancel_job', job_id=job['job_id'])
        assert service.call('reset')['accepted_settings'] == hp3['accepted_settings']
        collected = service.call('collect', options={'sweepPoints': 3, 'noiseSamples': 4})
        assert collected['status'] == 'complete' and collected['point_count'] == 6
        assert collected['machine_design'] == 'Hangprinter version 3 without line layering'
        dataset = json.loads(Path(collected['artifacts']['dataset']).read_text())
        assert dataset['machine_type'] == 'hangprinter_3'
        inherited = service.call('configure', base_radii_mm=[31])
        assert inherited['machine_design'] == 'Hangprinter version 3 without line layering'
        assert collected['world_configuration']['geometry']['cable_half_widths_m'] == [0.]
        assert collected['validation']['drive_span_mm'][0] > 1
        assert all(span > 1 for span in collected['validation']['physical_sensor_span_mm'])
    finally:
        service.close()


@pytest.mark.slow
@pytest.mark.asyncio
async def test_active_autocal_and_failed_restart_remain_recoverable(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location('research_runtime', ROOT / 'scripts/research_runtime.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv('HP_SIM5_RUNTIME_TOKEN', 'test-token')
    monkeypatch.setenv('HP_SIM5_RECORD', '0')
    runtime = module.Runtime('unused', tmp_path)
    await runtime.start(world_configuration(ROOT, {'machine': 'hp4_w_line_layers'}, backend='headless-js'))
    original = runtime.configuration
    old_manifest = runtime.session_dir / 'manifest.json'
    real_spawn = runtime.spawn
    real_session = module.NativeSession
    try:
        before = old_manifest.read_bytes()
        def unavailable_backend(*args, **kwargs):
            raise ValueError('Backend unavailable')
        monkeypatch.setattr(module, 'NativeSession', unavailable_backend)
        with pytest.raises(RuntimeError, match='Backend unavailable'):
            await runtime.dispatch('configure', {'machine': 'hp3'})
        assert runtime.session is not None and runtime.session.connected.is_set()
        assert runtime.configuration == original and old_manifest.read_bytes() == before
        assert runtime.status()['last_world_attempt']['status'] == 'failed'
        monkeypatch.setattr(module, 'NativeSession', real_session)
        runtime.autocal.active = 'running'
        with pytest.raises(ValueError, match='explicitly cancel'):
            await runtime.dispatch('configure', {'machine': 'hp3'})
        runtime.autocal.active = None
        def fail_spawn(*args):
            raise OSError('Deliberate firmware startup failure')
        monkeypatch.setattr(runtime, 'spawn', fail_spawn)
        with pytest.raises(RuntimeError, match='Deliberate firmware'):
            await runtime.dispatch('configure', {'machine': 'hp3'})
        state = runtime.status()
        assert state['reset_required'] and not state['connected']
        assert runtime.configuration == original
        assert json.loads(old_manifest.read_text())['status'] == 'archived'
        failed = json.loads(Path(state['artifacts']['world_manifest']).read_text())
        assert failed['status'] == 'failed' and failed['effective_settings']['machine'] == 'Hangprinter version 3 without line layering'
        monkeypatch.setattr(runtime, 'spawn', real_spawn)
        await runtime.dispatch('reset', {})
        assert runtime.status()['machine_design'] == 'Hangprinter version 4 with line layering'
        assert not runtime.status()['reset_required']
    finally:
        await runtime.close_session()


@pytest.mark.slow
@pytest.mark.asyncio
async def test_reset_reloads_scene_and_js_sources_with_fixed_settings(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location('research_runtime', ROOT / 'scripts/research_runtime.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv('HP_SIM5_RUNTIME_TOKEN', 'test-token')
    monkeypatch.setenv('HP_SIM5_RECORD', '0')
    runtime = module.Runtime('unused', tmp_path / 'artifacts')
    runtime.configuration = world_configuration(ROOT, {'machine': 'hp3'}, backend='headless-js')
    root = tmp_path / 'repo'
    root.mkdir()
    for name in ('src', 'hp-sim-3d', 'integrations', 'autocal'):
        (root / name).symlink_to(ROOT / name, target_is_directory=True)
    (root / 'scripts').mkdir()
    for source in (ROOT / 'scripts').iterdir():
        target = root / 'scripts' / source.name
        if source.name in ('research_physics.mjs', 'train_native_physics.mjs'):
            shutil.copyfile(source, target)
        else:
            target.symlink_to(source, target_is_directory=source.is_dir())
    (root / 'RRF').mkdir()
    (root / 'RRF/build').symlink_to(ROOT / 'RRF/build', target_is_directory=True)
    shutil.copytree(ROOT / 'RRF/run/vsd/sys', root / 'RRF/run/vsd/sys')
    scene = root / runtime.configuration.scene
    scene.parent.mkdir(parents=True)
    shutil.copyfile(ROOT / runtime.configuration.scene, scene)
    native = root / 'physics.node'
    shutil.copyfile(ROOT / 'src/wasm/cable_solver/target/native/physics.node', native)
    monkeypatch.setenv('HP_SIM5_NATIVE_ENGINE', str(native))
    monkeypatch.setattr(module, 'ROOT', root)
    try:
        await runtime.start(runtime.configuration)
        before = runtime.status()
        frozen = Path(before['artifacts']['scene']).read_bytes()
        scene.write_text(scene.read_text().replace('cablePath:stored = [1.0', 'cablePath:stored = [1.1', 1))
        worker = root / 'scripts/research_physics.mjs'
        worker.write_text("console.error('source-reload-marker');\n" + worker.read_text())
        stale = runtime.status()['source_changed']
        assert stale['scene_requires_session_reset'] and stale['bridge_requires_session_reset']
        trainer = root / 'scripts/train_native_physics.mjs'
        trainer.write_text('// changed training recipe\n' + trainer.read_text())
        assert runtime.status()['source_changed']['rust_physics_requires_session_reset']
        native.write_bytes(native.read_bytes() + b'\x00')  # ELF/Mach-O trailing data changes artifact identity.
        assert runtime.status()['source_changed']['rust_physics_requires_session_reset']
        await runtime.dispatch('reset', {})
        after = runtime.status()
        assert before['world_id'] != after['world_id']
        assert after['accepted_settings'] == before['accepted_settings']
        assert after['world_configuration']['baked_scene_sha256'] != before['world_configuration']['baked_scene_sha256']
        assert Path(before['artifacts']['scene']).read_bytes() == frozen
        assert 'source-reload-marker' in Path(after['services']['physics-js']['log']).read_text()
        assert not after['source_changed']['scene_requires_session_reset']
        assert not after['source_changed']['bridge_requires_session_reset']
        assert not after['source_changed']['rust_physics_requires_session_reset']
        assert after['world_configuration']['rust_physics_sha256'] != before['world_configuration']['rust_physics_sha256']
    finally:
        await runtime.close_session()

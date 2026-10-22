"""Matched longer NativeSession trials; no firmware planner or Rerun recording."""
import asyncio
import hashlib
import json
import math
import platform
import statistics
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / 'src/python'))
from hp_sim5_research.session import NativeSession
from cable_joints_3d.pbd_cable_constraint_solver import PBDCableConstraintSolver

OUT = Path(__file__).parent
BACKENDS = ['native-warp', 'native-warp-cuda', 'headless-js']
STEPS = 10000
DT = .002


def gpu_status():
    result = subprocess.run(['nvidia-smi', '--query-gpu=name,utilization.gpu,memory.used,temperature.gpu',
                             '--format=csv,noheader'], text=True, capture_output=True)
    return {'returncode': result.returncode, 'output': result.stdout.strip(), 'error': result.stderr.strip()}


def observations(path):
    return {row['step']: row['sample'] for row in map(json.loads, path.read_text().splitlines())
            if row['type'] == 'observation'}


def compare(first, second):
    # Exact sample and identity coverage; never silently drop mismatched arrays.
    assert first.keys() == second.keys(), 'Observation steps differ'
    maxima = {'raw_encoder_deg': 0., 'effector_position_mm': 0.,
              'cable_length_mm': 0., 'segment_force_n': 0.}
    for step in first:
        a, b = first[step], second[step]
        for field, key in [('motors', 'axis'), ('effectors', 'path'), ('cables', 'name')]:
            assert len(a[field]) == len(b[field])
            assert [v[key] for v in a[field]] == [v[key] for v in b[field]], (step, field)
        for x, y in zip(a['motors'], b['motors']):
            assert x['mode'] == y['mode'], (step, 'mode')
            maxima['raw_encoder_deg'] = max(maxima['raw_encoder_deg'],
                                            abs(x['encoder_angle_rad']-y['encoder_angle_rad'])*180/math.pi)
        for x, y in zip(a['effectors'], b['effectors']):
            maxima['effector_position_mm'] = max(maxima['effector_position_mm'],
                                                 math.dist(x['position'], y['position'])*1000)
        for x, y in zip(a['cables'], b['cables']):
            assert x['lengths_m'].keys() == y['lengths_m'].keys()
            for key, value in x['lengths_m'].items():
                expected = y['lengths_m'][key]
                assert (value is None) == (expected is None)
                if value is not None:
                    maxima['cable_length_mm'] = max(maxima['cable_length_mm'], abs(value-expected)*1000)
            assert len(x['segment_forces_n']) == len(y['segment_forces_n'])
            maxima['segment_force_n'] = max(maxima['segment_force_n'],
                max((abs(u-v) for u,v in zip(x['segment_forces_n'],y['segment_forces_n'])), default=0.))
    maxima['observation_count'] = len(first)
    maxima['encoder_within_0_01_deg'] = maxima['raw_encoder_deg'] <= .01
    return maxima


async def main():
    result = {'kind': 'controlled_long_native_session_comparison', 'steps': STEPS,
              'dt_s': DT, 'simulated_s': STEPS*DT, 'record': False,
              'method': 'Same frozen scene and commands, 10 Hz numerical observations; no new firmware collection',
              'python': platform.python_version(), 'revision': subprocess.check_output(
                  ['git','rev-parse','HEAD'], cwd=ROOT, text=True).strip(), 'trials': [], 'comparisons': []}
    import warp as wp
    result['warp_version'] = wp.__version__
    result['warp_source_sha256'] = hashlib.sha256((ROOT/'src/python/cable_joints_3d/warp_solver.py').read_bytes()).hexdigest()
    (OUT/'solver-source.py').write_bytes((ROOT/'src/python/cable_joints_3d/warp_solver.py').read_bytes())
    commands = [{'type':'Move','A': -.002*i} for i in range(500)]
    commands += [{'type':'SetTorqueMode','axis':'B','torqueNm': -.003}] + [{}]*(STEPS-501)
    command_bytes = json.dumps(commands, sort_keys=True).encode()
    result['commands_sha256'] = hashlib.sha256(command_bytes).hexdigest()
    (OUT/'commands.json').write_bytes(command_bytes)
    for machine, repeats in [('hp3',3), ('hp4',1)]:
        for repeat in range(repeats):
            # Rotate order to reduce systematic effects of warming/desktop activity.
            order = BACKENDS[repeat % 3:] + BACKENDS[:repeat % 3]
            for backend in order:
                directory = OUT/f'{machine}-{backend}-{repeat}'
                if directory.exists():
                    raise ValueError(f'Retain previous artifacts; use a fresh directory: {directory}')
                started = time.perf_counter()
                session = NativeSession(ROOT,directory,scene=f'public/usd_scenes/{machine}_rigid_body.usda',
                                        backend=backend,record=False)
                construction = time.perf_counter()-started
                # Warm a disposable step, then reset to the original input state.
                started = time.perf_counter()
                await session.advance(DT)
                warmup = time.perf_counter()-started
                await session.handle({'type':'reset'})
                await session.handle({'commands':commands})
                session.physics_wall_s = session.worker_wall_s = session.observation_wall_s = 0.
                solver = session.world.get_system(PBDCableConstraintSolver)
                solver_time = [0.]
                if backend != 'headless-js':
                    update = solver.update
                    def timed(world, dt):
                        t = time.perf_counter(); update(world, dt)
                        solver_time[0] += time.perf_counter()-t
                    solver.update = timed
                row = {'machine':machine,'repeat':repeat,'backend':backend,'construction_wall_s':construction,
                       'warmup_wall_s':warmup,'scene_sha256':hashlib.sha256(session.frozen_scene.encode()).hexdigest(),
                       'gpu_before':gpu_status()}
                started = time.perf_counter()
                session.deadline = time.monotonic()+300
                try:
                    for second in range(20):
                        await session.advance(1.)
                        print(json.dumps({'progress':f'{machine}/{backend}/{repeat}',
                                          'simulated_s':second+1,'wall_s':time.perf_counter()-started}),flush=True)
                    assert session.step == STEPS and session.queue_length() == 0
                    session.observe()
                    row.update(status='complete',steps=session.step,**{
                        key:session.status()[key] for key in ['physics_wall_s','worker_wall_s','observation_wall_s']})
                    row['solver_adapter_wall_s'] = solver_time[0] if backend != 'headless-js' else None
                except Exception as error:
                    row.update(status='failed',error=repr(error),steps=session.step)
                    raise
                finally:
                    session.close()
                    row['wall_s'] = time.perf_counter()-started
                    row['realtime_factor'] = row.get('steps',0)*DT/row['wall_s']
                    row['gpu_after'] = gpu_status()
                    result['trials'].append(row)
                    (directory/'timing.json').write_text(json.dumps(row,indent=2))
                    (OUT/'results.json').write_text(json.dumps(result,indent=2))
                print(json.dumps({'trial':row}),flush=True)
            # Warmup has epoch 0 step 1; exclude it, preserving all epoch 1 samples.
            peers = {b:{row['step']:row['sample'] for row in map(json.loads,(OUT/f'{machine}-{b}-{repeat}'/'events.jsonl').read_text().splitlines())
                        if row['type']=='observation' and row['epoch']==1} for b in BACKENDS}
            for first,second in [('native-warp-cuda','native-warp'),('native-warp-cuda','headless-js'),('native-warp','headless-js')]:
                result['comparisons'].append({'machine':machine,'repeat':repeat,'first':first,'second':second,
                                               **compare(peers[first],peers[second])})
            result['summaries'] = [{'machine':m,'backend':b,'repetitions':len(rows),
                'median_wall_s':statistics.median(r['wall_s'] for r in rows),
                'wall_s':[r['wall_s'] for r in rows],
                'median_realtime_factor':statistics.median(r['realtime_factor'] for r in rows)}
                for m in ['hp3','hp4'] for b in BACKENDS
                if (rows := [r for r in result['trials'] if r['machine']==m and r['backend']==b])]
            (OUT/'results.json').write_text(json.dumps(result,indent=2))


if __name__ == '__main__':
    asyncio.run(main())

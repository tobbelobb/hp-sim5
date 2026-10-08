"""Benchmark a frozen collection schedule and verify every recorded encoder read.

This replays commands through real physics. It does not run a new firmware planner
or let the collector choose a new schedule. Output explicitly names that contract.
"""
import argparse
import asyncio
import hashlib
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/python'))
from hp_sim5_research.experiments import sample, write_json
from hp_sim5_research.session import NativeSession


async def replay(args):
    args.output.mkdir(parents=True, exist_ok=False)
    session = NativeSession(ROOT, args.output, args.scene, backend=args.backend, record=not args.no_record)
    result = {'kind': 'frozen_collection_replay', 'backend': args.backend, 'record': not args.no_record,
              'scene_sha256': hashlib.sha256(session.frozen_scene.encode()).hexdigest(),
              'events_sha256': hashlib.sha256(args.events.read_bytes()).hexdigest(),
              'encoder_reads': 0, 'max_encoder_error_deg': 0., 'max_cable_length_error_mm': 0.,
              'max_effector_position_error_mm': 0., 'max_segment_force_error_n': 0.,
              'encoder_tolerance_deg': .01, 'schedule': 'Identical recorded commands and observation steps; no new collection'}
    started = time.monotonic()
    try:
        for line in args.events.read_text().splitlines():
            event = json.loads(line)
            delta = event['step'] - session.step
            if delta < 0:
                raise ValueError('Trace reset/epoch change requires a separate replay')
            if delta:
                await session.advance(delta * session.dt)
            if event['type'] == 'bridge_payload' and event['payload'].get('type') != 'encoder_request':
                await session.handle(event['payload'])
            elif event['type'] == 'encoder_response' and event['axes']:
                angles = session.encoder_angles(event['axes'])
                error = max(abs(first - second) for first, second in zip(angles, event['angles_deg']))
                result['encoder_reads'] += 1
                result['max_encoder_error_deg'] = max(result['max_encoder_error_deg'], error)
                session.observe()
            elif event['type'] == 'observation':
                row = sample(session.world, session.step, session.dt)
                for first, second in zip(row['effectors'], event['sample']['effectors']):
                    if first['path'] != second['path']:
                        raise ValueError('Effector identities changed')
                    result['max_effector_position_error_mm'] = max(result['max_effector_position_error_mm'],
                        math.dist(first['position'], second['position']) * 1000)
                for first, second in zip(row['cables'], event['sample']['cables']):
                    if first['name'] != second['name']:
                        raise ValueError('Cable identities changed')
                    for key, value in first['lengths_m'].items():
                        expected = second['lengths_m'][key]
                        if value is not None and expected is not None:
                            result['max_cable_length_error_mm'] = max(result['max_cable_length_error_mm'],
                                                                      abs(value - expected) * 1000)
                    result['max_segment_force_error_n'] = max(result['max_segment_force_error_n'],
                        max(abs(a - b) for a, b in zip(first['segment_forces_n'], second['segment_forces_n'])))
        result.update(status='complete', steps=session.step, simulated_s=session.step * session.dt,
                      physics_wall_s=session.physics_wall_s, worker_wall_s=session.worker_wall_s,
                      observation_wall_s=session.observation_wall_s)
        result['passed'] = result['encoder_reads'] > 0 and result['max_encoder_error_deg'] <= .01
    except Exception as error:
        result.update(status='failed', error=str(error), passed=False)
        raise
    finally:
        session.close()
        result['wall_s'] = time.monotonic() - started
        result['realtime_factor'] = session.step * session.dt / result['wall_s']
        write_json(args.output / 'replay.json', result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene', type=Path)
    parser.add_argument('events', type=Path)
    parser.add_argument('--output', required=True, type=Path, help='New artifact directory')
    parser.add_argument('--backend', choices=['headless-js', 'native-python', 'native-warp', 'native-warp-cuda'], default='headless-js')
    parser.add_argument('--no-record', action='store_true')
    result = asyncio.run(replay(parser.parse_args()))
    print(json.dumps(result, indent=2))
    sys.exit(0 if result['passed'] else 1)

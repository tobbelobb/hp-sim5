import asyncio
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path.cwd()
sys.path.insert(0, str(ROOT / 'src/python'))
from mcp import Client, StdioServerParameters
from hp_sim5_research.services import RuntimeService

DIRECTORY = Path(__file__).resolve().parent

def save(name, data):
    (DIRECTORY / name).write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')

async def main():
    service = RuntimeService(ROOT, DIRECTORY / 'native-final', record=False).start()
    try:
        params = StdioServerParameters(command=str(ROOT / '.venv/bin/python'),
            args=[str(ROOT / 'scripts/hp_sim5_mcp.py')],
            env={'HP_SIM5_RUNTIME_URL': service.endpoint, 'HP_SIM5_RUNTIME_TOKEN': service.token})
        async with Client(params, read_timeout_seconds=120) as client:
            async def call(name, args):
                response = await client.call_tool(name, args)
                if response.is_error:
                    raise RuntimeError(str(response.content))
                return response.structured_content
            before = await call('runtime_status', {})
            save('hp4-status.json', before)
            fresh = await call('configure_session', {'configuration': {'machine':'hp3', 'layering':False, 'base_radii_mm':[30], 'buildup_factor':0}})
            save('hp3-status.json', fresh)
            assert fresh['world_id'] != before['world_id']
            assert fresh['cable_half_widths_m'] == [0.0]
            assert all(value is False for value in fresh['layering_flags'].values())
            reset = await call('reset_session', {})
            save('reset-status.json', reset)
            assert reset['accepted_settings'] == fresh['accepted_settings']
            job = await call('start_collection', {'options': {'sweepPoints':3,'noiseSamples':4}, 'settling_timeout_s':30})
            save('collection-start.json', job)
            print('Started E4', job['job_id'], flush=True)
            started = time.monotonic()
            while True:
                status = await call('collection_status', {'job_id':job['job_id']})
                save('collection-status.json', status)
                if status['status'] not in ('running','cancelling'):
                    break
                if time.monotonic() - started > 900:
                    status = await call('cancel_collection', {'job_id':job['job_id']})
                    save('collection-status.json', status)
                    break
                await asyncio.sleep(2)
            result = status.get('result', {})
            print(json.dumps({'status':status['status'], 'point_count':result.get('point_count'), 'wall_s':result.get('wall_s'), 'validation':result.get('validation')}), flush=True)
            if result.get('artifacts', {}).get('events'):
                events = [json.loads(line) for line in Path(result['artifacts']['events']).read_text().splitlines()]
                angles = [row['angles_deg'] for row in events if row['type']=='encoder_response' and row['axes']==list('ABCD')]
                ranges = [max(row[i] for row in angles)-min(row[i] for row in angles) for i in range(4)] if angles else []
                save('encoder-movement.json', {'observations':len(angles), 'range_deg_abcd':ranges})
                print('Encoder ranges:', ranges, flush=True)
            assert status['status']=='complete', status
    except Exception as error:
        save('validation-failure.json', {'error':str(error)})
        raise
    finally:
        service.close()

asyncio.run(main())

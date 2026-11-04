"""Collect a bounded native HP3 stiffness pair with production bootstrap collection.

Source scripts/cloud/env.sh first. Outputs are numerical data and journals, with no
fit, truth inputs or calibration acceptance decision. Existing outputs are refused.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import shutil
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from autocal.headless_sim import HeadlessSimulation
from autocal._autocal_common import _write_bootstrap_sweep_config

def save(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')

def fields(text, name):
    return [float(x) for x in re.findall(r'cablePath:' + name + r'\s*=\s*([-+\d.eE]+)', text)]

def finite_measurements(dataset):
    points = [point for sweep in dataset.get('sweeps', []) for point in sweep.get('data_points', [])]
    try:
        return bool(points) and all(
            len(point['raw_angles_deg']) == 4 and all(math.isfinite(float(value)) for value in
                [point['l_drive'], point['l_sensor'], *point['raw_angles_deg']])
            for point in points)
    except (KeyError, TypeError, ValueError):
        return False


def node_identity():
    path = shutil.which('node')
    if path is None:
        raise RuntimeError('Node is missing. Source scripts/cloud/env.sh before collection.')
    version = subprocess.check_output([path, '--version'], text=True).strip()
    if version != 'v23.11.1':
        raise RuntimeError(f'Unsupported Node {version} at {path}. Source scripts/cloud/env.sh; native cloud collection requires Node23.11.1.')
    return path, version


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path, help='New output directory for both owned worlds')
    args_cli = parser.parse_args(argv)
    try:
        node_path, node_version = node_identity()
    except RuntimeError as error:
        parser.error(str(error))
    OUT = args_cli.output.resolve()
    OUT.mkdir(parents=True, exist_ok=False)
    contract = dict(node_executable=node_path, node_version=node_version, experiment='native-hp3-stiffness-pair', machine='hp3', collector_machine='hangprinter_3',
        stiffness_n=[20000,8000], half_width_m=0, base_radii_mm=[30]*4,
        buildup=0, layering=False, sweep_points_per_direction=10,
        target_sweeps=3, target_points=60, feed_mm_min=1400,
        force_low_n=.01, force_mid_n=.05, force_max_n=.5,
        sensor_collection_force_n=1, noise_samples=16, noise_rate_hz=40,
        speedup=25, recording_mode='none', physics_steps_per_world=1000000,
        collector_wall_timeout_s=540, retries=0,
        wall_budget='Collector limit excludes service startup and cleanup',
        acceptance='Complete three sweeps/60 finite measured points in each world, native engine and baked inputs verified. Otherwise paired native-transfer claim inconclusive. Ordinary production validity, underconstraint and offline conventional geometry measured separately.',
        truth_separation='No known anchors or poses used in collector, seeds, recovery or decisions. Offline scene-home effective anchors only, with origin/rotation caveat.')
    save(OUT/'contract.json',contract)
    source = (ROOT/'public/usd_scenes/hp3_rigid_body.usda').read_text()
    source = re.sub(r'(cablePath:halfWidth\s*=\s*)[-+\d.eE]+',r'\g<1>0.0',source)
    results=[]
    for name, stiffness in [('nominal',20000),('soft',8000)]:
        directory=OUT/name; directory.mkdir(exist_ok=False)
        scene=directory/'scene.usda'
        scene.write_text(re.sub(r'(cablePath:stiffness\s*=\s*)[-+\d.eE]+',lambda m:m[1]+str(float(stiffness)),source))
        dataset=directory/'sweeps.json'; cfg=directory/'bootstrap.cfg'
        count=_write_bootstrap_sweep_config(cfg,machine_type='hangprinter_3')
        collector=['--sim','--firmware','rrf','--sweepPoints','10','--feed','1400',
            '--force-low','.01','--force-mid','.05','--force-max','.5',
            '--sensor-collection-force','1','--no-auto-tune-force','--return-to-origin',
            '--speedup','25','--force-base-radii','30,30,30,30','--force-buildup-factor','0']
        args=argparse.Namespace(machine_type='hangprinter_3',find_radii='off',find_buildup_factor='off',
            rrf_config=None, config=None,buildup_factor=0,dataset=dataset,recording_mode='none',keep_sim_alive=False)
        started=time.monotonic(); result=dict(world=name,stiffness_n=stiffness,selected_configs=count)
        previous_step_limit = os.environ.get('HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS')
        os.environ['HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS']='1000000'
        try:
            sim=HeadlessSimulation(args,collector)
            sim.scene=scene.resolve()
            sim.manifest['scene']=str(sim.scene)
            sim.manifest['experiment_contract']=str(OUT/'contract.json')
            sim.manifest['node_executable']=node_path
            sim.manifest['node_version']=node_version
            with sim:
                initial=sim.request('status'); save(directory/'initial-status.json',initial)
                assert initial['physics_engine']=='rust-native',initial['physics_engine']
                baked=(sim.directory/initial['baked_scene_file']).read_text()
                assert fields(baked,'stiffness') and set(fields(baked,'stiffness'))=={float(stiffness)}
                assert set(fields(baked,'halfWidth'))=={0.0}
                assert initial['settings']['enableLayering'] is False
                assert 'R30.00:30.00:30.00:30.00' in sim.manifest['firmware_initial']['M666']
                assert 'Q0.0000' in sim.manifest['firmware_initial']['M666']
                cmd=['node','autocal/control/cli/collect_sweep_data.mjs','--machineType','hangprinter_3',
                    '--sweep-config-file',str(cfg),'--output-file',str(dataset),*collector]
                save(directory/'command.json',cmd)
                save(directory/'input-check.json',dict(scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
                    baked_sha256=hashlib.sha256(baked.encode()).hexdigest(),stiffness_n=fields(baked,'stiffness'),
                    half_widths_m=fields(baked,'halfWidth'),initial=initial,
                    reference_pose_available=False,reference_pose_caveat='Production standalone status exposes engine/clock/config but no effector pose. Conventional scene-home geometry may differ from encoder-zero reference after settling.'))
                with (directory/'collector.log').open('w') as log:
                    process=subprocess.Popen(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                    sim.processes.append(process)
                    sim.track_process('collector',process)
                    try: code=process.wait(timeout=540)
                    except subprocess.TimeoutExpired:
                        process.terminate()
                        try: process.wait(timeout=10)
                        except subprocess.TimeoutExpired: process.kill(); process.wait()
                        result['error']='Collection wall budget exhausted'; code=124
                sim.manifest['exit_code']=code
                sim.manifest['stop_reason']='bootstrap-only'
                result['collector_returncode']=code
                try: save(directory/'final-status.json',sim.request('status'))
                except Exception as error: result['final_status_error']=str(error)
            result['manifest']=str(sim.directory/'manifest.json')
        except Exception as error:
            result['error']=str(error); (directory/'failure.txt').write_text(traceback.format_exc())
        finally:
            if previous_step_limit is None:
                os.environ.pop('HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS', None)
            else:
                os.environ['HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS'] = previous_step_limit
        result['wall_s']=time.monotonic()-started
        if dataset.exists():
            try:
                data=json.loads(dataset.read_text()); sweeps=data.get('sweeps',[])
                result.update(finite_measurements=finite_measurements(data),sweeps=len(sweeps),points=sum(len(s.get('data_points',[])) for s in sweeps),
                    points_per_sweep=[len(s.get('data_points',[])) for s in sweeps],dataset_sha256=hashlib.sha256(dataset.read_bytes()).hexdigest())
            except (ValueError, TypeError, AttributeError) as error:
                result['error'] = f'Invalid collected dataset: {error}'
        result['complete']=(not result.get('error') and result.get('collector_returncode')==0
                            and result.get('points_per_sweep')==[20,20,20]
                            and result.get('finite_measurements') is True)
        results.append(result);save(OUT/'collection-results.json',results)
        print(json.dumps(result),flush=True)
    a=(OUT/'nominal/scene.usda').read_text();b=(OUT/'soft/scene.usda').read_text()
    assert re.sub(r'(cablePath:stiffness\s*=\s*)[-+\d.eE]+','STIFFNESS',a)==re.sub(r'(cablePath:stiffness\s*=\s*)[-+\d.eE]+','STIFFNESS',b)
    return 0 if all(result['complete'] for result in results) else 1

if __name__=='__main__': raise SystemExit(main())

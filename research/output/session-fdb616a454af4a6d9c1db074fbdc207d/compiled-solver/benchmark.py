import json, statistics, time, hashlib, platform
from pathlib import Path
import numpy as np
import warp as wp
from cable_joints_3d.machine_simulation import load_machine_world
from cable_joints_3d.pbd_cable_constraint_solver import PBDCableConstraintSolver
from cable_joints_3d.remote_spool_system import RemoteSpoolSystem
from cable_joints_3d.ecs import PositionComponent, OrientationComponent, EncoderComponent, SceneEntityInfoComponent
from cable_joints_3d.cable_joints_components import CableJointComponent
from usd.cable_scene_loader import open_cable_scene

ROOT=Path.cwd()
OUT=Path(__file__).parent
result={'revision':'working tree','python':platform.python_version(),'warp':wp.__version__,'steps':500,'simulated_s':1,'record':False,'rows':[]}
result['solver_sha256']=hashlib.sha256((ROOT/'src/python/cable_joints_3d/warp_solver.py').read_bytes()).hexdigest()
wp.init()
result['devices']=[str(d) for d in wp.get_devices()]

def state(world):
    rows={}
    for entity in world.entities:
        name=world.get_component(entity,SceneEntityInfoComponent)
        row={}
        for kind,field,attr in [(PositionComponent,'position_m','pos'),(OrientationComponent,'quaternion','quaternion'),(EncoderComponent,'encoder_rad','angle'),(CableJointComponent,'force_n','constraint_force')]:
            c=world.get_component(entity,kind)
            if c is not None:
                v=getattr(c,attr)
                row[field]=v.as_xyzw().tolist() if field=='quaternion' else (v.tolist() if hasattr(v,'tolist') else v)
        rows[name.name if name else str(entity)]=row
    rows['loads']={key:world.get_resource(key) for key in ['torqueModeCableLoadTorques','torqueModeCableLoadStiffnesses','torqueModeCableLoadDampings']}
    return rows

for machine in ['hp3','hp4']:
    scene=open_cable_scene(ROOT/f'public/usd_scenes/{machine}_rigid_body.usda').Flatten().ExportToString()
    references=[]
    for device in [None,'cpu']:
        times=[];solver_times=[];states=[];construct=[]
        for repeat in range(3):
            started=time.perf_counter()
            world=load_machine_world(scene,cable_solver_device=device)
            construct.append(time.perf_counter()-started)
            remote=world.get_system(RemoteSpoolSystem)
            commands=[{'type':'Move','A':-.002*i} for i in range(250)]
            commands += [{'type':'SetTorqueMode','axis':'B','torqueNm':-.003}] + [{}]*249
            for command in commands:
                remote.add_command(command)
            solver=world.get_system(PBDCableConstraintSolver)
            if device and 'first_kernel_wall_s' not in result:
                # Compile on a separate disposable world; don't alter benchmark inputs.
                cold=load_machine_world(scene,cable_solver_device=device)
                t=time.perf_counter();cold.update(.002)
                result['first_kernel_wall_s']=time.perf_counter()-t
            duration=[0.]
            update=solver.update
            def timed(w,dt):
                t=time.perf_counter();update(w,dt);duration[0]+=time.perf_counter()-t
            solver.update=timed
            started=time.perf_counter()
            for step in range(500):
                world.update(.002)
            times.append(time.perf_counter()-started)
            solver_times.append(duration[0]);states.append(state(world))
        row={'machine':machine,'device':device or 'reference','world_wall_s':times,'solver_wall_s':solver_times,'construction_wall_s':construct,'median_world_wall_s':statistics.median(times),'median_solver_wall_s':statistics.median(solver_times),'realtime_factor':1/statistics.median(times),'repeatable':states[0]==states[1]==states[2]}
        if device is None:
            references=states
        else:
            errors={field:0. for field in ['position_m','quaternion','encoder_rad','force_n']}
            for actual,expected in zip(states,references):
                assert actual.keys()==expected.keys()
                for name in actual:
                    if name=='loads':continue
                    assert actual[name].keys()==expected[name].keys()
                    for field,value in actual[name].items():
                        errors[field]=max(errors[field],float(np.max(np.abs(np.asarray(value)-np.asarray(expected[name][field])))))
            row['max_final_difference']=errors
        result['rows'].append(row)
        (OUT/'C03-timing.json').write_text(json.dumps(result,indent=2))
        (OUT/f'C03-{machine}-{device or "reference"}-final.json').write_text(json.dumps(states[0],indent=2))
        print(json.dumps(row),flush=True)

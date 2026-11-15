import sys; sys.path.insert(0,'.')
from sw import *; from mix import mixed
from autocal.pose_planning import predict_sweep_positions, rank_candidates_pose_information
from autocal.active_learning import SweepConfig, generate_candidate_sweeps
HOME=np.linalg.norm(TRUE,axis=1)
def L(cfgs): 
    P=np.concatenate([predict_sweep_positions(TRUE,c,20) for c in cfgs]); return np.linalg.norm(TRUE[None]-P[:,None],axis=2)-HOME
cands=generate_candidate_sweeps(num_anchors=4,dimensions=3,fixed_delta_values_mm=list(np.linspace(-390,390,21)),machine_type='hangprinter_3',max_total_fixed_delta_mm=390.)
def greedy(start,k):
    obs=list(start); out=[]
    for _ in range(k):
        keys={c.normalized_key(tol_mm=10) for c in obs}
        r=rank_candidates_pose_information(TRUE,machine_type='hangprinter_3',low_anchor_z=None,observed_lengths_mm=L(obs),candidates=[c for c in cands if c.normalized_key(tol_mm=10) not in keys],points_per_sweep=20,sigma_mm=0.03,regularization=1e-6,top_k=1)
        obs.append(r[0][1]); out.append(r[0][1])
    return out
boot=[SweepConfig((l,3),(390.,0.),*[a for a in range(3) if a!=l]) for l in range(3)]
seq=greedy(boot,5)
tup=lambda c:(c.fixed_anchors[0],c.fixed_deltas_mm[0],c.fixed_deltas_mm[1])
print([tuple(round(v) for v in tup(c)) for c in seq])
for k in (3,4,5):
    mixed(f'boot const + first {k} profiled',[(BOOT,'const',2.0),([tup(c) for c in seq[:k]],'equal',1.0)])
    mixed(f'first {k} profiled only',[([tup(c) for c in seq[:k]],'equal',1.0)],seed=1,ferr=0.36,noise=0.01)
# after the switch the planner continues from the profiled sweeps alone
seq2=greedy(seq[:4],4); print([tuple(round(v) for v in tup(c)) for c in seq2])
for sd in (1,2,3):
    mixed(f'4 profiled + 4 more, force err seed {sd}',[([tup(c) for c in seq[:4]+seq2],'equal',1.0)],seed=sd,ferr=0.36,noise=0.01)

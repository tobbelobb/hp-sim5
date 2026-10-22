"""E02: exact geometric translation and finite-difference scale sensitivity.

No optimizations or new observations. Truth used only for a separate synthetic
invariance case; recorded-data conditioning is evaluated at the fitted estimate.
"""
import importlib.util
from pathlib import Path
import copy
import json
import sys
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[5]))
from autocal.ellipse_cost import EllipseCostFunction
from autocal.stage_artifacts import read_stage_artifact
from autocal.theoretical_ellipse import canonicalize_anchor_gauge,anchors_matrix_to_opt_vec,anchor_opt_vec_to_matrix
from autocal.tools.rebase_dataset import rebase_dataset,restore_anchors

SESSION=Path(__file__).resolve().parents[1]
import argparse
ap=argparse.ArgumentParser();ap.add_argument('--iteration',type=int,default=3);ap.add_argument('--label',default='E02-corrected');args=ap.parse_args()


def residual(data,anchors):
    cost=EllipseCostFunction(data,use_flex=False,pointwise_filtering=False,sweep_wise_filtering=False,robust_loss=False)
    return np.array([r['residual_z_signed'] for r in cost.pointwise_residual_rows(np.asarray(anchors).ravel())])


def conditioning(data,anchors):
    x=anchors_matrix_to_opt_vec(anchors,'hangprinter_4')
    def f(v):return residual(data,anchor_opt_vec_to_matrix(v,'hangprinter_4',4,3))
    eps=.02
    jac=np.column_stack([(f(x+np.eye(len(x))[j]*eps)-f(x-np.eye(len(x))[j]*eps))/(2*eps) for j in range(len(x))])
    _,singular,vh=np.linalg.svd(jac,full_matrices=False)
    scale=x/np.linalg.norm(x)
    return {'singular_values_per_mm':singular.tolist(),'condition_number':float(singular[0]/singular[-1]),
            'unit_scale_response':float(np.linalg.norm(jac@scale)),
            'weakest_mode_scale_cosine':float(abs(vh[-1]@scale))}


out=SESSION/args.label;out.mkdir(exist_ok=False)
rows=[]
for name in ['REF','W00','W08']:
    history=read_stage_artifact(sorted((SESSION/'E01'/name/'stages').glob('history*.json'))[-1])['payload']['candidates']
    art=history[args.iteration-1]['plan']
    a=np.asarray(art['anchors']);data=art['dataset'];r=residual(data,a)
    row={'name':name,'original':conditioning(data,a),'shifts':[]}
    for mode,p in [('centroid',np.mean(a,axis=0)),('asymmetric',np.array([250.,100.,200.]))]:
        q=np.linalg.norm(a-p,axis=1)-np.linalg.norm(a,axis=1)
        b=canonicalize_anchor_gauge('hangprinter_4',a-p);shifted=rebase_dataset(data,q)
        rr=residual(shifted,b)
        row['shifts'].append({'mode':mode,'position_mm':p.tolist(),'offsets_mm':q.tolist(),
            'residual_count_original':len(r),'residual_count_shifted':len(rr),
            'max_normalized_residual_change':float(np.max(np.abs(rr-r))),
            'restoration':restore_anchors(b,q),'conditioning':conditioning(shifted,b)})
    rows.append(row)
    (out/'results.json').write_text(json.dumps(rows,indent=2)+'\n')
    print(name,'original condition',row['original']['condition_number'],
          [(r['mode'],r['max_normalized_residual_change'],r['conditioning']['condition_number']) for r in row['shifts']])
(out/'results.json').write_text(json.dumps(rows,indent=2)+'\n')

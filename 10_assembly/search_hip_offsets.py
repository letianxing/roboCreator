"""Discrete neutral hip motor-envelope search. No leg ROM or dynamics certification."""
import itertools,json
from pathlib import Path
import numpy as np
from offset_layout_screen import envelope,conflicts,export_urdf

ROOT=Path(__file__).resolve().parents[1]
vectors=[np.eye(3)[axis]*distance for axis in range(3) for distance in (-.075,-.065,-.055,-.045,.045,.055,.065,.075)]
valid=[]
for first,second in itertools.product(vectors,repeat=2):
    p=np.array([0.,.055,.205]);boxes=[]
    for i,axis in enumerate('zxy'):
        if i:p=p+(first if i==1 else second)
        boxes.append(envelope(f'hip_{i}',p,np.eye(3),axis))
    if conflicts(boxes):continue
    # Keep left hip endpoint on left side; no whole pelvis collision check here.
    if p[1]<.035:continue
    minimum=np.min([b['center']-b['half'] for b in boxes],axis=0)
    maximum=np.max([b['center']+b['half'] for b in boxes],axis=0)
    valid.append(dict(offsets_m=[first.tolist(),second.tolist()],terminal_joint_xyz_m=p.tolist(),
        bounds_min_m=minimum.tolist(),bounds_max_m=maximum.tolist(),
        score=float(4*abs(p[2]-.205)+abs(p[0])+abs(p[1]-.055)+np.sum(maximum-minimum)),boxes=boxes))
valid.sort(key=lambda x:x['score'])
if valid:
    export_urdf('hip_search_best',valid[0]['boxes'])
out=dict(status='neutral_motor_boxes_only',evaluated=len(vectors)**2,clear_candidates=len(valid),
    candidates=[{k:v for k,v in row.items() if k!='boxes'} for row in valid[:10]])
(ROOT/'10_assembly/hip_offset_candidates_v04.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2))

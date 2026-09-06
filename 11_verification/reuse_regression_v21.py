"""Check accelerated rigid-part reuse against legacy exact geometry construction."""
import json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'09_cad'))
from geometry_v21 import Geometry,V
g=Geometry();rows=json.loads((ROOT/'04_kinematics/recovery_v21/knee_replan.json').read_text())['cases']
first=rows[0];second=next(r for r in rows if r['h_mm']==120 and r['beta_deg']==45)
g.prepare_arms(first);g.fixed(first)
q=[-40.80874373225732,-3.7319511786843442,-147.32710606460782,65.57583781099416,-1.7418742829245337,29.97766748408413]
old,meta,*_=g.mod.build_geometry({'pose':second['right_pose'],'q':q});ap,_,_,_=g.arms(second,q);fixed,_=g.fixed(second)
results=[]
for group,parts in [('left_arm',ap),('fixed_body',fixed)]:
    for n,s in parts.items():
        prior=old[n];dv=abs(s.Volume-prior.Volume);dc=(g.env['volcenter'](s)-g.env['volcenter'](prior)).Length
        assert dv<1e-4 and dc<1e-5,(n,dv,dc)
        results.append({'part':n,'group':group,'volume_error_mm3':dv,'center_error_mm':dc})
# Conservative bounding boxes must contain exact bounding boxes, not just mesh vertices.
for n,b in g.boxes(ap).items():
    exact=ap[n].BoundBox
    assert all(a<=v+1e-5 for a,v in zip((b.XMin,b.YMin,b.ZMin),(exact.XMin,exact.YMin,exact.ZMin)))
    assert all(a>=v-1e-5 for a,v in zip((b.XMax,b.YMax,b.ZMax),(exact.XMax,exact.YMax,exact.ZMax)))
(ROOT/'11_verification/v21_reuse_regression.json').write_text(json.dumps({'checks':results,'conservative_arm_bounds_checked':True,'scope':'Geometry transform/cache equivalence, not collision/strength release'},indent=2)+'\n',encoding='utf-8')
print('Rigid reuse and conservative bounds regression passed',len(results),flush=True)

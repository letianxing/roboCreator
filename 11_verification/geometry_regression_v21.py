"""Check local rigid transform covariance and independent forward/inverse closure."""
import sys,json,math
from pathlib import Path
import numpy as np
import FreeCAD as A
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'09_cad'));sys.path.insert(0,str(ROOT/'04_kinematics'))
from geometry_v21 import Geometry,V
from search_recovery_v21 import leg_fk
g=Geometry();e=g.env['narrow'];saved=json.loads((ROOT/'04_kinematics/recovery_v21/coupled_search.json').read_text())
cases=[next(r for r in saved['cases'] if r['id']==i) for i in (152,153,165,177)];records=[]
canonical=None;reference={}
for row in cases:
    p,m,j,f=g.leg(row['left_pose']);q=row['q_deg'];Rt=f['shin'].Rotation.multiply(A.Rotation(V(0,1,0),-q[3]));K=V(*j[3]['origin_mm']);T=A.Placement(K,Rt)
    # Transform the completed thigh rail and pad back into knee-local axes.
    local={n:e['placed'](p[n],T.inverse()) for n in ('ThighRail1','KneePad')}
    if canonical is None:canonical=local;reference={n:s.Volume for n,s in p.items()}
    differences={n:local[n].cut(canonical[n]).Volume+canonical[n].cut(local[n]).Volume for n in local}
    assert max(differences.values())<1e-4,differences
    assert max(abs(p[n].Volume-reference[n]) for n in p)<1e-4
    P,R=leg_fk(np.deg2rad(q),row['h_mm'],row['beta_deg'],row['hip_x_mm'])
    B=A.Rotation(V(0,1,0),row['beta_deg']);expect=V(row['hip_x_mm'],0,row['h_mm'])+B.multVec(V(0,0,48))
    error=(f['body'].Base-expect).Length;assert error<.011
    old_delta=V(0,-10,0)-Rt.multVec(V(0,-10,0))
    records.append({'id':row['id'],'local_shape_symmetric_difference_mm3':differences,'root_closure_error_mm':error,
                    'FK_position_error_mm':float(np.linalg.norm(P*1000-np.array(row['left_pose']['foot_translation_mm']))),
                    'legacy_world_offset_error_mm':old_delta.Length,'geometry_pass_not_implied':True})
    print('transform test',row['id'],'closure',error,'old offset error',old_delta.Length,flush=True)
# Zero-roll sagittal reference must agree with v20's exact saved leg geometry.
v20=json.loads((ROOT/'04_kinematics/recovery_v20/contact_candidates.json').read_text())['asymmetric_endpoint'];p,_,_,_=g.leg(v20['left_leg_pose'])
doc=A.openDocument(str(ROOT/'09_cad/body_v20/asymmetric_foot_candidate.FCStd'));diff={}
for n,s in p.items():
    a=s.copy();a.translate(V(0,60,0));b=doc.getObject('L_'+n).Shape
    diff[n]=abs(a.Volume-b.Volume)
    assert diff[n]<1e-4 and (g.env['volcenter'](a)-g.env['volcenter'](b)).Length<1e-6,n
A.closeDocument(doc.Name)
out={'tests':records,'sagittal_regression_volume_difference_mm3':diff,'scope':'Coordinate and shape invariance only; collisions/strength not certified.'}
(ROOT/'11_verification/v21_geometry_regression.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
print('Geometry transform regression passed',flush=True)

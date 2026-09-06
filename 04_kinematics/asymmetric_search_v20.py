"""Six-axis planted-foot IK including hip yaw/roll and ankle roll."""
import json,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'04_kinematics/recovery_v20';p=json.loads((OUT/'contact_candidates.json').read_text());e=p['asymmetric_endpoint']
def rot(axis,q):
    c,s=math.cos(q),math.sin(q)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]]) if axis=='x' else np.array([[c,0,s],[0,1,0],[-s,0,c]]) if axis=='y' else np.array([[c,-s,0],[s,c,0],[0,0,1]])
def fk(q):
    B=rot('y',math.radians(e['body_pitch_deg']));H=np.array([e['hip_x_mm'],0,e['body_height_mm']])/1000
    P=H+B@np.array([0,0,.048]);R=B@rot('z',q[0]);P+=R@np.array([0,0,-.024]);R=R@rot('x',q[1]);P+=R@np.array([0,0,-.024]);R=R@rot('y',q[2]);Rt=R.copy();P+=R@np.array([0,0,-.09]);R=R@rot('y',q[3]);P+=R@np.array([0,0,-.053]);R=R@rot('x',q[4]);P+=R@np.array([0,0,-.037]);R=R@rot('y',q[5]);P+=R@np.array([0,0,-.035]);return P,R,Rt
def solve(target):
    lo=np.deg2rad([-50,-45,-160,0,-25,-35]);hi=np.deg2rad([50,45,160,150,25,35]);rng=np.random.default_rng(20020);best=None
    def residual(q):
        P,R,_=fk(q);return np.r_[P-target,.04*(R-np.eye(3)).ravel()]
    seed=np.deg2rad(e['left_leg_pose']['q_deg'])
    for k in range(10):
        q=seed.copy() if k==0 else rng.uniform(lo,hi)
        for _ in range(160):
            r=residual(q)
            if np.linalg.norm(r)<1e-7:break
            J=np.column_stack([(residual(q+np.eye(6)[j]*1e-6)-r)/1e-6 for j in range(6)])
            step=-np.linalg.solve(J.T@J+1e-6*np.eye(6),J.T@r);step=np.clip(step,-.25,.25);moved=False
            for scale in (1,.5,.15):
                new=np.clip(q+scale*step,lo,hi)
                if np.linalg.norm(residual(new))<np.linalg.norm(r):q=new;moved=True;break
            if not moved:break
        score=np.linalg.norm(residual(q))
        if best is None or score<best[0]:best=(score,q.copy())
        if score<1e-7:break
    q=best[1];P,R,Rt=fk(q);error=np.linalg.norm(P-target)*1000;angle=math.degrees(math.acos(np.clip((np.trace(R)-1)/2,-1,1)))
    return {'target_foot_local_y_mm':float(target[1]*1000),'position_error_mm':float(error),'orientation_error_deg':angle,'q_deg':np.rad2deg(q).tolist(),
     'thigh_rotation':Rt.tolist(),'ik_found':bool(error<.01 and angle<.01),
     'pose':{'name':'asymmetric_abducted','foot_translation_mm':(target*1000).tolist(),'foot_pitch_deg':0,'q_deg':np.rad2deg(q).tolist()}}
def main():
    rows=[]
    for x in (0,20,40,60):
        for dy in (30,50,70):
            r=solve(np.array([x,dy,2])/1000);rows.append(r)
    out={'cases':rows,'body':e,'joint_limits_deg':{'hip_yaw':[-50,50],'hip_roll':[-45,45],'hip_pitch':[-160,160],'knee':[0,150],'ankle_roll':[-25,25],'ankle_pitch':[-35,35]},
     'scope':'Assumed six-axis limits. Foot target Y is relative to the left hip-yaw lateral datum; add60mm for world Y. Numerical pose only; collision/support must be checked.'}
    (OUT/'asymmetric_ik_search.json').write_text(json.dumps(out,indent=2)+'\n');print('IK found',sum(r['ik_found'] for r in rows),'/',len(rows));print([(r['pose']['foot_translation_mm'],r['q_deg']) for r in rows if r['ik_found']])
if __name__=='__main__':main()

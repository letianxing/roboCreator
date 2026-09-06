"""Bounded coupled body/foot pose search, using the existing FreeCAD Python SciPy."""
import json,math,hashlib,itertools
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
import scipy
import recovery_phases_v19 as old
ROOT=Path(__file__).resolve().parents[1];PF=ROOT/'03_system/recovery_search_v21.json';cfg=json.loads(PF.read_text())
OUT=ROOT/'04_kinematics/recovery_v21';OUT.mkdir(exist_ok=True)
LO=np.deg2rad(cfg['joint_lower_deg']);HI=np.deg2rad(cfg['joint_upper_deg'])
rot=old.arm.rot

def leg_fk(q,h,beta,x):
    B=rot('y',math.radians(beta));H=np.array([x,0,h])/1000
    P=H+B@np.array([0,0,.048]);R=B@rot('z',q[0]);P+=R@np.array([0,0,-.024]);R=R@rot('x',q[1]);P+=R@np.array([0,0,-.024]);R=R@rot('y',q[2]);P+=R@np.array([0,0,-.09]);R=R@rot('y',q[3]);P+=R@np.array([0,0,-.053]);R=R@rot('x',q[4]);P+=R@np.array([0,0,-.037]);R=R@rot('y',q[5]);P+=R@np.array([0,0,-.035]);return P,R

def solve_leg(target,h,beta,x):
    def fun(q):
        p,R=leg_fk(q,h,beta,x);return np.r_[p-target,.06*(R-np.eye(3)).ravel()]
    # Multiple branches, plus a sagittal analytic seed. Bounds are kept unchanged.
    angle=math.degrees(math.acos(np.clip((h-37-90*math.cos(math.radians(35)))/90,-1,1)))
    seeds=[[0,0,-angle-beta,angle+35,0,-35],[20,20,-110-beta/2,115,-20,-20],[-20,20,-120,120,-20,-20]]
    best=None
    for seed in seeds:
        q=np.clip(np.deg2rad(seed),LO+1e-8,HI-1e-8)
        res=least_squares(fun,q,bounds=(LO,HI),max_nfev=120,ftol=1e-10,xtol=1e-10,gtol=1e-10)
        cost=np.linalg.norm(fun(res.x))
        if best is None or cost<best[0]:best=(cost,res.x)
        if cost<1e-7:break
    q=best[1];p,R=leg_fk(q,h,beta,x);pe=float(np.linalg.norm(p-target)*1000);oe=math.degrees(math.acos(np.clip((np.trace(R)-1)/2,-1,1)))
    return {'q_deg':np.rad2deg(q).tolist(),'position_error_mm':pe,'orientation_error_deg':oe,
            'ik_found':pe<cfg['position_tolerance_mm'] and oe<cfg['orientation_tolerance_deg']}

def main():
    first=json.loads((ROOT/'04_kinematics/recovery_v19/phase_results.json').read_text())['phase_samples'][0]
    rows=[]
    for h,beta in itertools.product(cfg['body_height_mm'],cfg['body_pitch_deg']):
        theta=math.degrees(math.acos((h-32)/90));x=first['knee_contact_x_mm']+90*math.sin(math.radians(theta))
        right=old.leg.leg_from_body(h,beta,theta-beta,90,35)['pose'];right['foot_translation_mm'][0]+=x
        for dx,dy in itertools.product(cfg['foot_x_relative_hip_mm'],cfg['foot_y_relative_left_hip_mm']):
            target=np.array([x+dx,dy,2])/1000;s=solve_leg(target,h,beta,x)
            rows.append({'id':len(rows),'h_mm':h,'beta_deg':beta,'hip_x_mm':x,'right_pose':right,
                'left_pose':{'name':'candidate','q_deg':s['q_deg'],'foot_pitch_deg':0,'foot_translation_mm':(target*1000).tolist()},**s})
        print('body',h,beta,'IK',sum(r['ik_found'] for r in rows[-12:]),'/12',flush=True)
    # Re-test all original v20 failed targets with a different existing numerical solver.
    v20=json.loads((ROOT/'04_kinematics/recovery_v20/asymmetric_ik_search.json').read_text());b=v20['body'];controls=[]
    for r in v20['cases']:
        t=np.array(r['pose']['foot_translation_mm'])/1000
        controls.append({'target_mm':(t*1000).tolist(),**solve_leg(t,b['body_height_mm'],b['body_pitch_deg'],b['hip_x_mm'])})
    out={'input_sha256':hashlib.sha256(PF.read_bytes()).hexdigest(),'scipy_version':scipy.__version__,'cases':rows,'v20_rechecks':controls,
         'scope':'Numerical IK only. No global infeasibility claim. All CAD/contact/dynamic checks separate.'}
    (OUT/'coupled_search.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
    print('Total',sum(r['ik_found'] for r in rows),'/',len(rows),'v20 recovered',sum(r['ik_found'] for r in controls),flush=True)
if __name__=='__main__':main()

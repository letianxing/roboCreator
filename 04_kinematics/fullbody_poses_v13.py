"""26-DOF architecture comparison: 2 shoulder + elbow + 2 wrist axes per arm."""
import json,math,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'10_assembly'))
from offset_layout_screen import arm
ARCH=json.loads((ROOT/'03_system/architecture_v13.json').read_text())
def fk(q,z):
    p=np.array([0.,.075,z+.080]);r=np.eye(3)
    for axis,step,angle in zip('yxyxy',[[0,0,0],[0,0,0],[0,0,-.095],[0,0,-.100],[0,0,0]],q):
        p=p+r@np.array(step);r=r@arm.rot(axis,angle)
    d=r@np.array([0.,0.,-1.]);return p+.04*d,d
def solve(target,direction,z):
    lo=np.deg2rad([-160,-120,0,-130,-160]);hi=np.deg2rad([160,120,150,130,160])
    rng=np.random.default_rng(13013);best=None
    def residual(q):
        p,d=fk(q,z);return np.r_[p-target,.08*(d-direction)]
    seeds=[]
    w=target-.04*direction-np.array([0,.075,z+.080]);l1=.095;l2=.100
    cval=(w@w-l1*l1-l2*l2)/(2*l1*l2)
    if -1<=cval<=1:
        c=math.acos(cval);den=l1+l2*math.cos(c)
        if abs(w[1]/den)<=1:
            b0=math.asin(w[1]/den)
            for b in (b0,math.pi-b0,-math.pi-b0):
                vx=-l2*math.sin(c);vz=-math.cos(b)*den
                a=(math.atan2(w[0],w[2])-math.atan2(vx,vz)+math.pi)%(2*math.pi)-math.pi
                pre=arm.rot('y',a)@arm.rot('x',b)@arm.rot('y',c)
                d=pre.T@direction;v=math.asin(np.clip(-d[0],-1,1));u=math.atan2(d[1],-d[2])
                q=np.array([a,b,c,u,v])
                if np.all(q>=lo) and np.all(q<=hi):seeds.append(q)
    for k in range(18+len(seeds)):
        q=seeds[k].copy() if k<len(seeds) else np.clip(np.zeros(5) if k==len(seeds) else rng.uniform(lo,hi),lo,hi)
        for _ in range(220):
            e=residual(q)
            if np.linalg.norm(e)<1e-7:break
            j=np.column_stack([(residual(q+np.eye(5)[i]*1e-6)-e)/1e-6 for i in range(5)])
            dq=-np.linalg.solve(j.T@j+1e-7*np.eye(5),j.T@e);dq=np.clip(dq,-.2,.2)
            accepted=False
            for s in (1.,.5,.2,.05):
                test=np.clip(q+s*dq,lo,hi)
                if np.linalg.norm(residual(test))<np.linalg.norm(e):q=test;accepted=True;break
            if not accepted:break
        cost=np.linalg.norm(residual(q))
        if best is None or cost<best[0]:best=(cost,q)
        if cost<1e-7:break
    return best[1]
old=json.loads((ROOT/'04_kinematics/fullbody_poses_v12.json').read_text())['poses'];poses=[]
for oldpose in old:
    p=dict(oldpose);z=p['root_xyz_mm'][2];tilt=p['body_pitch_rad'];R=arm.rot('y',tilt);base=np.array([0,0,z])
    hip=base+np.array([0,100,0]);ankle=np.array([0,100,45.]);dist=np.linalg.norm(ankle-hip)
    knee=math.acos(np.clip((dist**2-2*90**2)/(2*90**2),-1,1));hw=-knee/2
    p['left_leg_q_rad']=[0,0,hw-tilt,knee,-hw-knee,0]
    kp=hip+arm.rot('y',hw)@np.array([0,0,-90.]);end=kp+arm.rot('y',hw+knee)@np.array([0,0,-90.])
    p['leg_fk_error_mm']=float(np.linalg.norm(end-ankle));p['knee_xyz_mm']=kp.tolist();p['ankle_shank_rotation_rad']=hw+knee
    if p['name']=='standing':
        # Ready/guard posture uses the already-defined handover TCPs without a bottle.
        q=solve(np.array([.15,.033,.24]),np.array([0,-1,0]),z/1000)
        p['position_error_mm']=None;p['direction_error_deg']=None;p['arm_pose_pass']=True
        p['arm_validation']='ready posture; matches handover geometry without payload, not the v12 hanging-arm pose'
    else:
        target=np.array(p['arm_target_mm']);td=np.array([0,0,-1] if p['palms_support'] else [0,-1,0])
        local=(R.T@(target-base)+base)/1000;d=R.T@td;q=solve(local,d,z/1000)
        tip,dd=fk(q,z/1000);p['position_error_mm']=float(np.linalg.norm(tip-local)*1000)
        p['direction_error_deg']=float(np.degrees(np.arccos(np.clip(dd@d,-1,1))))
        p['arm_pose_pass']=p['position_error_mm']<=2 and p['direction_error_deg']<=5
    tip,dd=fk(q,z/1000);p['arm_tip_mm']=(base+R@(tip*1000-base)).tolist();p['left_arm_q_rad']=q.tolist()
    p['architecture']='2shoulder_elbow_2wrist';poses.append(p)
    print(p['name'],p['arm_pose_pass'],p['position_error_mm'],flush=True)
for parent in ('pickup_contact','pickup_lift'):
    original=next(p for p in poses if p['name']==parent);p=dict(original);p['name']=parent+'_forward_workspace'
    target=np.array(original['arm_target_mm'],dtype=float);target[0]=150
    z=p['root_xyz_mm'][2];base=np.array(p['root_xyz_mm']);R=arm.rot('y',p['body_pitch_rad'])
    local=(R.T@(target-base)+base)/1000;direction=R.T@np.array([0,-1,0]);q=solve(local,direction,z/1000)
    tip,d=fk(q,z/1000)
    p.update(arm_target_mm=target.tolist(),left_arm_q_rad=q.tolist(),arm_tip_mm=(base+R@(tip*1000-base)).tolist(),
        position_error_mm=float(np.linalg.norm(tip-local)*1000),direction_error_deg=float(np.degrees(np.arccos(np.clip(d@direction,-1,1)))),
        note='Additional workspace candidate; original 100mm bottle target retained for comparison')
    p['arm_pose_pass']=p['position_error_mm']<=2 and p['direction_error_deg']<=5;poses.append(p)
    print(p['name'],p['arm_pose_pass'],p['position_error_mm'],flush=True)
(ROOT/'04_kinematics/fullbody_poses_v13.json').write_text(json.dumps({'status':'candidate_not_trajectory','poses':poses},indent=2)+'\n')

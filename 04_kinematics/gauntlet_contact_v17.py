"""Compare forearm-fixed pad IK under current and added shoulder-yaw architectures."""
import json, math, hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];PFILE=ROOT/'03_system/single_leg_v17.json';OUT=ROOT/'04_kinematics/gauntlet_contact_v17.json'
p=json.loads(PFILE.read_text(encoding='utf-8'));cfg=p['arm']
def rot(axis,q):
    c,s=math.cos(q),math.sin(q)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]]) if axis=='x' else np.array([[c,0,s],[0,1,0],[-s,0,c]]) if axis=='y' else np.array([[c,-s,0],[s,c,0],[0,0,1]])
def fk(q,height,bodypitch,architecture,mount_roll=0):
    B=rot('y',bodypitch);base=np.array([0.,0.,height]);shoulder=base+B@np.array(cfg['shoulder_origin_body_mm'])/1000
    R=B@rot('y',q[0])@rot('x',q[1]);upstream=3 if architecture==26 else 4
    if architecture==28:R=R@rot('z',q[2])
    elbow=shoulder+R@np.array([0,0,-cfg['upper_arm_mm']/1000]);R=R@rot('y',q[upstream-1])
    wrist=elbow+R@np.array([0,0,-cfg['forearm_mm']/1000])
    sign=1 if mount_roll==0 else -1
    center=wrist+R@np.array([-.025*sign,0,.012]);normal=R@np.array([-1.*sign,0,0])
    corners=[center+R@np.array([0,sy*.020,sx*.010]) for sy in (-1,1) for sx in (-1,1)]
    return center,normal,{'shoulder':shoulder,'elbow':elbow,'wrist':wrist,'pad_corners':corners,'forearm_rotation':R}
def solve(target,h,beta,architecture,seeds=9,mount_roll=0,optimize_body=False):
    n=3 if architecture==26 else 4
    lo=np.deg2rad([-160,-120,0] if n==3 else [-160,-120,-160,0]);hi=np.deg2rad([160,120,150] if n==3 else [160,120,160,150])
    up=n
    if optimize_body:
        hb=cfg['body_height_optimization_bounds_mm'];bb=cfg['body_pitch_optimization_bounds_deg']
        lo=np.r_[lo,hb[0]/1000,math.radians(bb[0])];hi=np.r_[hi,hb[1]/1000,math.radians(bb[1])];n+=2
    rng=np.random.default_rng(17017);best=None
    def residual(q):
        z,tilt=(q[up],q[up+1]) if optimize_body else (h,beta)
        xyz,normal,_=fk(q[:up],z,tilt,architecture,mount_roll);return np.r_[xyz-target,.08*(normal-np.array([0,0,-1.]))]
    for seed in range(seeds):
        q=(lo+hi)/2 if seed==0 else rng.uniform(lo,hi)
        for _ in range(110):
            e=residual(q);norm=np.linalg.norm(e)
            if norm<1e-8:break
            J=np.column_stack([(residual(q+np.eye(n)[i]*1e-6)-e)/1e-6 for i in range(n)])
            step=-np.linalg.solve(J.T@J+1e-6*np.eye(n),J.T@e);step=np.clip(step,-.3,.3)
            accepted=False
            for scale in (1,.5,.15):
                trial=np.clip(q+step*scale,lo,hi)
                if np.linalg.norm(residual(trial))<norm:q=trial;accepted=True;break
            if not accepted:break
        norm=np.linalg.norm(residual(q))
        if best is None or norm<best[0]:best=(norm,q.copy())
        if norm<1e-8:break
    q=best[1]
    if optimize_body:h,beta=q[up],q[up+1]
    pos,normal,nodes=fk(q[:up],h,beta,architecture,mount_roll)
    error=np.linalg.norm(pos-target)*1000;angle=math.degrees(math.acos(np.clip(normal@np.array([0,0,-1.]),-1,1)))
    exact=error<=cfg['contact_position_tolerance_mm'] and angle<=cfg['contact_normal_tolerance_deg']
    return {'architecture_dof':architecture,'body_height_mm':h*1000,'body_pitch_deg':math.degrees(beta),'target_mm':(target*1000).tolist(),
       'upstream_q_deg':np.rad2deg(q[:up]).tolist(),'mount_roll_deg':mount_roll,'body_pose_optimized':optimize_body,'contact_mm':(pos*1000).tolist(),'normal':normal.tolist(),'position_error_mm':float(error),'normal_error_deg':angle,
       'finite_search_pose_found':bool(exact),'minimum_pad_corner_z_mm':min(c[2] for c in nodes['pad_corners'])*1000,
       'nodes_mm':{k:(v*1000).tolist() for k,v in nodes.items() if k in ['shoulder','elbow','wrist']},'fullbody_verified':False}
def main():
    records=[]
    for arch in (26,28):
        for h in cfg['body_height_samples_mm']:
            for beta in cfg['body_pitch_samples_deg']:
                r=solve(np.array(cfg['fixed_contact_xyz_mm'])/1000,h/1000,math.radians(beta),arch)
                r['scenario']='old_distant_contact';records.append(r)
        # New nearer contacts are explicitly additional cases; original failures are retained.
        for target in ([150,100,0],[150,125,0],[180,100,0],[180,125,0]):
            for beta in (75,90):
                r=solve(np.array(target)/1000,.080,math.radians(beta),arch,seeds=14)
                r['scenario']='nearer_contact_candidate';records.append(r)
        print('arch',arch,'found',sum(r['finite_search_pose_found'] for r in records if r['architecture_dof']==arch),flush=True)
        for mounting in cfg['mount_roll_deg_candidates']:
            for target in ([200,150,0],[150,75,0],[150,125,0],[180,100,0]):
                r=solve(np.array(target)/1000,.080,math.radians(90),arch,seeds=24,mount_roll=mounting,optimize_body=True)
                r['scenario']='body_pose_and_pad_mount_search';records.append(r)
    # Finite differences demonstrate the wrist has no authority over a forearm-mounted pad.
    probe=np.deg2rad([-50,25,60,20,-30]);base=fk(probe,.12,math.radians(60),26)
    wrist_effect=[]
    for j in (3,4):
        moved=probe.copy();moved[j]+=.25;pos,norm,_=fk(moved,.12,math.radians(60),26)
        wrist_effect.append({'joint_index':j,'delta_q_rad':.25,'pad_translation_mm':float(np.linalg.norm(pos-base[0])*1000),'normal_change':float(np.linalg.norm(norm-base[1]))})
    summary={'version':'v0.17','input_sha256':hashlib.sha256(PFILE.read_bytes()).hexdigest(),'records':records,'wrist_authority_check':wrist_effect,
       'added_shoulder_yaw_mass_reference_kg':2*.082,'proposed_dof':28,'dof_change_status':'candidate_pending_fullbody_validation',
       'limitations':['Finite multistart failure is not a proof of global unreachability','Body poses are not coupled to legs, COM or ground reaction constraints',
         'No self-collision, torso/shell interference or full getup trajectory has passed','Extra shoulder yaw has not been physically packaged','Wrist angles cannot control a forearm-fixed pad']}
    OUT.write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    for arch in (26,28):
        best=min([r for r in records if r['architecture_dof']==arch],key=lambda r:r['position_error_mm']+r['normal_error_deg'])
        print('best',arch,best,flush=True)
if __name__=='__main__':main()

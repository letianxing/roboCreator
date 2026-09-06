"""Whole-body pose candidates: exact two-link leg closure plus offset-arm numerical IK."""
import json,sys,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'10_assembly'))
import offset_layout_screen as arm
cfg=json.loads((ROOT/'03_system/fullbody_v12_parameters.json').read_text());geo=cfg['geometry']

def pose(name,z,tilt,target,direction,payload=False,palm=False):
    base=np.array([0.,0.,z]);R=arm.arm.rot('y',tilt)
    hip=base+R@np.array([-45.,130.,0.]);ankle=np.array([39.,130.,45.])
    dx=ankle[0]-hip[0];dz=hip[2]-ankle[2];length=math.hypot(dx,dz)
    assert length<=180 and abs(hip[1]-ankle[1])<1e-7
    knee=math.acos(np.clip((length**2-2*90**2)/(2*90**2),-1,1))
    hip_world=-math.atan2(dx,dz)-knee/2
    hip_local=hip_world-tilt;ankle_q=-hip_world-knee
    kp=hip+arm.arm.rot('y',hip_world)@np.array([0,0,-90.])
    ap=kp+arm.arm.rot('y',hip_world+knee)@np.array([0,0,-90.])
    assert np.linalg.norm(ap-ankle)<1e-7
    # Existing arm solver operates in metres in the unpitched reference frame.
    local_target=(R.T@(np.array(target)-base)+base)/1000
    local_dir=R.T@np.array(direction)
    sol=arm.solve([[0,.055,0],[0,0,-.055]],local_target,local_dir,(z+90)/1000)
    q=np.deg2rad(sol['q_deg'])
    if not sol['pose_pass']:
        # Refine the best finite-iteration candidate without widening joint limits.
        lo=np.deg2rad([-160,-120,-160,0,-120]);hi=np.deg2rad([160,120,160,150,120])
        def residual(v):
            p,d,_,_=arm.fk(v,[[0,.055,0],[0,0,-.055]],(z+90)/1000)
            return np.r_[p-local_target,.08*(d-local_dir)]
        for _ in range(350):
            e=residual(q)
            if np.linalg.norm(e)<1e-7:break
            j=np.column_stack([(residual(q+np.eye(5)[k]*1e-6)-e)/1e-6 for k in range(5)])
            dq=-np.linalg.solve(j.T@j+1e-8*np.eye(5),j.T@e);dq=np.clip(dq,-.15,.15)
            accepted=False
            for step in (1.,.5,.2,.05):
                candidate=np.clip(q+step*dq,lo,hi)
                if np.linalg.norm(residual(candidate))<np.linalg.norm(e):q=candidate;accepted=True;break
            if not accepted:break
        pp,dd,_,_=arm.fk(q,[[0,.055,0],[0,0,-.055]],(z+90)/1000)
        sol['direction_error_deg']=float(np.degrees(np.arccos(np.clip(dd@local_dir,-1,1))))
        sol['pose_pass']=bool(np.linalg.norm(pp-local_target)<=.002 and sol['direction_error_deg']<=5)
    tip,d,_,origins=arm.fk(q,[[0,.055,0],[0,0,-.055]],(z+90)/1000)
    tip=base+R@(tip*1000-base)
    result={'name':name,'root_xyz_mm':base.tolist(),'body_pitch_rad':tilt,'left_leg_q_rad':[0,0,hip_local,knee,ankle_q,0],
        'left_arm_q_rad':q.tolist(),'arm_target_mm':target,'arm_tip_mm':tip.tolist(),
        'arm_pose_pass':sol['pose_pass'],'position_error_mm':float(np.linalg.norm(tip-target)),
        'direction_error_deg':sol['direction_error_deg'],'knee_xyz_mm':kp.tolist(),
        'leg_fk_error_mm':float(np.linalg.norm(ap-ankle)),
        'ankle_shank_rotation_rad':hip_world+knee,'payload_carried':payload,'palms_support':palm}
    return result

poses=[];search=[]
standing=pose('standing',190,0,[150,33,240],[0,-1,0])
# Resting arms are a prescribed joint posture, not a solved task.
standing['left_arm_q_rad']=[-.6,.15,0,1.2,0]
tip,d,_,_=arm.fk(np.array(standing['left_arm_q_rad']),[[0,.055,0],[0,0,-.055]],.280)
standing.update(arm_tip_mm=(tip*1000).tolist(),arm_target_mm=None,arm_pose_pass=True,
                position_error_mm=None,direction_error_deg=None,arm_validation='prescribed rest joints, not IK success')
poses.append(standing)
for name,z,targets,payload in [
    ('pickup_contact',150,[[x,33,130] for x in (100,120,140)],False),
    ('pickup_lift',150,[[x,33,h] for h in (145,155,165) for x in (100,120,140)],True),
    ('handover',190,[[x,33,240] for x in (150,130,110)],True)]:
    candidates=[]
    for target in targets:
        p=pose(name,z,0,target,[0,-1,0],payload)
        candidates.append(p);search.append({'name':name,'target':target,'pass':p['arm_pose_pass'],'error_mm':p['position_error_mm']})
        if p['arm_pose_pass']:break
    poses.append(min(candidates,key=lambda x:(not x['arm_pose_pass'],x['position_error_mm'])))
candidates=[]
for z,angle,x,y in [(120,60,200,150),(120,60,180,170),(110,60,180,170),(110,45,180,170),(110,45,160,180),(110,60,160,180)]:
    p=pose('getup_intermediate',z,math.radians(angle),[x,y,0],[0,0,-1],False,True)
    candidates.append(p);search.append({'name':'getup_intermediate','root':z,'tilt':angle,'target':[x,y,0],'pass':p['arm_pose_pass'],'error_mm':p['position_error_mm']})
    if p['arm_pose_pass']:break
poses.append(min(candidates,key=lambda x:(not x['arm_pose_pass'],x['position_error_mm'])))
out=ROOT/'04_kinematics/fullbody_poses_v12.json';out.write_text(json.dumps({'poses':poses,'search':search,'status':'pose_candidates_not_complete_trajectories'},indent=2)+'\n')
for p in poses:print(p['name'],'arm',p['arm_pose_pass'],'error_mm',p['position_error_mm'],'target',p['arm_target_mm'],flush=True)

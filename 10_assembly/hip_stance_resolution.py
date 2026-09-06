"""Resolve one hip-offset/leg-length conflict and audit upper-body proxies.
No physical hardware operation. Mass, dynamics and complete assembly remain unverified.
"""
import json,itertools
from pathlib import Path
import numpy as np
from offset_layout_screen import arm,envelope,conflicts,fk,solve,encode

ROOT=Path(__file__).resolve().parents[1]

def leg_pose(root_z,ankle_y):
    hip=np.array([-.045,.13,root_z]);ankle=np.array([0.,ankle_y,.025])
    dx=ankle[0]-hip[0];dz=hip[2]-ankle[2];dy=ankle[1]-hip[1]
    dist=float(np.linalg.norm(ankle-hip));l=.09
    out={'root_z_m':root_z,'ankle_xyz_m':ankle.tolist(),'hip_pitch_origin_m':hip.tolist(),
         'distance_m':dist,'length_only_pass':dist<=2*l}
    if abs(dy)>1e-9 or dist>2*l:
        out['note']='3D reach bound only; no roll/yaw IK solved'
        return out
    knee=float(np.arccos(np.clip((dist**2-2*l*l)/(2*l*l),-1,1)))
    hipq=-np.arctan2(dx,dz)-knee/2
    ankleq=-hipq-knee
    r=arm.rot('y',hipq)
    k=hip+r@np.array([0,0,-l])
    ra=r@arm.rot('y',knee)
    end=k+ra@np.array([0,0,-l])
    assert np.linalg.norm(end-ankle)<1e-10
    boxes=[];p=np.array([0.,.055,root_z])
    for i,axis in enumerate('zxy'):
        if i==1:p=p+np.array([-.045,0,0])
        if i==2:p=p+np.array([0,.075,0])
        boxes.append(envelope(f'L_hip_{i}',p,np.eye(3),axis))
    boxes.append(envelope('L_knee_pitch',k,r,'y'))
    boxes.append(envelope('L_ankle_pitch',end,ra,'y'))
    reflect=np.diag([1,-1,1])
    right=[{**b,'name':b['name'].replace('L_','R_'),'center':reflect@b['center'],
            'rotation':reflect@b['rotation']} for b in boxes]
    # SAT works for orthonormal reflected box bases, not exported as rotations.
    all_boxes=boxes+right
    out.update(q_deg={'hip_pitch':float(np.degrees(hipq)),'knee_pitch':float(np.degrees(knee)),
                       'ankle_pitch':float(np.degrees(ankleq))},knee_xyz_m=k.tolist(),
               fk_error_m=float(np.linalg.norm(end-ankle)),
               full_extension_reserve_m=2*l-dist,
               ten_motor_proxy_conflicts=conflicts(all_boxes),boxes=all_boxes)
    return out

def main():
    original=leg_pose(.205,.055)
    standing=leg_pose(.185,.13)
    crouch=leg_pose(.145,.13)
    path=[]
    for z in np.linspace(.185,.145,81):
        p=leg_pose(float(z),.13)
        path.append({'root_z_m':float(z),'conflicts':p['ten_motor_proxy_conflicts'],
                     'q_deg':p['q_deg'],'fk_error_m':p['fk_error_m']})
    shoulders=[]
    offsets=[[0,.055,0],[0,0,-.055]]
    for task,z,target in [('pickup',.235,[.1,.04,.13]),('handover',.275,[.15,.04,.24])]:
        q=solve(offsets,np.array(target),np.array([0,-1,0]),z)
        torso={'name':'torso_structure_proxy','center':np.array([0.,0.,z-.045]),
               'rotation':np.eye(3),'half':np.array([.05,.06,.045])}
        frames=[]
        for u in np.linspace(0,1,101):
            _,_,boxes,_=fk(u*np.deg2rad(q['q_deg']),offsets,z)
            frames.append(conflicts(boxes+[torso]))
        q.update(task=task,shoulder_z_m=z,proxy_conflicting_frame_count=sum(bool(x) for x in frames),
                 endpoint_with_torso_conflicts=frames[-1],
                 conflict_pairs=sorted({tuple(pair) for frame in frames for pair in frame}))
        shoulders.append(q)
    result={'status':'conditional_geometric_resolution_only','original':original,'standing':standing,
       'crouch':crouch,'lower_body_path':path,'shoulder_recheck':shoulders,
       'height_candidate_m':.185+.095+.1,'feet_center_spacing_m':.26,
       'foot_outer_width_m':.26+.055,
       'limitations':['Hip yaw/roll fixed zero during planar squat; no general ROM check',
          '10 lower-body motor boxes only; two ankle-roll motors absent and unresolved',
          'No structural pelvis, brackets, bearings, foot shells, wiring or full-body collision',
          'Torso is assumed 100x120x90mm structural box, not detailed electronics/costume',
          'Shoulder tests are left arm only; no bottle or opposite arm collision',
          '38cm height is reference stack, not verified highest occupied point during motion',
          'Wider stance is candidate, not user-confirmed aesthetic or dynamically verified choice',
          'No continuous collision guarantee, inertia, force, power or impact validation']}
    (ROOT/'10_assembly/hip_stance_results_v05.json').write_text(json.dumps(result,default=encode,indent=2)+'\n')
    for n,p in [('original',original),('standing',standing),('crouch',crouch)]:
        print(n,'reach_mm',round(p['distance_m']*1000,2),'q',p.get('q_deg'),
              'proxy_conflicts',p.get('ten_motor_proxy_conflicts'))
    print('lower_body_conflicting_samples',sum(bool(x['conflicts']) for x in path),'of',len(path))
    for q in shoulders:print('shoulder',q['task'],'pose_pass',q['pose_pass'],
        'endpoint_conflicts',q['endpoint_with_torso_conflicts'],'path_conflict_frames',q['proxy_conflicting_frame_count'])

if __name__=='__main__':main()

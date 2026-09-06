"""12-motor lower-body packaging search, flat feet and planar squat only."""
import itertools,json
from pathlib import Path
import numpy as np
from offset_layout_screen import arm,envelope,overlap,encode

ROOT=Path(__file__).resolve().parents[1]

def pairs(boxes):
    found=[]
    bounds=[np.abs(b['rotation'])@b['half']+.002 for b in boxes]
    for i,j in itertools.combinations(range(len(boxes)),2):
        if np.any(np.abs(boxes[i]['center']-boxes[j]['center'])>bounds[i]+bounds[j]):continue
        if overlap(boxes[i],boxes[j]):found.append([boxes[i]['name'],boxes[j]['name']])
    return found

def evaluate(offsets,root_z,ankle_z,roll_x):
    p=np.array([0.,.055,root_z]);boxes=[]
    for i,axis in enumerate('zxy'):
        if i:p=p+np.array(offsets[i-1])
        boxes.append(envelope(f'L_hip_{i}',p,np.eye(3),axis))
    hip=p.copy();end=np.array([0.,hip[1],ankle_z])
    dx=end[0]-hip[0];dz=hip[2]-ankle_z;d=float(np.hypot(dx,dz))
    if not .02<d<.18:return {'pass':False,'reason':'leg_length'}
    knee=float(np.arccos(np.clip((d*d-2*.09**2)/(2*.09**2),-1,1)))
    hp=float(-np.arctan2(dx,dz)-knee/2);ap=-hp-knee
    if not (-np.deg2rad(100)<=hp<=np.deg2rad(100) and knee<=np.deg2rad(140) and abs(ap)<=np.deg2rad(75)):
        return {'pass':False,'reason':'assumed_ROM'}
    r=arm.rot('y',hp);k=hip+r@np.array([0,0,-.09]);ra=r@arm.rot('y',knee)
    actual=k+ra@np.array([0,0,-.09]);err=float(np.linalg.norm(actual-end))
    assert err<1e-9
    boxes.append(envelope('L_knee',k,r,'y'))
    boxes.append(envelope('L_ankle_pitch',end,ra,'y'))
    # Ankle pitch rotation ap cancels leg orientation; roll=0 for flat-foot squat.
    rf=ra@arm.rot('y',ap)
    assert np.allclose(rf,np.eye(3))
    roll_origin=end+rf@np.array([roll_x,0,0])
    boxes.append(envelope('L_ankle_roll',roll_origin,rf,'x'))
    mirror=np.diag([1,-1,1])
    boxes += [{**b,'name':b['name'].replace('L_','R_'),'center':mirror@b['center'],
               'rotation':mirror@b['rotation']} for b in boxes.copy()]
    ground=min(float(b['center'][2]-(np.abs(b['rotation'])@b['half'])[2]) for b in boxes)
    collisions=pairs(boxes)
    # Sole footprint covers both ankle envelopes, with 5mm packaging allowance.
    ankles=[b for b in boxes if b['name'].startswith('L_ankle')]
    amin=np.min([b['center']-np.abs(b['rotation'])@b['half'] for b in ankles],axis=0)
    amax=np.max([b['center']+np.abs(b['rotation'])@b['half'] for b in ankles],axis=0)
    return {'pass':not collisions and ground>=.010,'reason':'proxy_clear' if not collisions and ground>=.010 else 'collision_or_ground',
      'q_deg':[float(np.degrees(hp)),float(np.degrees(knee)),float(np.degrees(ap))],
      'conflicts':collisions,'motor_ground_clearance_m':ground,'fk_error_m':err,
      'hip_xyz_m':hip.tolist(),'ankle_pitch_xyz_m':end.tolist(),'ankle_roll_xyz_m':roll_origin.tolist(),
      'left_sole_xy_min_m':(amin[:2]-.005).tolist(),'left_sole_xy_max_m':(amax[:2]+.005).tolist(),
      'boxes':boxes}

def main():
    hips=json.loads((ROOT/'10_assembly/hip_offset_candidates_v04.json').read_text())['candidates']
    accepted=[];reasons={};tried=0
    for index,h in enumerate(hips):
      for root,az,rx in itertools.product((.165,.175,.185,.195),(.045,.055),(.085,.095,.105)):
        tried+=1;states=[evaluate(h['offsets_m'],z,az,rx) for z in (root,root-.02,root-.04)]
        if not all(s['pass'] for s in states):
            key=next(s['reason'] for s in states if not s['pass']);reasons[key]=reasons.get(key,0)+1;continue
        minxy=np.min([s['left_sole_xy_min_m'] for s in states],axis=0)
        maxxy=np.max([s['left_sole_xy_max_m'] for s in states],axis=0)
        width=2*maxxy[1];length=maxxy[0]-minxy[0]
        # Planning score only: narrower stance then smaller sole then hip fore-aft displacement.
        score=width+length+abs(states[0]['hip_xyz_m'][0])
        accepted.append({'hip_candidate_index':index,'offsets_m':h['offsets_m'],'root_z_m':root,
          'ankle_z_m':az,'roll_offset_x_m':rx,'reference_height_m':root+.195,
          'three_sample_outer_width_m':float(width),'three_sample_sole_length_m':float(length),'score':float(score)})
    accepted.sort(key=lambda x:x['score'])
    verified=[]
    for c in accepted[:10]:
        states=[evaluate(c['offsets_m'],float(z),c['ankle_z_m'],c['roll_offset_x_m']) for z in np.linspace(c['root_z_m'],c['root_z_m']-.04,81)]
        if all(s['pass'] for s in states):
            minxy=np.min([s['left_sole_xy_min_m'] for s in states],axis=0)
            maxxy=np.max([s['left_sole_xy_max_m'] for s in states],axis=0)
            c={**c,'sampled_left_sole_xy_min_m':minxy.tolist(),'sampled_left_sole_xy_max_m':maxxy.tolist(),
               'sampled_outer_width_m':float(2*maxxy[1]),'sampled_sole_length_m':float(maxxy[0]-minxy[0])}
            verified.append({'candidate':c,'states':[{k:v for k,v in s.items() if k!='boxes'} for s in states],
                'standing_boxes':states[0]['boxes'],'crouch_boxes':states[-1]['boxes']})
            break
    result={'status':'flat_foot_squat_proxy_only','tried':tried,'three_pose_accepted':len(accepted),'rejection_reasons':reasons,
      'top_candidates':accepted[:10],'verified':verified,
      'limitations':['Only top 10 hip offsets from v0.4; not global optimum',
        'All 12 lower actuators use AK40 V3 bounding dimensions; actual mounting holes/shafts unverified',
        'Hip yaw/roll and ankle roll fixed zero; roll origin offset changes mechanics during real roll',
        'Sole sizing is packaging projection, not a modeled load-bearing shoe or tested contact polygon',
        'No brackets/cables/pelvis/upper body/feet-shell collisions, mass or dynamics',
        '81 discrete samples do not prove continuous clearance',
        '10mm motor-ground threshold and ROM are engineering assumptions']}
    (ROOT/'10_assembly/ankle_complete_results_v06.json').write_text(json.dumps(result,default=encode,indent=2)+'\n')
    print('tried',tried,'three_pose_accepted',len(accepted),'verified',len(verified),'rejections',reasons)
    if verified:print(json.dumps(verified[0]['candidate'],indent=2));print('minimum_ground_mm',min(s['motor_ground_clearance_m'] for s in verified[0]['states'])*1000)

if __name__=='__main__':main()

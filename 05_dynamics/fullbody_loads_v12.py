"""Mass-based task load matrix. Quasi-static/effective-acceleration screen, not inverse dynamics."""
import json,math,argparse
from pathlib import Path
import numpy as np
parser=argparse.ArgumentParser();parser.add_argument('--variant',choices=['v12','v13'],default='v12');args=parser.parse_args();VER=args.variant
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'05_dynamics'/('fullbody_'+VER);OUT.mkdir(exist_ok=True)
data=json.loads((ROOT/'09_cad'/('fullbody_'+VER)/'wholebody_results.json').read_text());poses={p['pose']:p for p in data['results']}
stand=poses['standing'];squat=poses['pickup_contact'];M=stand['with_costume']['mass_kg'];g=9.81
stroke=(stand['with_costume']['com_mm'][2]-squat['with_costume']['com_mm'][2])/1000
assert stroke>0
static=[]
for name,p in poses.items():
    joints=sorted(p['joint_loads'],key=lambda x:abs(x['quasistatic_required_Nm']),reverse=True)
    static.append({'pose':name,'mass_with_payload_kg':p['with_supported_payload']['mass_kg'],
        'contact_algebra':p['contact_reactions'],'ik_pass':p['ik']['arm_pose_pass'],
        'top_joint_loads':joints[:8],
        'leg_or_arm_main_over_1_3Nm':[x for x in joints if abs(x['quasistatic_required_Nm'])>1.3 and not x['joint'].endswith('_grip')],
        'not_verified':'contact geometry, collision-free trajectory, true actuator selection and limits'})
jumps=[];landings=[]
for h in (.05,.10,.15):
    v=math.sqrt(2*g*h);T=2*v/g
    force=M*g*(1+h/stroke);push_time=2*stroke/v
    torque=[{'joint':x['joint'],'effective_acceleration_screen_Nm':x['quasistatic_required_Nm']*(1+h/stroke)} for x in squat['joint_loads']]
    speeds=[]
    q0={x['name']:x['q_rad'] for x in squat['joints']};q1={x['name']:x['q_rad'] for x in stand['joints']}
    for name in q0:speeds.append({'joint':name,'mean_posture_change_speed_rad_s':abs(q1[name]-q0[name])/push_time})
    I=np.array(squat['with_costume']['inertia_about_com_kgm2'])
    rotation=[]
    for name,idx in [('front_flip',1),('back_flip',1),('side_flip',0)]:
        w=2*math.pi/T;L=I[idx,idx]*w
        rotation.append({'type':name,'assumed_fixed_posture_inertia_kgm2':float(I[idx,idx]),
            'mean_body_rotation_rad_s':w,'angular_momentum_Nms':float(L),
            'mean_launch_moment_Nm_if_generated_over_push':float(L/push_time)})
    jumps.append({'com_apex_gain_m':h,'reference_com_stroke_m':stroke,'launch_speed_mps':v,'push_duration_s':push_time,
        'flight_duration_s_same_com_height':T,'total_effective_vertical_force_N':force,'each_foot_N':force/2,
        'translation_energy_J':M*g*h,'ideal_push_work_J':M*g*(h+stroke),
        'mean_ideal_push_mechanical_power_W':M*g*(h+stroke)/push_time,
        'joint_moment_screen':torque,'mean_joint_speed_screen':speeds,'rotation_screen':rotation})
    for s in (.01,.03,.05):
        landings.append({'com_drop_m':h,'braking_com_stroke_m':s,'total_displacement_average_force_N':M*g*(1+h/s),
            'symmetric_each_foot_N':M*g*(1+h/s)/2,'equivalent_stopping_time_s':2*s/v})
grip=[]
for mu in (.2,.4,.6):
    for lever in (.010,.015):
        N=2*.58*g/(2*mu)
        grip.append({'assumed_mu':mu,'lever_m':lever,'assumed_efficiency':.7,'normal_force_per_hand_N':N,
                    'required_drive_torque_Nm':N*lever/.7,'force_factor':2})
out={'status':'conditional_load_screen_not_certification','wholebody_mass_kg':M,'reference_com_stroke_m':stroke,
     'static':static,'jump':jumps,'landing':landings,'grip':grip,
     'limits':['All mass and geometry inherit research assumptions and unresolved interference',
      'Quasi-static torque includes distal gravity and an assumed ground reaction distribution',
      'Get-up reactions assume palms at requested contacts even if IK misses; conditional sensitivity only',
      'Takeoff posture delta is not an optimized trajectory or maximum stroke',
      'Effective acceleration scales gravity loads with frozen geometry and omits articulated acceleration/centrifugal terms',
      'Mean speed and moment maxima cannot be treated as a simultaneous torque-speed operating point',
      'Constant-inertia full-turn-in-flight angular estimates are illustrative, not flip feasibility proofs',
      'Mat/wood/tile/table properties unmeasured; no ground-specific success claim']}
(OUT/'task_loads.json').write_text(json.dumps(out,indent=2)+'\n')
print('mass_kg',M,'COM_stroke_mm',stroke*1000)
for x in static:print(x['pose'],'peak_static',x['top_joint_loads'][0])
print('jump_h100mm',jumps[1]['each_foot_N'],'N per foot','push_s',jumps[1]['push_duration_s'])
print('landing_h100mm_s30mm',next(x for x in landings if x['com_drop_m']==.1 and x['braking_com_stroke_m']==.03))

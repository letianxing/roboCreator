"""Scoped validation of geometry, contact arithmetic and failed recovery paths."""
import json,hashlib,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def read(f):return json.loads((ROOT/f).read_text(encoding='utf-8'))
pf='03_system/shoulder_recovery_v19.json';h=hashlib.sha256((ROOT/pf).read_bytes()).hexdigest();sf='03_system/shoulder_selected_v19.json'
scan=read('09_cad/body_v19/shoulder_scan.json');assembly=read('09_cad/body_v19/assembly_results.json');phase=read('04_kinematics/recovery_v19/phase_results.json');balance=read('05_dynamics/support_v19/balance_results.json')
assert all(d['input_sha256']==h for d in [scan,assembly,phase,balance])
assert assembly['selected_input_sha256']==hashlib.sha256((ROOT/sf).read_bytes()).hexdigest()
assert scan['search_count']==384 and not scan['selected']['exact_collisions']
assert len(assembly['scenes'])==4
for r in assembly['scenes']:
    assert r['motor_count']==28 and not r['ground_penetrations']
    if r['name']=='alternating_handoff_midpoint':assert r['collisions']
    else:assert not r['collisions']
    assert not r['manufacturable'] and not r['task_verified']
    for ext in ('.FCStd','.step'):assert (ROOT/'09_cad/body_v19'/(r['name']+ext)).stat().st_size>100
assert max(r['known_or_allocated_mass_kg'] for r in assembly['scenes'])-min(r['known_or_allocated_mass_kg'] for r in assembly['scenes'])<1e-8
for r in balance['balances']:
    assert r['nonnegative_vertical_balance_exists'] and min(r['forces_N'])>=-1e-8
    assert abs(r['force_residual_N'])<1e-8 and max(abs(x) for x in r['moment_residual_Nm'])<1e-8
assert balance['actual_midpoint_static_balance']['nonnegative_vertical_balance_exists']
assert balance['handoff_geometry_violations']>0 and not balance['whole_recovery_verified']
bad=phase['first_fixed_hand_reach_failure'];assert bad and bad['shoulder_to_fixed_hand_mm']>bad['absolute_arm_reach_bound_mm']
phi=math.radians(phase['toe_start_pitch_deg']);assert abs(37*math.cos(phi)+55*math.sin(phi)-32)<1e-8
for row in phase['toe_transfer_probe']:
    # Reconstruct sole toe from stored foot transform; this verifies the contact anchor independently.
    t=row['leg_pose'];q=math.radians(t['foot_pitch_deg']);x=t['foot_translation_mm'][0]+55*math.cos(q)-2*math.sin(q);z=t['foot_translation_mm'][2]-55*math.sin(q)-2*math.cos(q)
    assert abs(x-phase['toe_anchor_mm'][0])<1e-8 and abs(z)<1e-8
record={'stage':'v0.19','checks':[
 {'name':'384 shoulder layout comparisons and exact shortlisted motor checks','status':'passed','evidence':['09_cad/body_v19/shoulder_scan.json']},
 {'name':'Four candidates saved/reopened with 28 drives; three static endpoints have no modeled common-volume/ground penetration','status':'passed','evidence':['09_cad/body_v19/assembly_results.json','09_cad/assemble_recovery_v19.py']},
 {'name':'Estimated mass consistency and nonnegative quasi-static vertical force/moment balance','status':'passed','evidence':['05_dynamics/support_v19/balance_results.json']},
 {'name':'Toe anchor consistency and retained negative controls for simultaneous handoff/fixed-hand transfer','status':'passed','evidence':['04_kinematics/recovery_v19/phase_results.json','05_dynamics/support_v19/balance_results.json']},
 {'name':'Complete continuous recovery trajectory','status':'failed','evidence':['04_kinematics/recovery_v19/phase_results.json','05_dynamics/support_v19/balance_results.json']},
 {'name':'Manufacturable shoulder/elbow interfaces, structural/thermal/impact CAE and real robot','status':'not_run','evidence':['07_mechanics/knee_pad_v19.md']}],
 'pending':['Use continuous body-pitch family and collision-aware alternating hand placement; replan foot transfer','Full shoulder gimbal/shaft/bearing and elbow clevis design','Actual knee pad material, fastening and load validation','Continuous collision/dynamics, torque-speed-duration and real measurements','Wukong appearance and remaining user tasks','PLM/MES implementation deferred by user until prototype completion']}
(ROOT/'11_verification/v19_checkpoint_checks.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('v19 scoped checks passed; full continuous recovery remains FAILED, engineering release NOT RUN.')

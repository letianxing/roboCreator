"""Evidence check: retains failure status for real assembly and wholebody tasks."""
import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def read(f):return json.loads((ROOT/f).read_text(encoding='utf-8'))
inp='03_system/single_leg_v17.json';h=hashlib.sha256((ROOT/inp).read_bytes()).hexdigest()
base=read('09_cad/single_leg_v17/integration_results.json');leg=read('09_cad/single_leg_v17/interface_revision/integration_results.json');arm=read('04_kinematics/gauntlet_contact_v17.json');replay=read('09_cad/gauntlet_v17/replay_results.json');audit=read('11_verification/v16_interface_audit_v17.json')
assert leg['input_sha256']==arm['input_sha256']==h
for source in audit['records']:
    assert source['source_sha256']==hashlib.sha256((ROOT/source['source']).read_bytes()).hexdigest()
assert len(leg['records'])==5
for r in leg['records']:
    assert len(r['joints'])==6 and sum(p['role']=='motor' for p in r['parts'])==6
    assert abs(sum(p['mass_kg'] for p in r['parts'] if p['mass_kg'] is not None)-r['known_mass_kg'])<1e-10
    assert not r['collisions'] and not r['manufacturable']
    for ext in ('.FCStd','.step'):assert (ROOT/'09_cad/single_leg_v17/interface_revision'/(r['pose']+ext)).stat().st_size>100
assert all(r['collisions'] for r in base['records'])
assert all(r['pad_translation_mm']<1e-10 and r['normal_change']<1e-10 for r in arm['wrist_authority_check'])
assert any(r['finite_search_pose_found'] for r in arm['records'])
assert any(not r['finite_search_pose_found'] for r in arm['records'])
assert all(r['pad_CAD_to_IK_error_mm']<1e-5 and not r['wholebody_verified'] for r in replay['records'])
for r in replay['records']:
    assert abs(min(r['pad_corner_height_mm'])-r['IK']['minimum_pad_corner_z_mm'])<1e-5
    if r['IK']['finite_search_pose_found'] and not r['has_ground_penetration_over_0_5mm']:
        assert max(abs(z) for z in r['pad_corner_height_mm'])<.01
gate={'stage':'v0.17','scope':'six-joint integration and independent pad placement review; no manufacturable assembly or getup release','checks':[
 {'name':'Five revised six-joint CAD/STEP configurations saved/reopened, link closure and known mass sums','status':'passed','evidence':['09_cad/single_leg_v17/interface_revision/integration_results.json','09_cad/integrate_single_leg_v17.py']},
 {'name':'v16 source hashes unchanged; every pair audited including earlier exclusions','status':'passed','evidence':['11_verification/v16_interface_audit_v17.json','09_cad/audit_v16_for_v17.py']},
 {'name':'Forearm pad is invariant to wrist joints; solved poses replay consistently in saved CAD','status':'passed','evidence':['04_kinematics/gauntlet_contact_v17.json','09_cad/gauntlet_v17/replay_results.json']},
 {'name':'Five revised sagittal configurations: all modeled pairs have no positive common volume','status':'passed','evidence':['09_cad/single_leg_v17/interface_revision/integration_results.json']},
 {'name':'Inherited v16 interface assembly','status':'failed','evidence':['09_cad/single_leg_v17/integration_results.json']},
 {'name':'Manufacturing release and continuous 3D clearance','status':'not_run','evidence':['09_cad/single_leg_v17/interface_revision/integration_results.json']},
 {'name':'Wholebody getup including contacts/dynamics/appearance','status':'not_run','evidence':['04_kinematics/gauntlet_contact_v17.json']}],
 'pending':['Validate revised bearing/shaft fits, vendor interfaces and physical joining of all rails/motors',
 'Reduce knee/hip lateral package after outboard placement; attach physical bearing and rail interfaces',
 '28 DOF shoulder-yaw candidate packing and mass; original 26 DOF failure is not global impossibility proof',
 'Couple low-body pad poses to knees/feet, support forces and full recovery path',
 'Complete dressed geometry, dynamics, electrical/thermal, cost and manufacturing validation']}
(ROOT/'11_verification/v17_checkpoint_checks.json').write_text(json.dumps(gate,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('Evidence checks complete: five revised discrete CAD configurations pass modeled-volume checks; manufacturing and wholebody getup NOT RUN.')

"""Evidence checks for numerical search, corrected geometry, exported CAD and scoped outcomes."""
import json,hashlib,sys
from pathlib import Path
import numpy as np
import FreeCAD as A
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'04_kinematics'))
from search_recovery_v21 import leg_fk
def read(p):return json.loads((ROOT/p).read_text(encoding='utf-8'))
pf=ROOT/'03_system/recovery_search_v21.json';digest=hashlib.sha256(pf.read_bytes()).hexdigest()
a=read('04_kinematics/recovery_v21/coupled_search.json');b=read('04_kinematics/recovery_v21/forward_extension.json')
assert a['input_sha256']==b['input_sha256']==digest
cases=a['cases']+b['cases'];assert len(cases)==384
valid=[r for r in cases if r['ik_found']];assert len(valid)==87
for r in valid:
    p,R=leg_fk(np.deg2rad(r['q_deg']),r['h_mm'],r['beta_deg'],r['hip_x_mm'])
    assert np.linalg.norm(p*1000-np.array(r['left_pose']['foot_translation_mm']))<.01
    assert np.linalg.norm(R-np.eye(3))<.001
reg=read('11_verification/v21_geometry_regression.json');assert len(reg['tests'])==4
assert all(max(r['local_shape_symmetric_difference_mm3'].values())<1e-4 for r in reg['tests'])
reuse=read('11_verification/v21_reuse_regression.json');assert reuse['conservative_arm_bounds_checked']
leg=read('09_cad/body_v21/leg_replan_screen.json');assert leg['completed'] and len(leg['cases'])==20
assert [r['id'] for r in leg['cases'] if r['leg_and_fixed_body_geometry_pass']]==[344,345]
arms_path='09_cad/body_v21/arm_screen.json'
for filename in ('arm_refined_screen.json','arm_midpitch_screen.json'):
    if (ROOT/'09_cad/body_v21'/filename).exists():arms_path='09_cad/body_v21/'+filename
if (ROOT/'09_cad/body_v21/arm_power_screen.json').exists():arms_path='09_cad/body_v21/arm_power_screen.json'
single='09_cad/body_v21/single_palm_screen.json'
if (ROOT/single).exists() and read(single)['selected']:arms_path=single
extended='09_cad/body_v21/single_palm_extended.json'
if (ROOT/extended).exists() and read(extended)['selected']:arms_path=extended
arms=read(arms_path);assert arms['completed']
for result in arms['selected']:
    assert result['motor_count']==28 and not result['all_modeled_collisions']
    balance=result['static_balance']
    assert balance['nonnegative_vertical_balance_exists'] and min(balance['forces_N'])>=-1e-8
    assert abs(balance['force_residual_N'])<1e-8 and max(map(abs,balance['moment_residual_Nm']))<1e-8
    assert abs(result['mass_kg']-4.072501811775055)<1e-8
power=read('09_cad/body_v21/leg_power_screen.json')
assert power['completed'] and [r['id'] for r in power['cases'] if r['leg_and_fixed_body_geometry_pass']]==[544]
lift=read('09_cad/body_v21/local_lift.json');assert not lift['full_recovery_validated']
assert lift.get('complete_probe') and len(lift['samples'])==13
assert [r['h_mm'] for r in lift['samples'] if r['sample_geometry_and_static_pass']]==[120]
assert not lift['adjacent_sample_joint_changes']
margin=read('05_dynamics/support_v21/contact_margins.json')
assert abs(margin['cases'][0]['support_polygon_margin_mm']-1.3660895691672863)<1e-5
assert not margin['cases'][0]['right_palm_at_least_1N_feasible']
exports=read('09_cad/body_v21/review_exports.json');cad=[]
assert exports['names'],'No CAD witness exported'
for file in sorted((ROOT/'09_cad/body_v21').glob('*.FCStd')):
    doc=A.openDocument(str(file));shapes=[o.Shape for o in doc.Objects if hasattr(o,'Shape')]
    assert len(shapes)==82 and all(s.isValid() for s in shapes)
    cad.append({'file':str(file.relative_to(ROOT)),'valid_shapes':len(shapes),'sha256':hashlib.sha256(file.read_bytes()).hexdigest()});A.closeDocument(doc.Name)
    assert file.with_suffix('.step').stat().st_size>100
(ROOT/'11_verification/v21_cad_reopen.json').write_text(json.dumps(cad,indent=2)+'\n',encoding='utf-8')
def ck(n,status,e):return {'name':n,'status':status,'evidence':e}
checks=[ck('Thigh-local rail and knee-pad covariance; sagittal preservation','passed',['11_verification/v21_geometry_regression.json']),
 ck('Rigid part reuse and conservative arm bounds regression','passed',['11_verification/v21_reuse_regression.json']),
 ck('384 bounded numerical cases, 87 independently FK-rechecked solutions','passed',['04_kinematics/recovery_v21/coupled_search.json','04_kinematics/recovery_v21/forward_extension.json']),
 ck('20 knee120 replans, two partial leg/fixed-body geometry passes, failures retained','passed',['09_cad/body_v21/leg_replan_screen.json']),
 ck('Native CAD reopened with 82 valid shapes; STEP files exist','passed',['11_verification/v21_cad_reopen.json']),
 ck('Whole-body static geometry and nonnegative vertical support','passed' if arms['selected'] else 'failed',[arms_path]),
 ck('13 local fixed-contact probe samples and support-margin diagnostics retained','passed',['09_cad/body_v21/local_lift.json','05_dynamics/support_v21/contact_margins.json']),
 ck('Adjacent clear samples in the explored local lift family','passed' if lift['adjacent_sample_joint_changes'] else 'failed',['09_cad/body_v21/local_lift.json']),
 ck('Complete recovery trajectory, dynamic contact and control','not_run',['09_cad/body_v21/local_lift.json']),
 ck('Manufacturing, current structural CAE, real robot and Wukong appearance release','not_run',['deliverables/recovery_review_v21.md'])]
out={'stage':'v0.21','checks':checks,'pending':['Repack pelvis/battery/power over a motion interval and increase support margin','Connect earlier cuff contact to planted-foot phase with collision-aware contact switches','Actual shoulder/elbow interfaces and full-load structural CAE','Continuous collision/dynamics and torque-speed-duration checks','Manufacturing/real tests and approved dressed Wukong appearance','PLM/MES deferred by user']}
(ROOT/'11_verification/v21_checkpoint_checks.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
print('v21 evidence checks passed. Engineering and full recovery remain unvalidated.')

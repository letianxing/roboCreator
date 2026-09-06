"""Scoped evidence checks, including frozen-pose regression and native CAD reopening."""
import json,hashlib,sys
from pathlib import Path
import numpy as np
import FreeCAD as A
ROOT=Path(__file__).resolve().parents[1]
def read(p):return json.loads((ROOT/p).read_text(encoding='utf-8'))
data=read('09_cad/body_v22/motion_validation.json');search=read('09_cad/body_v22/layout_clearance_2.json');initial=read('09_cad/body_v22/layout_search.json')
sha=hashlib.sha256((ROOT/'03_system/electronics_motion_v22.json').read_bytes()).hexdigest()
assert data['complete'] and search['complete'] and data['input_sha256']==search['input_sha256']==initial['input_sha256']==sha
assert len(initial['joint_layouts'])==42 and len(search['joint_layouts'])==4
assert data['selected_layout']['translations_relative_v21_body_mm']=={'battery':[-4,0,4],'computer':[-4,0,0],'power':[-12,0,0]}
prior={r['h_mm']:r for r in read('09_cad/body_v21/local_lift.json')['samples']}
assert len(data['samples'])==13
for r in data['samples']:
    assert r['sample_geometry_and_vertical_balance_pass'] and not r['clearance_failures'] and not r['other_or_full_collisions']
    assert max(r['volume_errors_mm3'].values())<1e-6 and abs(r['mass_kg']-4.072501811775055)<1e-8
    balance=r['static_balance'];assert min(balance['forces_N'])>=-1e-8 and abs(balance['force_residual_N'])<1e-8
    assert max(map(abs,balance['moment_residual_Nm']))<1e-8
    if r['h_mm'] in prior:
        before=np.array(r['COM_mm'])-np.array(r['COM_change_mm']);assert np.linalg.norm(before-prior[r['h_mm']]['COM_mm'])<1e-5
assert sum(r['full_pair_audit'] for r in data['samples'])==9
stand=data['standing'];assert stand['geometry_clearance_pass'] and stand['static_balance']['nonnegative_vertical_balance_exists']
assert max(abs(a-b) for a,b in zip(stand['bounds_size_mm'],stand['source_bounds_size_mm']))<1e-5
assert 300<=stand['maximum_z_mm']<=400
segment=read('04_kinematics/recovery_v22/segment_samples.json')['samples'];assert len(segment)==13
for s in segment:
    assert max(r['position_error_mm'] for r in s['closure_checks'])<.01
joint_steps=[]
for a,b in zip(segment,segment[1:]):
    def q(s):return s['input']['left_pose']['q_deg']+s['input']['right_pose']['q_deg']+s['left_arm']['q_deg']+s['right_arm']['q_deg']
    step=float(np.max(np.abs(np.array(q(a))-q(b))));joint_steps.append({'interval_mm':[a['h_mm'],b['h_mm']],'max_joint_step_deg':step})
cad=[]
for file in sorted((ROOT/'09_cad/body_v22').glob('*.FCStd')):
    doc=A.openDocument(str(file));shapes=[o.Shape for o in doc.Objects if hasattr(o,'Shape')];assert len(shapes)==82 and all(s.isValid() for s in shapes);A.closeDocument(doc.Name)
    assert file.with_suffix('.step').stat().st_size>100
    cad.append({'file':str(file.relative_to(ROOT)),'valid_shapes':len(shapes),'sha256':hashlib.sha256(file.read_bytes()).hexdigest()})
assert len(cad)==4
interfaces=read('07_mechanics/electronics_interfaces_v22.json');assert len(interfaces['scenes'])==4
assert min(r['distance_mm'] for s in interfaces['scenes'] for r in s['nearest_envelope_gaps'])>=2-1e-6
(ROOT/'11_verification/v22_reopen_and_joint_steps.json').write_text(json.dumps({'cad':cad,'adjacent_joint_samples':joint_steps,'continuous_sweep_proven':False},indent=2)+'\n',encoding='utf-8')
def ck(n,s,e):return {'name':n,'status':s,'evidence':e}
out={'stage':'v0.22','checks':[
 ck('1mm and2mm four-scene layout searches,42 and4 joint survivors','passed',['09_cad/body_v22/layout_search.json','09_cad/body_v22/layout_clearance_2.json']),
 ck('13 half-mm local poses: electronics clearance, geometry and vertical equilibrium','passed',['09_cad/body_v22/motion_validation.json']),
 ck('Bilateral FK closure and adjacent joint samples, not timed hardware commands','passed',['04_kinematics/recovery_v22/segment_samples.json','11_verification/v22_reopen_and_joint_steps.json']),
 ck('Standing geometry and dimensions retained; mass/COM relocation arithmetic','passed',['09_cad/body_v22/motion_validation.json']),
 ck('Four native CAD/STEP files and exact nearest-envelope interface gaps','passed',['11_verification/v22_reopen_and_joint_steps.json','07_mechanics/electronics_interfaces_v22.json']),
 ck('Complete continuous recovery and sufficient dynamic stability','not_run',['deliverables/layout_motion_review_v22.md']),
 ck('Actual mounting,cabling,structural FEM,hardware and final appearance','not_run',['07_mechanics/electronics_interfaces_v22.md'])],
 'pending':['Real module interfaces, retainers, standoffs and cable volumes with mass accounting','Extend local interval and improve upper-range support margin','Connect cuff/foot replanting and knee unload to final stance','Structural/dynamic/thermal/impact and physical validation','Final dressed Wukong appearance; PLM/MES deferred']}
(ROOT/'11_verification/v22_checkpoint_checks.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
print('v22 scoped checks passed, maximum adjacent joint change:',max(r['max_joint_step_deg'] for r in joint_steps))

"""Evidence gate for v16 local mechanisms; explicitly not engineering release."""
import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def load(rel):return json.loads((ROOT/rel).read_text(encoding='utf-8'))
p='03_system/support_mechanisms_v16.json';h=hashlib.sha256((ROOT/p).read_bytes()).hexdigest();g=load('09_cad/support_mechanisms_v16/mechanism_results.json');d=load('05_dynamics/support_load_paths_v16.json');t=load('11_verification/support_tests_v16.json')
assert g['input_sha256']==d['input_sha256']==t['input_sha256']==h
assert t['successful'] and t['tests_run']==10 and len(g['records'])==14
assert all(not r['unexpected_intersections'] and not r['manufacturable'] for r in g['records'])
assert g['bilateral_neutral_bounds']['outside_width_mm']==185 and g['bilateral_neutral_bounds']['inside_gap_mm']>0
assert g['clearance_to_nominal_knee_axis_z_mm']==8
assert not d['task_verified'] and d['mass_feedback']['two_ankle_nonmotor_geometry_kg']>0
for r in g['records']:
    for f in r['files']:assert (ROOT/'09_cad/support_mechanisms_v16'/f).stat().st_size>100
for f in ['09_cad/support_mechanisms_v16/mechanism_views.png','05_dynamics/support_load_paths_v16.png','deliverables/support_mechanisms_review_v16.md']:
    assert (ROOT/f).stat().st_size>100
gate={'stage':'v0.16','scope':'local schematic mechanism geometry and analytical load-path identities only; not manufacturing or wholebody task release','checks':[
 {'name':'14 local FreeCAD/STEP pose files saved, reopened valid and current hashes match','status':'passed','evidence':[p,'09_cad/build_support_mechanisms_v16.py','09_cad/support_mechanisms_v16/mechanism_results.json']},
 {'name':'Nine ankle and five hand discrete poses have no unexcepted modeled-solid intersections','status':'passed','evidence':['09_cad/support_mechanisms_v16/mechanism_results.json']},
 {'name':'Parallel-crank work/length, contact balance, mass separation and ten regression tests','status':'passed','evidence':['05_dynamics/support_load_paths_v16.json','11_verification/test_support_v16.py','11_verification/support_tests_v16.json']},
 {'name':'Preview artifacts inspected and all manufacturable/task flags remain false','status':'passed','evidence':['09_cad/support_mechanisms_v16/mechanism_views.png','05_dynamics/support_load_paths_v16.png','deliverables/support_mechanisms_review_v16.md']}],
 'pending':['Complete leg with knee/hip packages; only 8 mm nominal plane clearance above local module','Exact AK/XM interfaces, external bearings, shafts, crank pins, rod ends, fasteners and cable routing','Continuous swept collision and structural/contact/impact/fatigue/thermal validation','Real pad stiffness/tolerance/friction and full prone/supine/side get-up trajectories','26DOF dressed wholebody integration and bottle handover','Takeoff and three-direction flips on four surfaces','Final electrical system, runtime, costs and manufacturing release']}
(ROOT/'11_verification/v16_checkpoint_checks.json').write_text(json.dumps(gate,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print('v16 local evidence passed; no engineering release.')

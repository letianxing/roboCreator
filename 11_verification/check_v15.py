"""Validate evidence/provenance then prepare snapshot gates; no engineering release."""
import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def read(path):return json.loads((ROOT/path).read_text(encoding='utf-8'))
p='03_system/layout_load_inputs_v15.json';h=hashlib.sha256((ROOT/p).read_bytes()).hexdigest()
geo=read('09_cad/modules_v15/packaging_results.json');dyn=read('05_dynamics/trajectory_v15/summary.json');tests=read('11_verification/trajectory_tests_v15.json')
assert all(r['input_sha256']==h for r in [geo,dyn,tests])
assert tests['successful'] and tests['tests_run']==8
assert geo['vendor_xm_valid'] and geo['vendor_xm_solid_count']==18
assert len(geo['modules'])==4
for module in geo['modules']:
    assert len(module['all_trials'])==module['search_count']==300
    assert module['sample_count_each']==5
    assert module['selected']['sampled_motor_clearance_pass']
    assert not module['manufacturable']
    for suffix in ['.FCStd','_envelopes.step']:
        assert (ROOT/'09_cad/modules_v15'/(module['module']+suffix)).stat().st_size>100
assert len(dyn['cases'])==12
assert all(abs(c['work_energy_residual_J'])<1e-7 and not c['task_verified'] for c in dyn['cases'])
assert any(j['sampled_duration_over_no_load_speed_s']>0 for c in dyn['cases'] for j in c['joint_summaries'])
for path in ['09_cad/modules_v15/module_comparison.png','05_dynamics/trajectory_v15/load_curves.png','deliverables/layout_load_review_v15.md']:
    assert (ROOT/path).stat().st_size>100
gate={'stage':'v0.15','scope':'local motor packaging and conditional planar mathematics, not wholebody/engineering release','checks':[
 {'name':'Manufacturer STEP import, 4 local CAD save/reopen and 1200 layout records','status':'passed','evidence':['09_cad/module_packaging_v15.py','09_cad/modules_v15/packaging_results.json']},
 {'name':'Selected two-motor envelopes meet specified clearance at five roll samples only','status':'passed','evidence':['09_cad/modules_v15/packaging_results.json']},
 {'name':'12 planar histories energy checks and 8 numerical implementation tests','status':'passed','evidence':['05_dynamics/trajectory_v15/summary.json','11_verification/trajectory_tests_v15.json','11_verification/test_trajectory_v15.py']},
 {'name':'Current input hashes, artifact presence and preservation of speed-limit failures','status':'passed','evidence':[p,'11_verification/check_v15.py','deliverables/layout_load_review_v15.md']}],
 'pending':['Whole leg/arm physical mounting, shafts/bearings/transmission and mass','B ankle width/axis offsets and upgraded wrist packaging','All 26DOF trajectories with costume, bottle and contact constraints',
 'Real torque-speed-current-duration-temperature validation including reflected inertia/friction','Takeoff, three-direction flips and four-surface landing','Wholebody safety, appearance approval, 20min runtime and real tests','Final costs and manufacturing release']}
(ROOT/'11_verification/v15_checkpoint_checks.json').write_text(json.dumps(gate,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('v15 evidence gates passed; engineering release remains pending.')

"""Validate v18 evidence without converting failed assemblies into released designs."""
import json,hashlib,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def read(f):return json.loads((ROOT/f).read_text(encoding='utf-8'))
cfg=read('03_system/body_integration_v18.json');c=read('04_kinematics/body_contacts_v18.json');body=read('09_cad/body_v18/body_results.json');repack=read('09_cad/body_v18/electronics_repack_results.json')
h=hashlib.sha256((ROOT/'03_system/body_integration_v18.json').read_bytes()).hexdigest();assert c['input_sha256']==body['input_sha256']==h
assert repack['input_sha256']==hashlib.sha256((ROOT/'03_system/electronics_repack_v18.json').read_bytes()).hexdigest()
beta=c['arm_candidate']['body_pitch_deg'];analytic=37+90*math.cos(math.radians(35))+90*math.cos(math.radians(beta-160))
assert abs(analytic-c['flat_feet']['minimum_body_height_mm'])<1e-8
assert c['flat_feet']['height_gap_mm']>30 and not c['flat_feet']['compatible_in_grid']
assert len(c['kneeling_candidates'])==len(body['knee_contact_trials'])==50
assert all(s['motor_count']==28 and not s['task_verified'] and not s['manufacturable'] and s['collisions'] for s in body['scenes'])
for s in body['scenes']:
    for ext in ('.FCStd','.step'):assert (ROOT/'09_cad/body_v18'/(s['name']+ext)).stat().st_size>100
scene=body['scenes'][2]
for n in scene['arm_nodes']:
    assert abs(n['pad_center_mm'][2])<.01 and abs(n['pad_center_mm'][0]-150)<.01 and abs(abs(n['pad_center_mm'][1])-125)<.01
for v in repack['volume_checks']:assert abs(v['old_volume_mm3']-v['new_volume_mm3'])<1e-6
assert not [x for x in repack['collisions'] if 'electronics' in (x['group_a'],x['group_b'])]
assert repack['collisions'] and not repack['task_verified']
manifest=read('26_plm/artifact_manifest_v18.json')
for row in manifest['artifacts']:assert hashlib.sha256((ROOT/row['path']).read_bytes()).hexdigest()==row['sha256']
assert read('11_verification/lifecycle_checks_v18.json')['reference_and_hash_checks_passed']
assert not manifest['engineering_release'] and not manifest['mes_dispatched']
gate={'stage':'v0.18','scope':'Integrated packaging, geometry/contact constraints and local lifecycle records; not engineering release','checks':[
 {'name':'Four integrated CAD/STEP studies saved/reopened; base scenes count 28 motors','status':'passed','evidence':['09_cad/body_v18/body_results.json','09_cad/body_v18/electronics_repack_results.json']},
 {'name':'Flat-foot analytic height bound and 50 knee-contact candidate records','status':'passed','evidence':['04_kinematics/body_contacts_v18.json','09_cad/body_v18/body_results.json']},
 {'name':'Seven rigid relocations preserve volume; modeled electronics-related intersections eliminated in standing candidate','status':'passed','evidence':['09_cad/body_v18/electronics_repack_results.json']},
 {'name':'PLM draft references/hashes and undispatched MES template validated; software permission rule saved','status':'passed','evidence':['26_plm/artifact_manifest_v18.json','11_verification/lifecycle_checks_v18.json','tools/software_policy.md']},
 {'name':'Fullbody interference clearance','status':'failed','evidence':['09_cad/body_v18/body_results.json','09_cad/body_v18/electronics_repack_results.json']},
 {'name':'CAE engineering validation and actual MES production','status':'not_run','evidence':['11_verification/cae_plan_v18.json','27_mes/work_order_template.json']}],
 'pending':['Shoulder/arm versus leg interference and remaining assembly interfaces','Actual protective knee contact and connected recovery trajectory','Fullbody mass/inertia, stability, contact/thermal/impact CAE and physical validation','Approved Wukong appearance and all user tasks','Released EBOM/MBOM, supplier-specific manufacturing plan and real inspection data','Ask before introducing open-source software or new/configured MCP']}
(ROOT/'11_verification/v18_checkpoint_checks.json').write_text(json.dumps(gate,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('v18 scoped evidence verified. Wholebody interference FAILED; CAE/production not run.')

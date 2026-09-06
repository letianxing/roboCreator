"""Verify local PLM/MES references and generate a scoped artifact hash manifest."""
import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def read(path):return json.loads((ROOT/path).read_text(encoding='utf-8'))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
registry=read('26_plm/registry.json');mes=read('27_mes/work_order_template.json');cae=read('11_verification/cae_plan_v18.json')
ids=[r['id'] for k in ('requirements','studies','changes') for r in registry[k]]
assert len(ids)==len(set(ids))
requirements={r['id'] for r in registry['requirements']}
for case in cae['cases']:
    assert set(case['requirements'])<=requirements
    assert case['status']=='not_run'
    if case.get('existing_related_evidence'):assert (ROOT/case['existing_related_evidence']).is_file()
for study in registry['studies']:assert (ROOT/study['artifact']).is_file()
assert not registry['releases'] and not registry['as_built_units']
assert mes['template'] and mes['status']=='draft_not_dispatched' and mes['release_id'] is None and mes['unit_serial_number'] is None
assert all(o['status']=='not_started' for o in mes['operations'])
artifacts=[
 'prompts/robot_master_prompt.md','03_system/body_integration_v18.json','04_kinematics/body_contacts_v18.py',
 '04_kinematics/body_contacts_v18.json','09_cad/leg_reuse_v18.py','09_cad/body_integration_v18.py','09_cad/body_v18/body_results.json',
 '09_cad/body_v18/stand_baseline.FCStd','09_cad/body_v18/stand_narrow.FCStd','09_cad/body_v18/kneeling_contact_probe.FCStd',
 '03_system/electronics_repack_v18.json','09_cad/repack_electronics_v18.py','09_cad/body_v18/electronics_repack_results.json',
 '09_cad/body_v18/stand_electronics_repack.FCStd','tools/software_policy.md',
 'deliverables/body_review_v18.md','26_plm/lifecycle_workflow.md','26_plm/registry.json','11_verification/cae_plan_v18.json','27_mes/work_order_template.json']
rows=[]
for i,rel in enumerate(artifacts):
    path=(ROOT/rel).resolve();assert path.is_relative_to(ROOT) and path.is_file()
    rows.append({'artifact_id':f'RC-ART-{1800+i:04d}','study_id':'RC-STUDY-0018','path':rel,'sha256':sha(path),'bytes':path.stat().st_size,'status':'draft'})
manifest={'version':'v0.18','scope':'Current study artifacts and workflow records only; full backup is in checkpoints',
          'artifacts':rows,'engineering_release':False,'mes_dispatched':False}
path=ROOT/'26_plm/artifact_manifest_v18.json';path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
for row in read('26_plm/artifact_manifest_v18.json')['artifacts']:assert sha(ROOT/row['path'])==row['sha256']
(ROOT/'11_verification/lifecycle_checks_v18.json').write_text(json.dumps({'reference_and_hash_checks_passed':True,'artifact_count':len(rows),
 'enterprise_platform_deployed':False,'engineering_release':False,'manufacturing_work_dispatched':False},indent=2)+'\n')
print('Local lifecycle references and',len(rows),'artifact hashes verified; no release/work order dispatched.')

"""Audit integration evidence, mass replacement, nominal contacts and received-photo provenance."""
import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def read(p):return json.loads((ROOT/p).read_text(encoding='utf-8'))
def sha(p):return hashlib.sha256((ROOT/p).read_bytes()).hexdigest()
original=read('09_cad/body_v23_integration/integration.json');revision=read('09_cad/body_v23_revision/integration.json');final=read('09_cad/body_v23_fit/integration.json')
assert original['completed'] and len(original['scenes'])==4
assert all(not r['geometry_pass'] for r in original['scenes'])
for data in (revision,final):
    assert data['completed'] and len(data['scenes'])==14
    assert all(r['geometry_pass'] and not r['changed_pair_failures'] and not r['ground_penetrations_mm'] for r in data['scenes'])
assert final['source_geometry_evidence_sha256']==sha(final['unchanged_pair_evidence'])
assert final['source_motion_sha256']==sha('04_kinematics/recovery_v22/segment_samples.json')
assert final['carrier_input_sha256']==sha('03_system/electronics_carrier_v23.json')
assert final['revision_input_sha256']==sha('03_system/carrier_revision_v23.json')
assert final['fit_input_sha256']==sha('03_system/carrier_fit_v23.json')
for r in final['scenes']:
    assert r['component_count']==101 and r['motor_count']==28
    assert abs(r['new_mass_kg']-(r['old_mass_kg']-r['removed_plate_mass_kg']+r['added_native_mass_kg']))<1e-8
    f=r['vertical_balance'];assert f['nonnegative_vertical_balance_exists'] and min(f['forces_N'])>=-1e-8
    assert abs(f['force_residual_N'])<1e-8 and max(map(abs,f['moment_residual_Nm']))<1e-8
    assert not r['manufacturing_release']
interfaces=read('07_mechanics/carrier_interfaces_v23.json');assert len(interfaces['scenes'])==4
for r in interfaces['scenes']:
    assert r['part_count']==r['valid_shape_count']==101
    assert r['file_sha256']==sha('09_cad/body_v23_fit/'+r['scene']+'.FCStd')
    assert (ROOT/'09_cad/body_v23_fit'/(r['scene']+'.step')).stat().st_size>100
    seat=next(c for c in r['checks'] if c['a']=='ElectronicsCarrier' and c['b']=='CarrierScrew0');assert seat['distance_mm']<1e-6
    cable=next(c for c in r['checks'] if c['a']=='HarnessGrommet');assert cable['distance_mm']>.4
receipt=read('08_electrical/sipeed_receipt_20260907.json');assert receipt['user_reported_arrived'] and not receipt['uac_board_in_current_cad']
for row in [receipt['array_photo'],receipt['driver_package_photo']]+receipt['downloaded_pdf']:assert sha(row['file'])==row['sha256']
def ck(name,status,evidence):return {'name':name,'status':status,'evidence':evidence}
out={'stage':'v0.23-integrated','checks':[
 ck('Original failed assemblies retained and two revised evidence chains checked','passed',['09_cad/body_v23_integration/integration.json','09_cad/body_v23_revision/integration.json','09_cad/body_v23_fit/integration.json']),
 ck('13 reference local poses and standing: changed-pair geometry,ground and vertical balance','passed',['09_cad/body_v23_fit/integration.json']),
 ck('101 CAD objects,28 drives,mass replacement and four saved valid CAD/STEP assemblies','passed',['07_mechanics/carrier_interfaces_v23.json','09_cad/body_v23_fit/integration.json']),
 ck('Nominal seat/support contacts and limited cable/thigh/reference-microphone clearances measured','passed',['07_mechanics/carrier_interfaces_v23.json']),
 ck('Arrival photos and manufacturer PDF files archived with verified hashes','passed',['08_electrical/sipeed_receipt_20260907.json']),
 ck('Actual UAC board installation and measured array interfaces','not_run',['08_electrical/sipeed_arrival_review.md']),
 ck('Manufacturing,tolerance/preload/strength,full recovery,dynamics and final Wukong appearance','not_run',['deliverables/carrier_review_v23.md'])],
 'pending':['UAC bare-board identity,dimensions,connectors and mass; match actual array to drawings','Include UAC and actual FPC/USB wiring; revise mounting using measured interfaces','Resolve small clearances and high-end support margin before release','Thread/preload,material/thermal/impact and structural CAE','Full recovery and dressed Wukong validation; PLM/MES deferred']}
(ROOT/'11_verification/v23_integration_checks.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
print('v23 integrated reference checks passed; actual audio-module mounting and engineering release remain open.')

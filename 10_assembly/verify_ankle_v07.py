"""Regression checks for corrected units/support and persisted CAD screening results."""
import json,math,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
x=json.loads((ROOT/'10_assembly/ankle_foot_results_v06_corrected.json').read_text())
for row in x['configs']:
    foot=next(b for b in row['proxy_boxes'] if b['name']=='foot_shell')
    assert math.isclose(2*foot['half'][0],row['foot_length_m'])
    assert math.isclose(2*foot['half'][1],row['foot_width_m'])
    assert math.isclose(foot['center'][2]-foot['half'][2],0,abs_tol=1e-12)
    assert math.isclose(foot['center'][2]+foot['half'][2],row['sole_mm']/1000)
    assert row['single_support_margin_com_at_robot_center_m']<0
    assert row['single_support_margin_com_over_foot_center_m']>0
    assert row['strict_penetrations']==[['ankle_pitch','ankle_roll']]
source=ROOT/'00_inputs/actuators/AK40_V3/model/AK40-10 V3.0 KV170.STEP'
for folder in ['ankle_v07','ankle_v07_axial']:
    d=ROOT/'09_cad'/folder
    r=json.loads((d/'verification.json').read_text())
    assert r['source_sha256']==hashlib.sha256(source.read_bytes()).hexdigest()
    assert r['source_geometry']['solids']==0
    assert len(r['samples'])==63 and not any(s['clashes'] for s in r['samples'])
    assert r['max_common_volume_mm3']<1e-5
    assert (d/'wukong_ankle_v07.FCStd').stat().st_size>100000
    assert (d/'wukong_ankle_v07.step').stat().st_size>100000
axial=json.loads((ROOT/'09_cad/ankle_v07_axial/verification.json').read_text())
assert axial['minimum_distances']['foot__pitch_envelope']['distance_mm']>2
assert math.isclose(axial['minimum_distances']['roll_envelope__pitch_envelope']['distance_mm'],13.4,abs_tol=1e-6)
print('PASS: v06 unit/support regressions; vendor hash; 126 pose records; saved FCStd/STEP; axial clearances.')

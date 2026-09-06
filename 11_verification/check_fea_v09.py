"""Numerical equilibrium checks, not structural approval."""
import json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
for folder in ['yoke_fea_v09_radial','yoke_fea_reinforced_v09_radial']:
    for h in ['1.2','0.8']:
        r=json.loads((ROOT/'11_verification'/folder/('mesh_'+h)/'result_summary.json').read_text())
        assert r['bearing_constraint']=='radial'
        assert r['force_balance_residual_N']<.001
        assert r['moment_balance_residual_Nmm']<.01
        assert r['energy_relative_error']<1e-6
        assert r['max_displacement_mm']>0 and r['peak_von_mises_MPa']>0
c=json.loads((ROOT/'11_verification/yoke_fea_reinforced_v09/connection_load_screen.json').read_text())
bolts=c['bolt_group_combined_screen']['bolts']
assert abs(sum(x['shear_y_N'] for x in bolts))<1e-8
assert abs(sum(x['shear_z_N'] for x in bolts)+100)<1e-8
assert abs(sum(x['y_mm']*x['shear_z_N']-x['z_mm']*x['shear_y_N'] for x in bolts)-4100)<1e-7
for x in c['ideal_pin_bearing_cases']:
    assert abs(x['internal_reaction_N']+x['external_reaction_N']-100)<1e-8
geom=json.loads((ROOT/'09_cad/ankle_v09/verification.json').read_text())
assert geom['penetration_frame_count']==0 and geom['sample_count']==35
print('PASS: four corrected FEA cases satisfy force/moment/energy checks; bolt-group equilibrium; 35 geometry samples.')
print('NOT PASSED: mesh convergence, physical contacts, strength, fatigue, manufacturing release.')

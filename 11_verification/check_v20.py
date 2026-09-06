"""Validate saved v20 evidence, reopen native CAD, and preserve failed engineering gates."""
import hashlib,json
from pathlib import Path
import FreeCAD as A
ROOT=Path(__file__).resolve().parents[1]
def read(p):return json.loads((ROOT/p).read_text(encoding='utf-8'))
path='09_cad/body_v20/contact_validation.json';r=read(path)
assert r['input_sha256']==hashlib.sha256((ROOT/'03_system/contact_route_v20.json').read_bytes()).hexdigest()
assert len(r['toe_steps'])==11
for row in r['toe_steps']:
    assert row['sampled_geometry_pass'] and not row['collisions']
    assert abs(row['front_edge_predicted_height_mm']-row['CAD_minimum_moving_z_mm'])<.001
    assert abs(row['mass_kg']-4.072501811775056)<1e-8
    b=row['retained_cuff_knee_support']
    assert b['nonnegative_vertical_balance_exists'] and min(b['forces_N'])>=0
    assert abs(b['force_residual_N'])<1e-8 and max(map(abs,b['moment_residual_Nm']))<1e-8
b=r['left_cuff_unload_vertical_balance']
assert b['nonnegative_vertical_balance_exists'] and min(b['forces_N'])>=0
assert abs(b['force_residual_N'])<1e-8 and max(map(abs,b['moment_residual_Nm']))<1e-8
assert abs(r['toe_steps'][-1]['toe_center_mm'][2])<.001
assert len(r['asymmetric_endpoint']['collisions'])==24
assert not r['whole_recovery_validated']
audit=read('04_kinematics/recovery_v20/ik_audit.json')
assert audit['FK_positive_control_error_mm']<1e-8 and audit['IK_positive_control']['ik_found']
assert sum(x['ik_found'] for x in audit['additional_cases'])==2
assert not any(x['ik_found'] for x in read('04_kinematics/recovery_v20/asymmetric_ik_search.json')['cases'])
cad=[]
for name in ('toe_engage_00','toe_engage_05','toe_engage_10','asymmetric_foot_candidate'):
    file=ROOT/'09_cad/body_v20'/f'{name}.FCStd'
    doc=A.openDocument(str(file))
    shapes=[o.Shape for o in doc.Objects if hasattr(o,'Shape') and not o.Shape.isNull()]
    assert shapes and all(s.isValid() for s in shapes),name
    cad.append({'file':str(file.relative_to(ROOT)),'valid_shape_count':len(shapes),'sha256':hashlib.sha256(file.read_bytes()).hexdigest()})
    A.closeDocument(doc.Name)
    assert file.with_suffix('.step').stat().st_size>100
    print('Reopened valid CAD:',name,len(shapes),flush=True)
(ROOT/'11_verification/v20_cad_reopen.json').write_text(json.dumps(cad,indent=2)+'\n',encoding='utf-8')
def check(name,status,evidence):return dict(name=name,status=status,evidence=evidence)
out={'stage':'v0.20','checks':[
 check('11 sampled wrist-to-palm-edge geometries and analytic/CAD height agreement','passed',[path]),
 check('Quasi-static nonnegative vertical equilibrium; not verified force transfer','passed',[path]),
 check('Four native CAD files reopen with valid shapes; STEP files exist','passed',['11_verification/v20_cad_reopen.json']),
 check('Six-axis FK/IK positive control and retained failed search evidence','passed',['04_kinematics/recovery_v20/ik_audit.json','04_kinematics/recovery_v20/asymmetric_ik_search.json']),
 check('Existing FreeCAD FEM, Gmsh, CalculiX and MCP availability only','passed',['tools/freecad_cae_stack_v20.json']),
 check('Asymmetric planted-foot candidate collision check','failed',[path]),
 check('Complete continuous recovery trajectory','failed',[path,'04_kinematics/recovery_v19/phase_results.json']),
 check('New v20 structural FEM, dynamic contact, hardware and Wukong appearance approval','not_run',['deliverables/contact_review_v20.md'])],
 'pending':['Coupled body pose and foot placement search with full-body collisions',
 'Replant/unload hand contact with friction and compliance; not just vertical force allocation',
 'Actual shoulder/elbow mounting, shaft/bearing interfaces and load paths',
 'FreeCAD FEM/Gmsh/CalculiX structural analysis after geometry/load conditions are defined',
 'Torque-speed-duration, impact/thermal/material/physical tests and Wukong appearance',
 'PLM/MES implementation deferred until prototype completion']}
(ROOT/'11_verification/v20_checkpoint_checks.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
print('Scoped checks passed; full recovery remains FAILED; new structural FEM NOT RUN.')

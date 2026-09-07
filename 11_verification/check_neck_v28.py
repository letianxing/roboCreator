"""Evidence gate for the bounded neck study; failed routes remain failed."""
import json,math,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'09_cad'))
from neck_chain_v28 import A,Part,V,around,sha
OUT=ROOT/'09_cad/neck_hardware_v28_final'
def main():
    study=json.loads((OUT/'study.json').read_text());long=json.loads((OUT/'cable_and_loads.json').read_text());short=json.loads((OUT/'cable_and_loads_compact.json').read_text())
    height=json.loads((OUT/'height_refined.json').read_text());frames=json.loads((ROOT/'03_system/neck_interfaces_v28.json').read_text())
    assert study['complete'] and len(study['cases'])==27
    assert sum(r['pass'] for r in study['cases'])==17
    forward=[r for r in study['cases'] if r['yaw_deg']==0];assert len(forward)==11 and all(r['pass'] for r in forward)
    assert study['swept_motor_opening']['between_grid_motion_bound_mm']<study['swept_motor_opening']['padding_mm']
    assert study['neck_opening_radius_mm'] is None and study['neck_opening_z_body_mm'] is None
    assert study['source_motor_sha256']==sha(ROOT/'00_inputs/actuators/XC330/manufacturer.stp')
    assert frames['camera_center_in_head_mm']==[38,0,4] and not frames['brackets_and_fasteners_completed']
    # Verify explicit FK composition against around-axis representation.
    for y,p in [(0,0),(0,50),(-30,30),(30,45)]:
        Y=around([0,0,125],[0,0,1],y);H=Y.multiply(around([5,0,146],[0,1,0],p))
        linked=A.Placement(V(0,0,125),A.Rotation(V(0,0,1),y)).multiply(A.Placement(V(5,0,21),A.Rotation(V(0,1,0),p)))
        assert (H.multVec(V(43,0,150))-linked.multVec(V(38,0,4))).Length<1e-8
    for routes,length in [(long,70),(short,38)]:
        assert routes['complete'] and len(routes['cases'])==27
        for r in routes['cases']:
            a=r['cable'];assert abs(a['centerline_length_mm']-length)<1e-6
            assert abs(2*a['bend_radius_mm']*math.sin(a['angle_rad']/2)-a['chord_mm'])<1e-7
    assert height['head_at_pitch50_refined_surface_upper_Z_mm']<400
    assert height['crown_exact_max_over_pitch0_to50']['world_Z_mm']>400
    artifacts={}
    for prefix in ['', 'cable_', 'compact_cable_']:
        for label in ['y0_p0','y0_p45','y0_p50','y30_p45']:
            file=OUT/(prefix+label+'.FCStd');doc=A.openDocument(str(file));expected=118 if not prefix else 119
            assert len(doc.Objects)==expected and all(o.Shape.isValid() for o in doc.Objects);A.closeDocument(doc.Name)
            artifacts[str(file.relative_to(ROOT))]=sha(file)
    source=ROOT/'09_cad/step_repair_v26/repaired/stand.FCStd'
    previous=json.loads((ROOT/'09_cad/camera_v27/study.json').read_text());assert sha(source)==previous['source_CAD_sha256']
    metadata={p:sha(ROOT/p) for p in ['03_system/neck_v28.json','03_system/neck_interfaces_v28.json','09_cad/neck_chain_v28.py',
        '09_cad/neck_hardware_v28.py','09_cad/neck_cable_v28.py','11_verification/neck_height_v28.py','11_verification/check_neck_v28.py',
        '00_inputs/actuators/XC330/manufacturer_drawing.pdf','00_inputs/actuators/XC330/manufacturer.stp']}
    result={'stage':'v0.28-neck-study','checks':[
        {'name':'Manufacturer reference geometry and explicit yaw-pitch-camera frame composition','status':'passed','evidence':['03_system/neck_interfaces_v28.json','00_inputs/actuators/XC330/manufacturer_drawing.pdf']},
        {'name':'11 zero-yaw pitch samples 0..50deg without modeled housing/head collisions','status':'passed','evidence':['09_cad/neck_hardware_v28_final/study.json']},
        {'name':'Nominal motor-box containment bound for proposed head-local clearance opening','status':'passed','evidence':['09_cad/neck_hardware_v28_final/study.json']},
        {'name':'54 constant-length free-end arc cases and refined height arithmetic','status':'passed','evidence':['09_cad/neck_hardware_v28_final/cable_and_loads.json','09_cad/neck_hardware_v28_final/cable_and_loads_compact.json','09_cad/neck_hardware_v28_final/height_refined.json']},
        {'name':'12 native research CAD files reopen; v26 source unchanged','status':'passed','evidence':list(artifacts)},
        {'name':'All combined yaw/pitch configurations clear','status':'failed','evidence':['09_cad/neck_hardware_v28_final/study.json']},
        {'name':'Candidate cable routes cover intended neck motion','status':'failed','evidence':['09_cad/neck_hardware_v28_final/cable_and_loads_compact.json']},
        {'name':'Crown placeholder stays below400mm throughout pitch0..50','status':'failed','evidence':['09_cad/neck_hardware_v28_final/height_refined.json']},
        {'name':'Actual neck brackets/rotor fasteners,real USB cable,actuator thermal/strength,continuous whole-robot motion and manufacturing','status':'not_run','evidence':['03_system/neck_interfaces_v28.json']}
    ],'hardware_geometry_pass_count':17,'hardware_cases':27,'compact_combined_pass_count':sum(r['combined_geometry_pass'] for r in short['cases']),
       'artifacts_sha256':artifacts,'code_source_sha256':metadata,
       'pending':['Split and reroute yaw/pitch flex segments with actual cable and fixed exit tangents','Body/yaw/pitch/head brackets,rotor mounting phases and mechanical load path','Reduce/reshape crown allowance if400mm applies across all neck poses; appearance still unapproved',
                  'Joint-coupled safe motion sequence needs full continuous and dynamic verification','D436 purchase/mode/M4 gates,real microphone interfaces and full robot CAE/recovery remain open']}
    (ROOT/'11_verification/neck_v28_checks.json').write_text(json.dumps(result,indent=2)+'\n');print('Research gate saved:17/27 bare geometry; cable and crown gates FAILED, hardware release NOT RUN')
if __name__=='__main__':main()

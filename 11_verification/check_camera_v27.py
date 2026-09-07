"""Check documented research inputs and numerical/CAD evidence, not device approval."""
import json,sys,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'09_cad'))
from camera_study_v27 import A,Part,V,visible,pose,sha,C,P,OUT,CP,PF

def main():
    source=OUT/'study.json';r=json.loads(source.read_text());assert r['complete']
    assert r['candidate_input_sha256']==sha(CP) and r['task_input_sha256']==sha(PF)
    base=ROOT/'09_cad/step_repair_v26/repaired/stand.FCStd';assert r['source_CAD_sha256']==sha(base)
    assert len(r['assembly_cases'])==48 and len(r['optical_cases'])==1056 and len(r['mass_cases'])==128
    assert sum(x['box_geometry_pass'] for x in r['assembly_cases'])==15
    assert not P['camera_neck_motion_linkage_verified']
    c=next(x for x in C['candidates'] if x['id']=='D436');_,center,R,origin=pose(c,40,45)
    axispoint=origin+R.multVec(V(500,0,0));q=visible(list(axispoint),origin,R,[87,58])
    assert q['within_nominal_fov'] and abs(q['axial_Z_mm']-500)<1e-8
    behind=origin-R.multVec(V(10,0,0));assert not visible(list(behind),origin,R,[87,58])['within_nominal_fov']
    for row in r['optical_cases']:
        camera=next(x for x in C['candidates'] if x['id']==row['candidate']);mode=next(m for m in camera['modes'] if m['resolution']==row['mode'])
        assert row['above_mode_minZ']==(row['depth']['axial_Z_mm']>=mode['min_z_mm'])
        assert not row['measured_visibility_verified']
    prior=json.loads((ROOT/'09_cad/uac_mount_v25_service/integration.json').read_text())
    massbase=next(x['mass_with_CAD_mount_kg'] for x in prior['mass_sensitivity'] if x['scene']=='stand' and x['extra_board_wire_assumption_kg']==0)
    for row in r['mass_cases']:
        assert abs(row['modeled_total_mass_kg']-(massbase-.09+row['body_mass_kg']+row['accessory_assumption_kg']))<1e-12
    assert c['mass_kg'] is None and c['modes'][1]['inferred']
    files=[]
    for row in r['assembly_cases']:
        if 'study_CAD' not in row:continue
        p=ROOT/row['study_CAD'];doc=A.openDocument(str(p));assert len(doc.Objects)==119 and all(o.Shape.isValid() for o in doc.Objects);A.closeDocument(doc.Name);files.append(row['study_CAD'])
    assert len(files)==4
    rawdir=ROOT/'00_inputs/sensors/camera_v27';assets=[p for p in rawdir.iterdir() if p.is_file()]
    for name in ['realsense_d400.pdf','gemini330_v1_4.pdf']:
        assert (rawdir/name).read_bytes().startswith(b'%PDF')
    decision=json.loads((ROOT/'08_electrical/camera_decision_v27.json').read_text());assert not decision['procurement_frozen'] and not decision['hardware_test_executed']
    evidence=['09_cad/camera_v27/study.json','08_electrical/camera_candidates_v27.json','08_electrical/camera_decision_v27.json']
    result={'stage':'v0.27-camera-study','checks':[
        {'name':'Manufacturer PDF/page evidence archived; mode-dependent MinZ kept separate and D436 inference flagged','status':'passed','evidence':evidence+[str(p.relative_to(ROOT)) for p in assets if p.suffix=='.pdf']},
        {'name':'48 conservative catalogue-envelope cases,1056 nominal point/FOV cases,128 mass scenarios computed','status':'passed','evidence':evidence},
        {'name':'Four research CAD files reopen; original v26 CAD unchanged','status':'passed','evidence':files},
        {'name':'Pinhole orientation/axial-Z tests and mass replacement arithmetic','status':'passed','evidence':evidence},
        {'name':'All camera placements meet existing shell/neck geometry','status':'failed','evidence':evidence},
        {'name':'Actual camera fit,stereo visibility,continuous neck motion,M4/Windows device testing and procurement approval','status':'not_run','evidence':['08_electrical/camera_acceptance_v27.md']}
    ],'study_CAD_sha256':{p:sha(ROOT/p) for p in files},'source_archives_sha256':{str(p.relative_to(ROOT)):sha(p) for p in assets},
       'code_input_sha256':{str(p.relative_to(ROOT)):sha(p) for p in [CP,PF,ROOT/'09_cad/camera_study_v27.py',ROOT/'09_cad/render_camera_v27.py',Path(__file__)]},
       'pending':['D436 stock/China supplier,own drawing,mass/power and mode evidence','Define actual two-axis neck/head/camera coupling and optical centers; full moving-shell path',
                  'Rectified stereo overlap/invalid band,whole bottle/hands and target material performance','Windows and M4 SDK/capture latency tests; no SDK installation authorized','UAC mounting/wiring,full robot CAE/recovery and final Wukong appearance still pending']}
    (ROOT/'11_verification/camera_v27_checks.json').write_text(json.dumps(result,indent=2)+'\n')
    print('Research checks passed;33/48 box cases failed. Hardware/neck/platform/procurement NOT validated.')

if __name__=='__main__':main()

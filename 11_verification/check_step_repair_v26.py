"""Independent fresh-load exchange and critical-interface verification."""
import json,sys,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'09_cad'))
from repair_step_v26 import A,Part,cfg,DEST,OUT,sha,serialization_compare
from rebuild_rollrod_v26 import dimensions
from diagnose_step_v26 import check,delta
V=A.Vector

def main():
    resultfile=OUT/'repair_results.json';result=json.loads(resultfile.read_text());assert result['complete'] and len(result['scenes'])==4
    assert result['input_sha256']==sha(ROOT/'03_system/step_repair_v26.json')
    prior=json.loads((ROOT/'09_cad/uac_mount_v25_service/integration.json').read_text())
    rows=[];artifacts={}
    for scene in result['scenes']:
        name=scene['scene'];oldpath=ROOT/cfg['source_directory']/(name+'.FCStd');newpath=DEST/(name+'.FCStd')
        assert sha(oldpath)==scene['source_native_sha256'] and sha(oldpath.with_suffix('.step'))==scene['source_STEP_sha256']
        for p,h in scene['output_sha256'].items():assert sha(ROOT/p)==h
        olddoc=A.openDocument(str(oldpath));newdoc=A.openDocument(str(newpath))
        assert [o.Name for o in olddoc.Objects]==[o.Name for o in newdoc.Objects]
        assert len(newdoc.Objects)==118 and all(o.Shape.isValid() for o in newdoc.Objects)
        base=next(r for r in prior['mass_sensitivity'] if r['scene']==name and r['extra_board_wire_assumption_kg']==0)
        mass=base['mass_with_CAD_mount_kg'];moment=V(*base['COM_mm'])*mass
        interfaces=[]
        for o in olddoc.Objects:
            new=newdoc.getObject(o.Name).Shape
            if o.Name not in cfg['target_objects']:
                serialization_compare(o.Shape,new)
                continue
            before=dimensions(o.Shape);after=dimensions(new)
            errors=[]
            for e in before:
                point=o.Shape.Placement.multVec(e['point']);axis=o.Shape.Placement.Rotation.multVec(e['axis'])
                match=min(after,key=lambda f:(new.Placement.multVec(f['point'])-point).Length)
                center_error=(new.Placement.multVec(match['point'])-point).Length
                angle=abs(abs(axis.dot(new.Placement.Rotation.multVec(match['axis'])))-1)
                assert center_error<1e-6 and angle<1e-12 and abs(match['width']-4)<1e-6
                errors.append({'world_eye_center_error_mm':center_error,'axis_dot_error':angle})
            extra_check=check(new,True)
            interfaces.append({'object':o.Name,'eyes':errors,'native_BOP_check':extra_check})
            a=o.Shape.Solids[0];b=new.Solids[0];ma=a.Volume*7.85e-6;mb=b.Volume*7.85e-6
            mass+=mb-ma;moment+=b.CenterOfMass*mb-a.CenterOfMass*ma
        com=moment/mass
        assert abs(mass-base['mass_with_CAD_mount_kg'])<1e-6 and (com-V(*base['COM_mm'])).Length<.001
        step=Part.read(str(newpath.with_suffix('.step')))
        assert step.isValid() and all(s.isValid() for s in step.Solids)
        assert len(step.Solids)==sum(len(o.Shape.Solids) for o in newdoc.Objects)
        assert scene['changed_pair_audit']['pass']
        rows.append({'scene':name,'STEP_valid':True,'STEP_solids':len(step.Solids),'interfaces':interfaces,
                     'modeled_mass_kg_after':mass,'modeled_mass_delta_kg':mass-base['mass_with_CAD_mount_kg'],
                     'modeled_COM_mm_after':list(com),'modeled_COM_shift_mm':(com-V(*base['COM_mm'])).Length})
        A.closeDocument(olddoc.Name);A.closeDocument(newdoc.Name)
        for p in [newpath,newpath.with_suffix('.step')]:artifacts[str(p.relative_to(ROOT))]=sha(p)
        print('FRESH PASS',name,'solids',len(step.Solids),'BOP',[r['native_BOP_check']['error'] for r in interfaces],flush=True)
    bop_ok=all(t['native_BOP_check']['error'] is None for r in rows for t in r['interfaces'])
    sources=['03_system/step_repair_v26.json','09_cad/diagnose_step_v26.py','09_cad/rebuild_rollrod_v26.py',
             '09_cad/repair_step_v26.py','11_verification/check_step_repair_v26.py']
    gate={'stage':'v0.26-STEP-repair','checks':[
        {'name':'Four repaired native assemblies and full STEP files fresh-loaded with valid solids','status':'passed','evidence':list(artifacts)},
        {'name':'Eight rods preserve eye centers/axes/widths; bounded shape and mass/COM changes','status':'passed','evidence':['09_cad/step_repair_v26/repair_results.json']},
        {'name':'116 unrelated objects per scene verified; original native/STEP source hashes unchanged','status':'passed','evidence':['09_cad/step_repair_v26/repair_results.json']},
        {'name':'Rebuilt rods detailed BOP checks','status':'passed' if bop_ok else 'failed','evidence':['09_cad/step_repair_v26/verification.json']},
        {'name':'Physical fits, material strength/CAE, full recovery and manufacturing approval','status':'not_run','evidence':['03_system/step_repair_v26.json']}
    ],'fresh_load_results':rows,'artifacts_sha256':artifacts,'source_sha256':{p:sha(ROOT/p) for p in sources},
       'pending':['STEP exchange success is not CAE or manufacturing approval','Future CAD regeneration must rerun this postprocessor and gates; legacy exports remain preserved and invalid',
                  'Actual UAC board dimensions/holes/wiring, USB strain relief and mechanical verification still missing',
                  'Camera FOV/task/appearance/host-platform selection remains unfrozen','Full recovery stability, final Wukong appearance and Windows audio compatibility remain unresolved']}
    (OUT/'verification.json').write_text(json.dumps(gate,indent=2)+'\n')
    (ROOT/'11_verification/step_repair_v26_checks.json').write_text(json.dumps(gate,indent=2)+'\n')

if __name__=='__main__':main()

"""Evidence gate for reference structure; explicitly excludes physical release."""
import json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'09_cad'))
from uac_mount_v25 import A,Part,sha,cfg
OUT=ROOT/'09_cad/uac_mount_v25_service'

def main():
    f=OUT/'integration.json';data=json.loads(f.read_text())
    servicefile=ROOT/'07_mechanics/uac_service_v25_final.json';service=json.loads(servicefile.read_text())
    basefile=ROOT/'09_cad/body_v23_fit/integration.json';base=json.loads(basefile.read_text())
    assert data['completed'] and len(data['scenes'])==4 and all(r['pass'] for r in data['scenes'])
    assert data['input_sha256']==sha(ROOT/'03_system/uac_mount_v25.json')
    assert data['baseline_sha256']==sha(basefile) and service['integration_sha256']==sha(f)
    assert data['port_windows_and_probes'] and data['tool_relief_length_mm']==13
    assert service['sampled_service_geometry_pass'] and len(service['tool_access'])==4 and len(service['withdrawal_samples'])==16
    assert cfg['actual_board_dimensions_mm'] is None and cfg['actual_board_holes_mm'] is None and cfg['actual_board_mass_kg'] is None
    assert not data['actual_board_fit_verified'] and not data['preload_verified'] and not data['manufacturing_release']
    first=json.loads((ROOT/'09_cad/uac_mount_v25/integration.json').read_text())
    ports=json.loads((ROOT/'09_cad/uac_mount_v25_revision/port_audit.json').read_text())
    access=json.loads((ROOT/'07_mechanics/uac_service_v25.json').read_text())
    assert len(first['scenes'])==4 and all(len(r['failed_pairs'])==4 for r in first['scenes'])
    assert len(ports['failed_pairs'])==2 and not access['sampled_service_geometry_pass']
    assert len(data['mass_sensitivity'])==56
    for row in data['mass_sensitivity']:
        old=next(s for s in base['scenes'] if s['scene']==row['scene'])
        assert abs(row['mass_with_CAD_mount_kg']-old['new_mass_kg']-data['added_CAD_mass_kg']-row['extra_board_wire_assumption_kg'])<1e-10
    # 4 right-angle fillets trim adjacent straight runs. Verify analytic length.
    import math
    pts=cfg['array_route_body_mm'];length=sum(math.dist(a,b) for a,b in zip(pts,pts[1:]))
    expected=length-4*(2*6-math.pi*6/2)
    assert abs(expected-data['path']['centerline_length_mm'])<1e-8
    hashes={};step_checks=[]
    for name,count in [('mount_native',18),('stand',118),('lift_h114',118),('lift_h117',118),('lift_h120',118)]:
        p=OUT/(name+'.FCStd');d=A.openDocument(str(p));assert len(d.Objects)==count
        assert all(o.Shape.isValid() and len(o.Shape.Solids)>=1 for o in d.Objects)
        if name=='mount_native':
            modeled=sum(o.Shape.Volume*cfg['densities_kg_mm3'][data['mount_meta'][o.Name]['material_assumption']] for o in d.Objects if data['mount_meta'][o.Name]['mass_kg'] is not None)
        if name!='mount_native':
            src=ROOT/'09_cad/body_v23_fit'/(name+'.FCStd')
            assert next(r for r in data['scenes'] if r['scene']==name)['source_sha256']==sha(src)
        A.closeDocument(d.Name)
        hashes[str(p.relative_to(ROOT))]=sha(p)
        step=p.with_suffix('.step');shape=Part.read(str(step));assert len(shape.Solids)>=count
        invalid=[i for i,s in enumerate(shape.Solids) if not s.isValid()]
        sr={'file':str(step.relative_to(ROOT)),'valid':shape.isValid(),'solid_count':len(shape.Solids),'invalid_solid_indices':invalid}
        if name=='mount_native':assert sr['valid'] and not invalid
        elif not sr['valid']:
            oldstep=ROOT/'09_cad/body_v23_fit'/(name+'.step');oldshape=Part.read(str(oldstep))
            bad=[i for i,s in enumerate(oldshape.Solids) if not s.isValid()]
            matches=[]
            for i in invalid:
                if i>=len(oldshape.Solids):continue
                a,b=shape.Solids[i],oldshape.Solids[i]
                match=i in bad and abs(a.Volume-b.Volume)<1e-5 and all(abs(getattr(a.BoundBox,k)-getattr(b.BoundBox,k))<1e-5 for k in ('XMin','XMax','YMin','YMax','ZMin','ZMax'))
                matches.append({'index':i,'matches_baseline_volume_and_bounds':match})
            sr.update(baseline_step=str(oldstep.relative_to(ROOT)),baseline_sha256=sha(oldstep),baseline_invalid_solid_indices=bad,baseline_matches=matches)
        step_checks.append(sr)
        hashes[str(step.relative_to(ROOT))]=sha(step)
        print('Reopened native CAD',name,'STEP valid',sr['valid'],'invalid solids',invalid,flush=True)
    d=A.openDocument(str(ROOT/'09_cad/body_v23_fit/stand.FCStd'))
    oldmass=d.getObject('ElectronicsCarrier').Shape.Volume*cfg['densities_kg_mm3']['PETG_reference'];A.closeDocument(d.Name)
    assert abs(modeled-oldmass-data['added_CAD_mass_kg'])<1e-10
    sources=['03_system/uac_mount_v25.json','09_cad/uac_mount_v25.py','07_mechanics/check_uac_service_v25.py',
             '11_verification/check_uac_mount_v25.py','09_cad/render_uac_mount_v25.py','08_electrical/camera_selection_requirements.md',
             'deliverables/uac_mount_review_v25.md','09_cad/uac_mount_v25_service/mount_review.png']
    result={'stage':'v0.25-reference-mount','checks':[
        {'name':'Four sampled poses: changed structure, ports, cable proxy, ground and positive-volume intersection checks','status':'passed','evidence':[str(f.relative_to(ROOT))]},
        {'name':'Four shaft-access probes and sixteen stationary cassette-withdrawal samples','status':'passed','evidence':[str(servicefile.relative_to(ROOT))]},
        {'name':'Five native CAD reloaded; net CAD mass and 56 scenario totals checked','status':'passed','evidence':[p for p in hashes if p.endswith('.FCStd')]},
        {'name':'New mount-only STEP reloaded with valid solids','status':'passed','evidence':['09_cad/uac_mount_v25_service/mount_native.step']},
        {'name':'Full-robot STEP topology on reimport','status':'passed' if all(r['valid'] for r in step_checks) else 'failed','evidence':[p for p in hashes if p.endswith('.step')]},
        {'name':'Previous head/port/tool-access failures retained','status':'passed','evidence':['09_cad/uac_mount_v25/integration.json','09_cad/uac_mount_v25_revision/port_audit.json','07_mechanics/uac_service_v25.json']},
        {'name':'Camera task/FOV/mechanical/platform selection requirements saved','status':'passed','evidence':['08_electrical/camera_selection_requirements.md']},
        {'name':'Real PCB fixation, actual cable/USB retention, material strength/preload, CAE and full robot tasks','status':'not_run','evidence':[str(f.relative_to(ROOT))]}
    ],'source_sha256':{rel:sha(ROOT/rel) for rel in sources},'artifacts_sha256':hashes,'STEP_checks':step_checks,
       'pending':['Actual board data, mounting interface and insulation/thermal clearance','Actual wire bend radius/type, signal integrity, USB strain relief and pull retention',
                  'Fastener SKU, thread/preload, nut capture, print tolerance and material/impact strength','Continuous motion/removal and full recovery/CAE/final Wukong verification',
                  'Full-robot STEP reimport errors require separate topology/export investigation; do not release affected STEP files',
                  'Camera remains a placeholder; future FOV/task/mechanical/platform selection required','Windows audio Code10 unchanged; no device operations or software/MCP installs']}
    (ROOT/'11_verification/uac_mount_v25_checks.json').write_text(json.dumps(result,indent=2)+'\n')
    print('v25 native-CAD/mass/service checks passed; see explicit STEP failures. Physical/manufacturing gates NOT RUN')

if __name__=='__main__':main()

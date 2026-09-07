"""Reopen CAD, validate provenance and arithmetic; keep physical gates pending."""
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'09_cad'))
from uac_space_v24 import A,Part,V,box,audit,sha,OUT


def main():
    sf=OUT/'search.json';rf=OUT/'routing_upper.json'
    s=json.loads(sf.read_text());r=json.loads(rf.read_text())
    p=json.loads((ROOT/'03_system/uac_space_v24.json').read_text())
    baseline=json.loads((ROOT/p['source_geometry']).read_text())
    assert s['completed'] and r['completed']
    assert s['input_sha256']==sha(ROOT/'03_system/uac_space_v24.json')
    assert s['baseline_sha256']==sha(ROOT/p['source_geometry'])
    assert r['search_sha256']==sha(sf)
    assert len(s['candidates'])==16 and len(r['routes'])==6
    assert all(len(c['scenes'])==4 for c in s['candidates']+r['routes'])
    assert s['selected_candidate']['four_scene_space_pass'] and r['selected_route']['pass']
    for candidate in s['candidates']+r['routes']:
        for scene in candidate['scenes']:
            assert scene['tested_pairs']==len(scene['exact_pairs'])+scene['aabb_certified_pairs']
    # Regression tests include complete containment (not just intersecting surfaces).
    outer=box([0,0,0],[10,10,10]);inner=box([0,0,0],[1,1,1])
    assert not audit({'p':inner},{'outer':outer},{'outer':outer.BoundBox},2)['pass']
    assert not audit({'p':box([6.5,0,0],[1,1,1])},{'outer':outer},{'outer':outer.BoundBox},2)['pass']
    assert audit({'p':box([8,0,0],[1,1,1])},{'outer':outer},{'outer':outer.BoundBox},2)['pass']
    assert len(s['mass_sensitivity'])==56
    for row in s['mass_sensitivity']:
        old=next(b for b in baseline['scenes'] if b['scene']==row['scene'])
        added=row['added_assumed_mass_kg'];total=row['total_scenario_mass_kg']
        assert abs(total-old['new_mass_kg']-added)<1e-12
        for i in range(3):
            expected=(old['new_mass_kg']*old['COM_mm'][i]+added*row['point_mass_world_mm'][i])/total
            assert abs(expected-row['COM_mm'][i])<1e-10
        if added==0 and old['scene']!='stand':
            assert abs(row['support_margin_mm']-old['support_margin']['support_polygon_margin_mm'])<1e-9
    assert p['actual_board_dimensions_mm'] is None and p['actual_board_mass_kg'] is None
    assert not s['actual_board_fit_verified'] and not r['actual_cable_routing_verified']
    artifacts={};dims=[]
    for name in p['scenes']:
        original=ROOT/'09_cad/body_v23_fit'/(name+'.FCStd')
        assert s['cad_input_sha256'][name]==sha(original)
        docs=[]
        for prefix,count in [('',104),('routed_',109)]:
            path=OUT/(prefix+name+'.FCStd');d=A.openDocument(str(path));docs.append(d)
            assert len(d.Objects)==count and all(o.Shape.isValid() for o in d.Objects)
            # Retained objects stay unchanged between bay-only and routed assemblies.
            if len(docs)==2:
                for o in docs[0].Objects:
                    other=d.getObject(o.Name)
                    assert abs(o.Shape.Volume-other.Shape.Volume)<1e-6
                    assert abs(o.Shape.Area-other.Shape.Area)<1e-6
                    assert len(o.Shape.Solids)==len(other.Shape.Solids)
                    for field in ('XMin','XMax','YMin','YMax','ZMin','ZMax'):
                        assert abs(getattr(o.Shape.BoundBox,field)-getattr(other.Shape.BoundBox,field))<1e-8
            bb=A.BoundBox()
            for obj in d.Objects:bb.add(obj.Shape.BoundBox)
            dims.append({'scene':name,'variant':prefix or 'bay_only','xyz_mm':[bb.XLength,bb.YLength,bb.ZLength]})
            artifacts[str(path.relative_to(ROOT))]=sha(path)
            step=path.with_suffix('.step');assert step.stat().st_size>1000
            artifacts[str(step.relative_to(ROOT))]=sha(step)
        for d in docs:A.closeDocument(d.Name)
    result={'stage':'v0.24-space-study','source_sha256':{rel:sha(ROOT/rel) for rel in [
        '03_system/uac_space_v24.json','09_cad/uac_space_v24.py','09_cad/uac_route_v24.py',
        '09_cad/render_uac_space_v24.py','11_verification/check_uac_space_v24.py',
        'deliverables/uac_packaging_review_v24.md','09_cad/uac_space_v24/space_review.png']},'checks':[
        {'name':'Four-pose bay and upper-route reservation clearance; source hashes and negative tests','status':'passed','evidence':[str(sf.relative_to(ROOT)),str(rf.relative_to(ROOT))]},
        {'name':'Retained lower-route failures','status':'passed','evidence':['09_cad/uac_space_v24/routing.json']},
        {'name':'Eight native CAD files reopen; retained volume/area/solid-count/bounds match; STEP exports exist','status':'passed','evidence':list(artifacts)},
        {'name':'56 conditional mass and COM arithmetic cases','status':'passed','evidence':[str(sf.relative_to(ROOT))]},
        {'name':'Received board fits mount and actual cable route','status':'not_run','evidence':['03_system/uac_space_v24.json']},
        {'name':'Full recovery, actuator/structural CAE and manufacturing release','status':'not_run','evidence':[str(sf.relative_to(ROOT))]}
    ],'artifacts_sha256':artifacts,'envelopes':dims,
       'pending':['Actual PCB dimensions/holes/mass and mounting design','Actual array revision, sockets, wires, bend radius, signal integrity and strain relief',
                  'Only four geometries checked; no new 13-pose or continuous swept-volume claim',
                  'High recovery margin remains small; full task, CAE and final Wukong appearance unvalidated',
                  'Windows UAC error remains; no device operations/firmware/software changes']}
    (ROOT/'11_verification/uac_space_v24_checks.json').write_text(json.dumps(result,indent=2)+'\n')
    print('CHECKS PASSED; physical fit, CAE and manufacturing NOT RELEASED')
    print(dims)


if __name__=='__main__':main()

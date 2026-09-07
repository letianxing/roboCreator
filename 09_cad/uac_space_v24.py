"""Read frozen v23 CAD; test explicit *space reservations*, not invented PCB sizes."""
import hashlib
import itertools
import json
import math
from pathlib import Path
import sys

import FreeCAD as A
import Part
import numpy as np
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / '09_cad/uac_space_v24'
PF = ROOT / '03_system/uac_space_v24.json'
V = A.Vector


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def box(center, dims):
    return Part.makeBox(*dims, V(*[c-d/2 for c,d in zip(center,dims)]))


def placed(shape, T):
    result = shape.copy()
    result.Placement = T.multiply(result.Placement)
    return result


def gap(a,b):
    return math.sqrt(sum(max(0,l2-h1,l1-h2)**2 for l1,h1,l2,h2 in
        [(a.XMin,a.XMax,b.XMin,b.XMax),(a.YMin,a.YMax,b.YMin,b.YMax),(a.ZMin,a.ZMax,b.ZMin,b.ZMax)]))


def probes(center, orientation, cfg):
    dims = cfg[orientation+'_bay_xyz_mm']
    axis = 2 if orientation == 'vertical' else 1
    result = {'UAC_Bay_RESERVATION':box(center,dims)}
    for name, sign, length in [('USB_Exit_RESERVATION',-1,cfg['usb_plug_and_exit_reservation_mm']),
                               ('Array_Exit_RESERVATION',1,cfg['array_connector_exit_reservation_mm'])]:
        size = [dims[0],16,16]
        size[axis] = length
        point = list(center)
        point[axis] += sign*(dims[axis]/2+length/2)
        result[name] = box(point,size)
    return result


def audit(probe, parts, bounds, clearance):
    """AABB lower bound proves distant pairs; close pairs use exact BRep distance."""
    failed = []
    exact = []
    skipped = 0
    for pn,ps in probe.items():
        pb = ps.BoundBox
        for name,s in parts.items():
            if gap(pb,bounds[name]) >= clearance+1e-6:
                skipped += 1
                continue
            d = ps.distToShape(s)[0]
            row = {'reservation':pn,'part':name,'distance_mm':d}
            exact.append(row)
            if d < clearance-1e-6:
                failed.append(row)
    return {'pass':not failed,'failed_pairs':failed,'exact_pairs':exact,'aabb_certified_pairs':skipped,
            'tested_pairs':len(probe)*len(parts)}


def margin(points,com):
    eq = ConvexHull(np.array(points)[:,:2]).equations
    return float(np.min(-(eq[:,:2]@np.array(com)[:2]+eq[:,2])))


def main():
    cfg = json.loads(PF.read_text(encoding='utf8'))
    OUT.mkdir(exist_ok=True)
    source = ROOT / cfg['source_geometry']
    baseline = json.loads(source.read_text())
    segment = json.loads((ROOT/'04_kinematics/recovery_v22/segment_samples.json').read_text())
    assert baseline['completed'] and all(s['geometry_pass'] for s in baseline['scenes'])
    poses = {}
    for name in cfg['scenes']:
        if name == 'stand':
            T = A.Placement(V(0,0,37+180*math.cos(math.radians(10))),A.Rotation())
        else:
            sample = next(s for s in segment['samples'] if 'lift_h'+format(s['h_mm'],'g')==name)
            row = sample['input']
            T = A.Placement(V(row['hip_x_mm'],0,row['h_mm']),A.Rotation(V(0,1,0),row['beta_deg']))
        path = ROOT/'09_cad/body_v23_fit'/(name+'.FCStd')
        doc = A.openDocument(str(path))
        parts = {o.Name:o.Shape.copy() for o in doc.Objects if hasattr(o,'Shape')}
        A.closeDocument(doc.Name)
        assert len(parts)==101 and all(s.isValid() for s in parts.values())
        poses[name] = (parts,{n:s.BoundBox for n,s in parts.items()},T)
        print('Loaded',name,len(parts),flush=True)
    results=[]
    for idx,(center,orientation) in enumerate(itertools.product(cfg['centers_body_mm'],('vertical','horizontal'))):
        native=probes(center,orientation,cfg)
        scenes=[]
        for name,(parts,bounds,T) in poses.items():
            transformed={n:placed(s,T) for n,s in native.items()}
            result=audit(transformed,parts,bounds,cfg['clearance_probe_mm'])
            result['scene']=name
            result['ground_clearance_mm']=min(s.BoundBox.ZMin for s in transformed.values())
            result['pass']=result['pass'] and result['ground_clearance_mm']>=cfg['clearance_probe_mm']
            scenes.append(result)
        row={'id':idx,'center_body_mm':center,'orientation':orientation,'scenes':scenes,
             'four_scene_space_pass':all(s['pass'] for s in scenes)}
        results.append(row)
        (OUT/'search.json').write_text(json.dumps({'completed':False,'candidates':results},indent=2)+'\n')
        print('Candidate',idx,center,orientation,[len(s['failed_pairs']) for s in scenes],flush=True)
    passed=[r for r in results if r['four_scene_space_pass']]
    # Preference is minimal extra front/back envelope, then low and central mounting.
    passed.sort(key=lambda r:(abs(r['center_body_mm'][0]),abs(r['center_body_mm'][1]),r['center_body_mm'][2],r['id']))
    selected=passed[0] if passed else None
    mass_rows=[]
    if selected:
        native=probes(selected['center_body_mm'],selected['orientation'],cfg)
        for name,(parts,bounds,T) in poses.items():
            doc=A.newDocument('UAC_space_'+name)
            for n,s in {**parts,**{n:placed(s,T) for n,s in native.items()}}.items():
                obj=doc.addObject('PartDesign::Feature',n);obj.Shape=s
                obj.addProperty('App::PropertyString','Scope')
                obj.Scope='v24 space reservation only; no actual board dimensions, holes, mount, cable or manufacturing release'
            doc.recompute();target=OUT/(name+'.FCStd');doc.saveAs(str(target))
            Part.export(doc.Objects,str(OUT/(name+'.step')));A.closeDocument(doc.Name)
            reopened=A.openDocument(str(target));assert len(reopened.Objects)==104
            assert all(o.Shape.isValid() for o in reopened.Objects)
            A.closeDocument(reopened.Name)
        for base in baseline['scenes']:
            if base['scene']=='stand':
                T=poses['stand'][2]
                points=[[x,y,0] for x in (-50,50) for y in (-87.5,-32.5,32.5,87.5)]
            else:
                sample=next(s for s in segment['samples'] if s['h_mm']==base['h_mm'])
                r=sample['input'];T=A.Placement(V(r['hip_x_mm'],0,r['h_mm']),A.Rotation(V(0,1,0),r['beta_deg']))
                points=segment['support_points_mm']
            center=np.array(list(T.multVec(V(*selected['center_body_mm']))))
            for mass in [0]+cfg['accessory_mass_probes_kg']:
                total=base['new_mass_kg']+mass
                com=(np.array(base['COM_mm'])*base['new_mass_kg']+center*mass)/total
                mass_rows.append({'scene':base['scene'],'added_assumed_mass_kg':mass,'total_scenario_mass_kg':total,
                    'COM_mm':com.tolist(),'support_margin_mm':margin(points,com),
                    'point_mass_world_mm':center.tolist(),'dynamic_stability_verified':False})
    report={'completed':True,'input_sha256':sha(PF),'baseline_sha256':sha(source),
            'cad_input_sha256':{n:sha(ROOT/'09_cad/body_v23_fit'/(n+'.FCStd')) for n in poses},
            'candidates':results,'selected_candidate':selected,'mass_sensitivity':mass_rows,
            'geometry_scope':'4 frozen scenes, only new reservations versus 101 retained shapes. No internal material claims for connected reservation zones.',
            'actual_board_fit_verified':False,'actual_cable_routing_verified':False,
            'manufacturing_release':False,'full_recovery_verified':False}
    (OUT/'search.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
    print('PASSED CANDIDATES',[r['id'] for r in passed],flush=True)
    print('SELECTED',selected['id'] if selected else None,flush=True)


if __name__=='__main__':
    main()

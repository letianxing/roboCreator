"""Build v21 frozen motion poses, applying explicit v22 electronics translations only."""
import json,sys
from pathlib import Path
import FreeCAD as A,Part
from geometry_v21 import Geometry,V,ROOT
sys.path.insert(0,str(ROOT/'05_dynamics'))
from support_balance_v19 import reactions
from contact_margin_v21 import assess
g=Geometry();OUT=ROOT/'09_cad/body_v22';OUT.mkdir(exist_ok=True);g.out=OUT;g.env['OUT']=OUT
PF=ROOT/'03_system/electronics_motion_v22.json';cfg=json.loads(PF.read_text())

def motion(sample):
    row=sample['input'];parts,info=g.fixed(row);lp,lm,_,_=g.leg(row['left_pose'])
    for n,s in lp.items():
        s=s.copy();s.translate(V(0,60,0));parts['L_'+n]=s;info['L_'+n]={**lm[n],'group':'L_leg'}
    for side,key in [(1,'left_arm'),(-1,'right_arm')]:
        ap,am,_,_=g.arms(row,sample[key]['q_deg'],side);parts.update(ap);info.update(am)
    T=A.Placement(V(row['hip_x_mm'],0,row['h_mm']),A.Rotation(V(0,1,0),row['beta_deg']))
    return parts,info,T

def stand():
    source=json.loads((ROOT/'09_cad/body_v19/assembly_results.json').read_text())['scenes'][0]
    doc=A.openDocument(str(ROOT/'09_cad/body_v19/stand_candidate.FCStd'));parts={o.Name:o.Shape.copy() for o in doc.Objects if hasattr(o,'Shape')};A.closeDocument(doc.Name)
    # Metadata/masses from the unchanged current geometry, not from uniform-density motor CAD.
    ref=next(s for s in json.loads((ROOT/'09_cad/body_v21/local_lift.json').read_text())['samples'] if s['h_mm']==120)
    _,info,_=motion(ref)
    B=A.Rotation(V(0,1,0),0)
    # Source standing hip pitch datum = planted foot37 + two90mm links at10deg.
    import math
    hip=V(0,0,37+180*math.cos(math.radians(10)));T=A.Placement(hip,B)
    parts['power'].translate(V(-5,0,0))
    return parts,info,T,source

def apply_layout(parts,T,layout):
    out={n:s.copy() for n,s in parts.items()}
    for n,delta in layout.items():out[n].translate(T.Rotation.multVec(V(*delta)))
    return out

def mass_com(parts,info):
    mass=sum(r['mass_kg'] for r in info.values() if r['mass_kg'] is not None);com=V()
    for n,r in info.items():
        if r['mass_kg'] is not None:com+=g.env['volcenter'](parts[n])*(r['mass_kg']/mass)
    return mass,list(com)

def box_gap(a,b):
    import math
    return math.sqrt(sum(max(0,lo2-hi1,lo1-hi2)**2 for lo1,hi1,lo2,hi2 in [(a.XMin,a.XMax,b.XMin,b.XMax),(a.YMin,a.YMax,b.YMin,b.YMax),(a.ZMin,a.ZMax,b.ZMin,b.ZMax)]))

def pair_check(a,b,clearance=0):
    try:
        common=a.common(b)
        if common.isNull():raise ValueError('empty common result requires separation certificate')
        volume=common.Volume
    except Exception as e:
        d=a.distToShape(b)[0]
        return {'pass':bool(d>=max(clearance,1e-6)),'distance_mm':d,'volume_mm3':None,'kernel_note':str(e)}
    if volume>cfg['collision_volume_threshold_mm3']:return {'pass':False,'volume_mm3':volume,'distance_mm':0}
    if clearance:
        distance=a.distToShape(b)[0];return {'pass':bool(distance>=clearance-1e-6),'volume_mm3':volume,'distance_mm':distance}
    return {'pass':True,'volume_mm3':volume,'distance_mm':None}

def changed_audit(parts,changed,clearance=0):
    import itertools
    bb=g.boxes(parts);failed=[];near=[]
    for a,b in itertools.combinations(parts,2):
        if a not in changed and b not in changed:continue
        if box_gap(bb[a],bb[b])>clearance+1e-6:continue
        check=pair_check(parts[a],parts[b],clearance)
        near.append({'a':a,'b':b,**check})
        if not check['pass']:failed.append({'a':a,'b':b,**check})
    return failed,near

def save(name,parts,info):
    doc=A.newDocument(name);objects=[];mesh=[]
    for n,s in parts.items():
        assert s.isValid(),n
        o=doc.addObject('PartDesign::Feature',n);o.Shape=s;o.addProperty('App::PropertyString','ValidationScope');o.ValidationScope='v22 motion packaging research; manufacturing/dynamics unvalidated';objects.append(o)
        bb=s.BoundBox;vis=Part.makeBox(bb.XLength,bb.YLength,bb.ZLength,V(bb.XMin,bb.YMin,bb.ZMin)) if info[n]['role'] in ('motor','electronics') else s
        pts,tris=vis.tessellate(1);mesh.append({'name':n,'role':info[n]['role'],'points':[list(p) for p in pts],'triangles':tris})
    doc.recompute();path=OUT/(name+'.FCStd');doc.saveAs(str(path));Part.export(objects,str(OUT/(name+'.step')));A.closeDocument(doc.Name)
    doc=A.openDocument(str(path));assert len(doc.Objects)==82 and all(o.Shape.isValid() for o in doc.Objects if hasattr(o,'Shape'));A.closeDocument(doc.Name)
    (OUT/(name+'_mesh.json')).write_text(json.dumps(mesh)+'\n',encoding='utf-8')

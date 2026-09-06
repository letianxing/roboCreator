"""Official-model based orthogonal ankle/wrist motor packaging. No frame release."""
import FreeCAD as A, Part, json, math, itertools, hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/modules_v15';OUT.mkdir(exist_ok=True)
PFILE=ROOT/'03_system/layout_load_inputs_v15.json';cfg=json.loads(PFILE.read_text(encoding='utf-8'));p=cfg['packaging'];V=A.Vector
xm=Part.Shape();xm.read(str(ROOT/cfg['references']['XM430_file']))
assert xm.isValid() and len(xm.Solids)>0
bb=xm.BoundBox
# Vendor native shaft line: X=Y=0, Z axis, based on coaxial cylindrical surfaces.
# Origin along shaft line is arbitrary for packing; horn/bolt datum not released.
xmenv=Part.makeBox(bb.XLength,bb.YLength,bb.ZLength,V(bb.XMin,bb.YMin,bb.ZMin))
ak=Part.makeCylinder(26.5,40.2,V(0,0,-20.1))
xc=Part.makeBox(20,34,26,V(-10,-17,-13))
refs={'XM':xmenv,'AK':ak,'XC':xc}
def posed(kind,point,axis,clock):
    s=refs[kind].copy();r=A.Rotation(V(0,0,1),V(*axis)).multiply(A.Rotation(V(0,0,1),clock))
    s.Placement=A.Placement(V(*point),r);return s
def bbdict(s):
    b=s.BoundBox;return {'min_mm':[b.XMin,b.YMin,b.ZMin],'max_mm':[b.XMax,b.YMax,b.ZMax], 'size_mm':[b.XLength,b.YLength,b.ZLength]}
def build_shapes(kinds,dx,dy,dz,roll,zbase,clock0=0,clock1=0):
    roll_origin=V(0,0,zbase+dz)
    fixed=posed(kinds[0],[-dx,0,zbase+dz],(1,0,0),clock0)
    moving=posed(kinds[1],[0,dy,zbase],(0,1,0),clock1)
    moving.rotate(roll_origin,V(1,0,0),roll)
    return fixed,moving
allout=[];render=[]
for label,kinds in [('ankle_A',('XM','XM')),('ankle_B',('XM','AK')),('wrist_small',('XC','XC')),('wrist_upgrade',('XM','XM'))]:
    rows=[];isfoot=label.startswith('ankle');zbase=p['pitch_axis_height_mm'] if isfoot else 0
    for dx,dy,dz,c0,c1 in itertools.product(p['axis_offsets_mm'],p['axis_offsets_mm'],p['roll_pitch_height_offsets_mm'],p['body_clocking_deg'],p['body_clocking_deg']):
        minsep=1e9;minz=1e9;points=[]
        for roll in p['roll_samples_deg']:
            s1,s2=build_shapes(kinds,dx,dy,dz,roll,zbase,c0,c1)
            distance=s1.distToShape(s2)[0]
            minsep=min(minsep,distance);b=Part.makeCompound([s1,s2]).BoundBox;minz=min(minz,b.ZMin)
            points.extend([V(b.XMin,b.YMin,b.ZMin),V(b.XMax,b.YMax,b.ZMax)])
        swept=A.BoundBox()
        for point in points:swept.add(point)
        clear=minsep>=p['clearance_mm'] and (not isfoot or minz>=p['ground_clearance_mm'])
        rows.append({'dx_mm':dx,'dy_mm':dy,'dz_mm':dz,'clock0_deg':c0,'clock1_deg':c1,'min_motor_gap_mm':minsep,'min_z_mm':minz,
          'swept_size_mm':[swept.XLength,swept.YLength,swept.ZLength],
          'centered_sole_xy_offset_mm':[(swept.XMin+swept.XMax)/2,(swept.YMin+swept.YMax)/2] if isfoot else None,
          'could_fit_recentered_sole_xy':swept.XLength<=p['foot_length_mm'] and swept.YLength<=p['foot_width_mm'] if isfoot else None,
          'within_foot_xy':swept.XMin>=-p['foot_length_mm']/2 and swept.XMax<=p['foot_length_mm']/2 and swept.YMin>=-p['foot_width_mm']/2 and swept.YMax<=p['foot_width_mm']/2 if isfoot else None,
          'sampled_motor_clearance_pass':clear})
    valid=[r for r in rows if r['sampled_motor_clearance_pass']]
    assert valid,label
    # Select smallest swept bounding volume, not a hidden full-task success score.
    best=min(valid,key=lambda r:math.prod(r['swept_size_mm']))
    solids=build_shapes(kinds,best['dx_mm'],best['dy_mm'],best['dz_mm'],0,zbase,best['clock0_deg'],best['clock1_deg'])
    doc=A.newDocument(label+'_v15');objs=[]
    for idx,s in enumerate(solids):
        o=doc.addObject('PartDesign::Feature','MotorEnvelope'+str(idx));o.Shape=s
        o.addProperty('App::PropertyString','EvidenceScope');o.EvidenceScope='Conservative envelope only; support/frame/cables/costume absent'
        objs.append(o)
        verts,tris=s.tessellate(.8);render.append({'module':label,'name':o.Name,'points':[list(v) for v in verts],'triangles':tris,'color':[.2,.45,.75] if idx==0 else [.85,.4,.15]})
        if kinds[idx]=='XM':
            vendor=xm.copy();vendor.Placement=s.Placement
            o2=doc.addObject('PartDesign::Feature','VendorXM'+str(idx));o2.Shape=vendor
            o2.addProperty('App::PropertyString','EvidenceScope');o2.EvidenceScope='Official XM/H430 with idler reference; assembly mass differs from motor-only 82g'
    if isfoot:
        foot=doc.addObject('PartDesign::Feature','SoleReference');foot.Shape=Part.makeBox(p['foot_length_mm'],p['foot_width_mm'],2,V(-p['foot_length_mm']/2,-p['foot_width_mm']/2,-2))
    doc.recompute();path=OUT/(label+'.FCStd');doc.saveAs(str(path));Part.export(objs,str(OUT/(label+'_envelopes.step')))
    A.closeDocument(doc.Name);reopened=A.openDocument(str(path));assert all(o.Shape.isValid() for o in reopened.Objects if hasattr(o,'Shape'));A.closeDocument(reopened.Name)
    allout.append({'module':label,'kinds':kinds,'search_count':len(rows),'sample_count_each':len(p['roll_samples_deg']),
      'sampled_clear_candidates':len(valid),'foot_xy_candidates':sum(bool(r['within_foot_xy']) for r in valid) if isfoot else None,
      'selected':best,'all_trials':rows,'geometry_scope':'Motor envelopes at 5 roll samples only; pitch output/link absent; not a functional joint',
      'mass_from_shapes_used':False,'manufacturable':False})
summary={'input_sha256':hashlib.sha256(PFILE.read_bytes()).hexdigest(),'vendor_xm_sha256':hashlib.sha256((ROOT/cfg['references']['XM430_file']).read_bytes()).hexdigest(),
 'vendor_xm_bounds':bbdict(xm),'vendor_xm_solid_count':len(xm.Solids),'vendor_xm_valid':xm.isValid(),
 'freecad_version':A.Version(),'modules':allout,'note':'XM envelope includes idler and protrusions: shaft dimension 42.7mm vs bare advertised 34mm. Mass of accessories unverified. AK official surface model retained as source; closed diameter53 length40.2 envelope used. No whole-leg/whole-arm or costume clearance certification.'}
(OUT/'packaging_results.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
(OUT/'render_mesh.json').write_text(json.dumps(render)+'\n',encoding='utf-8')
for row in allout:print(row['module'],row['sampled_clear_candidates'],'clear of',row['search_count'],'selected',row['selected'])

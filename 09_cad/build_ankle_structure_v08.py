"""Solid structural concept using measured STEP interface positions.
Run in FreeCAD bundled Python. NOT a manufacturing/strength release.
"""
import FreeCAD as A, Part, json, math, itertools, argparse
from pathlib import Path
parser=argparse.ArgumentParser();parser.add_argument('--variant',choices=['v08','v09'],default='v08')
args=parser.parse_args();VERSION=args.variant
R=Path(__file__).resolve().parents[1]; OUT=R/'09_cad'/('ankle_'+VERSION);OUT.mkdir(exist_ok=True)
V=A.Vector
SRC=R/'00_inputs/actuators/AK40_V3/model/AK40-10 V3.0 KV170.STEP'
vendor=Part.read(str(SRC)); vendor.translate(V(0,-6.1,0))
RC=V(-30,0,45);PC=V(39,0,45)
# Native negative Y is output (PCD27), positive Y is rear cover.
rr=A.Rotation(V(0,-1,0),V(1,0,0));pr=A.Rotation(V(0,-1,0),V(0,1,0))
rp=A.Placement(RC,rr);pp=A.Placement(PC,pr)
def native(s,placement):
    s=s.copy();s.Placement=placement;return s
def box(x,y,z,dx,dy,dz):return Part.makeBox(dx,dy,dz,V(x,y,z))
def cyl(radius,length,origin,axis):return Part.makeCylinder(radius,length,origin,axis)
def ring(ro,ri,length,origin,axis):return cyl(ro,length,origin,axis).cut(cyl(ri,length,origin,axis))
def union(*shapes):
    s=shapes[0]
    for t in shapes[1:]:s=s.fuse(t)
    return s.removeSplitter()
def drill(s,centers,axis,radius=1.4,length=12):
    for p in centers:s=s.cut(cyl(radius,length,p-axis*(length/2),axis))
    return s.removeSplitter()
output_xz=[(11.69134295109,6.75),(-11.69134295109,6.75),(0,-13.5)]
stator_xz=[(23.75,0),(-11.875,20.56810333988),(-11.875,-20.56810333988)]
def centers(points,placement,s):return [placement.multVec(V(x,-s,z)) for x,z in points]
# Centered native output plane=-20.1, stator front plane=-19.1.
roll_out=RC+V(20.1,0,0);pitch_out=PC+V(0,20.1,0)
roll_stat=RC+V(19.1,0,0);pitch_stat=PC+V(0,19.1,0)
parts={};groups={};materials={}
def add(name,shape,group,material='aluminum_candidate'):
    assert shape.isValid(),name
    assert len(shape.Solids)==1,(name,len(shape.Solids))
    parts[name]=shape;groups[name]=group;materials[name]=material

# Foot plate: independent mounting holes and four non-loadbearing boot-interface tabs.
foot=box(-75,-45,0,150,90,4)
foot_holes=[V(-56,y,2) for y in (-15,15)]+[V(0,y,2) for y in (-19,19)]
foot=drill(foot,foot_holes,V(0,0,1),1.7)
boot_holes=[V(x,y,2) for x in (-59,59) for y in (-36,36)]
foot=drill(foot,boot_holes,V(0,0,1),1.7)
for h in foot_holes+boot_holes:
    foot=foot.cut(Part.makeCone(3.2,1.7,1.5,V(h.x,h.y,0),V(0,0,1)))
add('FootPlate',foot,'fixed')

# Split low boot-interface rim: cosmetic PETG/PLA part only, not a landing absorber.
rim=box(-75,-45,4,150,90,8).cut(box(-72,-42,3,144,84,10))
for x in (-59,59):
    for y in (-36,36):
        rim=rim.fuse(box(x-5,31 if y>0 else -45,4,10,14,3))
rim=drill(rim,boot_holes,V(0,0,1),1.7)
add('BootInterfaceHeel',rim.common(box(-76,-46,3,75.8,92,10)).removeSplitter(),'fixed','PETG_or_PLA_non_loadbearing')
add('BootInterfaceToe',rim.common(box(.2,-46,3,75.8,92,10)).removeSplitter(),'fixed','PETG_or_PLA_non_loadbearing')

mount=union(ring(30,20,3,V(-53.1,0,45),V(1,0,0)),
    box(-53.1,-12,7,3,24,19),box(-62,-20,4,20,40,4))
rear_xz=[(23.5,0),(-23.5,0),(0,23.5),(0,-23.5)]
mount=drill(mount,centers(rear_xz,rp,-20.1),V(1,0,0))
mount=drill(mount,[V(-56,y,6) for y in (-15,15)],V(0,0,1),1.7)
add('RollStatorMount',mount,'fixed')

# Bearing carrier: rear shoulder, 26.04 diameter pocket, front removable retaining cap.
carrier=union(box(-5.9,-17,7,10,34,59),box(-5.9,-23,4,12,46,4))
carrier=carrier.cut(cyl(6,16,V(-7,0,45),V(1,0,0)))
carrier=carrier.cut(cyl(13.02,8.2,V(-3.9,0,45),V(1,0,0)))
carrier=drill(carrier,[V(0,y,6) for y in (-19,19)],V(0,0,1),1.7)
cap_centers=[V(4.1,y,45+z) for y in (-14,14) for z in (-14,14)]
carrier=drill(carrier,cap_centers,V(1,0,0),1.025)
add('RollBearingCarrier',carrier,'fixed')
cap=box(4.1,-17,28,2,34,34).cut(cyl(6,3,V(3.6,0,45),V(1,0,0)))
cap=drill(cap,cap_centers,V(1,0,0),1.4)
add('RollBearingCap',cap,'fixed')
add('Bearing6000_Envelope',ring(13,5,8,V(-3.9,0,45),V(1,0,0)),'fixed','bearing_envelope_only')

# Roll output adapter, bearing journal and intermediate yoke form a single solid concept.
adapter=cyl(18.5,3,roll_out,V(1,0,0))
adapter=union(adapter,ring(7.45,3,.5,roll_out-V(.5,0,0),V(1,0,0)),
              cyl(4.99,13.4,V(-7.4,0,45),V(1,0,0)))
adapter=drill(adapter,centers(output_xz,rp,20.1),V(1,0,0))
yoke=union(adapter,box(-9.9,-38,41,3,28,8),
     box(-9.9,-38,41,82.9,5,8),box(68,-38,41,5,60.1,8),
     ring(32,20,3,pitch_stat,V(0,1,0)))
if VERSION=='v09':
    # Additional top load path clears bearing carrier and pitch motor in the nominal layout.
    yoke=union(yoke,box(-9.9,-44,38,82.9,12,14),box(68,-44,38,5,66.1,14),
               box(-9.9,-8,58,3,16,27),box(-9.9,-8,77,56.9,16,8),
               box(31,6,73,16,16.1,8))
yoke=drill(yoke,centers(stator_xz,pp,19.1),V(0,1,0))
add('RollTransferYoke',yoke,'roll')

shank=union(cyl(18.5,3,pitch_out,V(0,1,0)),
    ring(7.45,3,.5,pitch_out-V(0,.5,0),V(0,1,0)),box(31,22.6,45,16,4.5,80))
shank=drill(shank,centers(output_xz,pp,20.1),V(0,1,0))
shank=drill(shank,[V(x,25,z) for x in (34,44) for z in (95,115)],V(0,1,0),1.7)
add('PitchOutputShankBracket',shank,'pitch')

# Closed envelopes with output boss and center bore; not converted vendor surfaces.
def motor_env(c,axis):
    body=union(cyl(26.5,39.2,c-axis*20.1,axis),cyl(18.5,1,c+axis*19.1,axis))
    return body.cut(cyl(7.5,2.1,c+axis*18.05,axis))
add('RollMotor_ClosedEnvelope',motor_env(RC,V(1,0,0)),'fixed','motor')
add('PitchMotor_ClosedEnvelope',motor_env(PC,V(0,1,0)),'roll','motor')

def move(s,group,r,p):
    s=s.copy()
    if group=='pitch':s.rotate(PC,V(0,1,0),p)
    if group in ('roll','pitch'):s.rotate(RC,V(1,0,0),r)
    return s
rows=[];worst={};penetrations=[]
for r,p in itertools.product((-20,-10,0,10,20),(-45,-30,-15,0,15,30,45)):
    posed={n:move(s,groups[n],r,p) for n,s in parts.items()}
    hits=[]
    for (na,a),(nb,b) in itertools.combinations(posed.items(),2):
        # Bounding-box precheck avoids expensive Booleans for remote parts.
        ba,bb=a.BoundBox,b.BoundBox
        if any(getattr(ba,k+'Max')<getattr(bb,k+'Min')-1e-7 or getattr(bb,k+'Max')<getattr(ba,k+'Min')-1e-7 for k in 'XYZ'):continue
        vol=a.common(b).Volume
        if vol>1e-4:hits.append({'parts':[na,nb],'volume_mm3':vol})
    rows.append({'roll_deg':r,'pitch_deg':p,'penetrations':hits})
    penetrations.extend(hits)

doc=A.newDocument('Wukong_Ankle_Structure_'+VERSION);exports=[];mesh=[]
colors={'fixed':(.42,.5,.6),'roll':(.82,.55,.2),'pitch':(.72,.75,.79)}
for name,shape in parts.items():
    o=doc.addObject('PartDesign::Feature',name);o.Shape=shape
    o.addProperty('App::PropertyString','RigidGroup');o.RigidGroup=groups[name]
    o.addProperty('App::PropertyString','DesignStatus');o.DesignStatus='CONCEPT ONLY - strength/fit/fasteners pending'
    if name.endswith('ClosedEnvelope'):continue
    exports.append(o)
for name,placement in [('RollMotor_Vendor',rp),('PitchMotor_Vendor',pp)]:
    o=doc.addObject('PartDesign::Feature',name);o.Shape=native(vendor,placement);exports.append(o)
doc.recompute();doc.saveAs(str(OUT/('wukong_ankle_structure_'+VERSION+'.FCStd')))
Part.export(exports,str(OUT/('wukong_ankle_structure_'+VERSION+'.step')))
Part.export([doc.getObject('RollTransferYoke')],str(OUT/'RollTransferYoke.step'))
for o in exports:
    points,tri=o.Shape.tessellate(.35)
    color=(.9,.62,.16) if o.Name.startswith('BootInterface') else colors.get(groups.get(o.Name),(.18,.22,.28))
    mesh.append({'name':o.Name,'color':list(color),
                 'points':[[p.x,p.y,p.z] for p in points],'triangles':[list(t) for t in tri]})
(OUT/'render_mesh.json').write_text(json.dumps(mesh)+'\n')
for name in ('BootInterfaceHeel','BootInterfaceToe'):
    parts[name].exportStl(str(OUT/(name+'_fit_check_only.stl')))
report={'status':'solid_structural_concept_not_released','variant':VERSION,'dimensions_mm':{'foot':[150,90,4],
    'roll_center':list(RC),'pitch_center':list(PC),'bearing':[10,26,8]},
 'interfaces':{'output_pcd_mm':27,'output_hole_count':3,'output_thread':'M2.5','drawing_output_depth_mm':3,
               'stator_pcd_mm':47.5,'stator_hole_count':3,'stator_thread':'M2.5','drawing_stator_depth_mm':5,
               'adapter_thickness_mm':3,'candidate_screw_length_mm':6,'assumed_washer_mm':.5,
               'nominal_engagement_mm':2.5,'output_depth_margin_mm':.5,'rear_pcd_mm':47,'rear_hole_count':4},
 'part_volumes_mm3':{n:s.Volume for n,s in parts.items()},'sample_count':len(rows),'samples':rows,
 'penetration_frame_count':sum(bool(x['penetrations']) for x in rows),
 'assumptions':['No fastener strength or tolerance stack verification','CNC aluminum geometry candidate, not printed load-bearing approval',
    '6000 shown as annulus, not internal bearing geometry; fits not released',
    'All solid joints use nominal contacts; no bolt pretension, stress, fatigue or thermal analysis',
    'Cable/plug sweeps and boot shell still pending','35 samples only; not continuous collision certification',
    'Factory output side determined from STEP PCD27 and 37mm face; previous v07 output orientation was only assumed']}
(OUT/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'parts':len(parts),'sample_count':len(rows),'penetration_frames':report['penetration_frame_count'],
                  'first_penetrations':penetrations[:12]},indent=2))

"""Run with FreeCAD's bundled Python. Vendor surfaces + closed envelope draft.
Units mm; poses degrees. Not manufacturing release, not a trainable robot.
"""
import FreeCAD as App
import Part
import json, hashlib, itertools
import argparse
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument('--layout',choices=['stacked','axial'],default='stacked')
args=parser.parse_args(); axial=args.layout=='axial'
OUT=ROOT/'09_cad'/('ankle_v07_axial' if axial else 'ankle_v07')
OUT.mkdir(exist_ok=True)
V=App.Vector
SOURCE=ROOT/'00_inputs/actuators/AK40_V3/model/AK40-10 V3.0 KV170.STEP'
source=Part.read(str(SOURCE)); bb=source.BoundBox
# STEP principal axis is Y. Center by measured axial extents, not an output datum.
cy=(bb.YMin+bb.YMax)/2
base=source.copy();base.translate(V(0,-cy,0))
RADIUS=26.5; LENGTH=40.2
ROLL=V(-30,0,38) if axial else V(0,0,33)
PITCH=V(30,0,38) if axial else V(0,0,91)

def placed(shape,axis,center):
    x=shape.copy();x.Placement=App.Placement(center,App.Rotation(V(0,1,0),axis));return x

raw_roll=placed(base,V(1,0,0),ROLL)
raw_pitch=placed(base,V(0,1,0),PITCH)
foot=Part.makeBox(120,70,4,V(-60,-35,0))
roll_env=Part.makeCylinder(RADIUS,LENGTH,ROLL-V(LENGTH/2,0,0),V(1,0,0))
pitch_env=Part.makeCylinder(RADIUS,LENGTH,PITCH-V(0,LENGTH/2,0),V(0,1,0))
# Shank clearance proxy offset laterally from pitch motor. Mount/yoke intentionally not designed.
shank=Part.makeCylinder(10,70,PITCH+V(0,31,36),V(0,0,1))

def transform(shape,roll,pitch=0,shank_member=False):
    x=shape.copy()
    if shank_member:x.rotate(PITCH,V(0,1,0),pitch)
    x.rotate(ROLL,V(1,0,0),roll)
    return x

rows=[];minimum={}; max_penetration=0
for r,p in itertools.product(range(-20,21,5),range(-45,46,15)):
    shapes={'foot':foot,'roll_envelope':roll_env,
            'pitch_envelope':transform(pitch_env,r),
            'shank_proxy':transform(shank,r,p,True)}
    clashes=[]
    for (na,a),(nb,b) in itertools.combinations(shapes.items(),2):
        volume=a.common(b).Volume
        dist=a.distToShape(b)[0]
        key=na+'__'+nb
        if key not in minimum or dist<minimum[key]['distance_mm']:
            minimum[key]={'distance_mm':dist,'roll_deg':r,'pitch_deg':p}
        if volume>1e-5:clashes.append({'pair':key,'volume_mm3':volume})
        max_penetration=max(max_penetration,volume)
    rows.append({'roll_deg':r,'pitch_deg':p,'clashes':clashes,
                 'upper_pitch_center_z_mm':(App.Rotation(V(1,0,0),r).multVec(PITCH-ROLL)+ROLL).z})

doc=App.newDocument('Wukong_Ankle_v07')
meta=doc.addObject('App::FeaturePython','DesignParameters')
for name,value in [('MotorDiameter',53),('MotorLength',40.2),('RollAxisHeight',ROLL.z),
                   ('PitchAxisHeight',PITCH.z),('FootLength',120),('FootWidth',70),('FootThickness',4)]:
    meta.addProperty('App::PropertyLength',name,'Draft parameters');setattr(meta,name,value)
meta.addProperty('App::PropertyString','Status');meta.Status='PARAMETER RECORD ONLY; rerun script to rebuild; NO MANUFACTURING RELEASE'

def obj(name,shape,color,trans=0):
    o=doc.addObject('PartDesign::Feature',name);o.Shape=shape
    if App.GuiUp:
        o.ViewObject.ShapeColor=color;o.ViewObject.Transparency=trans
    return o

objects=[obj('FootPlate_Concept',foot,(.28,.35,.42)),
         obj('RollMotor_VendorSurface',raw_roll,(.2,.25,.32)),
         obj('PitchMotor_VendorSurface',raw_pitch,(.2,.25,.32)),
         obj('Shank_ClearanceProxy',shank,(.55,.6,.65))]
envs=[obj('Roll_ClosedEnvelope',roll_env,(1.,.5,.1),80),obj('Pitch_ClosedEnvelope',pitch_env,(1.,.5,.1),80)]
axes=[obj('RollAxis_Schematic',Part.makeCylinder(.7,120,ROLL-V(30,0,0),V(1,0,0)),(1.,.15,.15)),
      obj('PitchAxis_Schematic',Part.makeCylinder(.7,80,PITCH-V(0,40,0),V(0,1,0)),(.1,.8,.2))]
# Only connectivity diagram: deliberately not a load-bearing bracket.
wire=obj('RequiredYoke_Schematic',Part.makePolygon([ROLL+V(23.1,0,0),ROLL+V(30,0,0),
    V(ROLL.x+30,-24,PITCH.z),PITCH+V(0,-24,0)]),(.95,.6,.1))
doc.recompute()
save=OUT/'wukong_ankle_v07.FCStd';doc.saveAs(str(save))
Part.export(objects,str(OUT/'wukong_ankle_v07.step'))
report={'status':'vendor_surface_and_envelope_draft','layout':args.layout,'source_url':'https://www.cubemars.com/data/cms/202607/ak40-10-v3-0-kv170-3d-drawing.zip',
 'source_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
 'source_geometry':{'faces':len(source.Faces),'shells':len(source.Shells),'solids':len(source.Solids),'valid':source.isValid(),
                    'bounds_mm':[bb.XMin,bb.XMax,bb.YMin,bb.YMax,bb.ZMin,bb.ZMax]},
 'dimensions_mm':{'foot':[120,70,4],'roll_axis_xyz':list(ROLL),'pitch_axis_xyz':list(PITCH),
                  'center_separation':(PITCH-ROLL).Length,'neutral_motor_top_z':max(PITCH.z,ROLL.z)+26.5,'motor_diameter':53,'motor_length':40.2},
 'actuator_mass_only_kg':.380,'sample_count':len(rows),'max_common_volume_mm3':max_penetration,
 'minimum_distances':minimum,'samples':rows,
 'warnings':['Vendor STEP is open surfaces; no volume/inertia inferred from it.',
   'Collision test uses published outer cylindrical dimensions, not vendor surface intersection.',
   '63 pose grid only; no continuous collision proof or validated safe ROM.',
   'Yoke drawn as a line, not manufactured: attachment holes, bearing support and loads unresolved.',
   'Shank is a clearance cylinder, not a finalized link.',
   'Envelope checks exclude mounts, screws, connectors/cable sweeps and other robot parts.',
   'Parameter properties are records; edit script and rerun for geometry changes.',
   'Earlier ankle pitch datum was 25mm; this layout changes both datum and/or longitudinal placement, so recompute whole-body geometry.']}
(OUT/'verification.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
assert all(s.isValid() and len(s.Solids)>0 for s in (foot,roll_env,pitch_env,shank))
# Reload the saved document to verify persistence independently of the original in-memory tree.
App.closeDocument(doc.Name)
reloaded=App.openDocument(str(save))
assert len(reloaded.Objects)==10
assert reloaded.FootPlate_Concept.Shape.isValid()
print(json.dumps({'sample_count':len(rows),'clash_frames':sum(bool(x['clashes']) for x in rows),
                   'minimum_distances':minimum,'saved':str(save)},indent=2))

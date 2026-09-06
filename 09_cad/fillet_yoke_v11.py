"""Targeted transition-radius candidates; preserve v09 and check assembled geometry."""
import FreeCAD as A,Part,json,itertools
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/ankle_v11';OUT.mkdir(exist_ok=True)
doc=A.openDocument(str(ROOT/'09_cad/ankle_v09/wukong_ankle_structure_v09.FCStd'))
source=doc.RollTransferYoke.Shape
selected=[]
for e in source.Edges:
    if len(e.Vertexes)!=2:continue
    a,b=[v.Point for v in e.Vertexes]
    if abs(a.y-b.y)>1e-5 or abs(a.z-b.z)>1e-5 or abs(e.Length-3)>1e-5:continue
    c=e.CenterOfMass
    if (abs(abs(c.y)-8)<1e-4 and abs(c.z-61.6808273)<1e-4) or (abs(c.y+32)<1e-4 and min(abs(c.z-41),abs(c.z-49))<1e-4):
        selected.append(e)
assert len(selected)==4
candidates=[];shapes={}
for radius in (.5,1.,1.5,2.):
    try:
        shape=source.makeFillet(radius,selected)
        valid=shape.isValid() and len(shape.Solids)==1
        candidates.append({'radius_mm':radius,'valid_single_solid':valid,'volume_mm3':shape.Volume,
                           'delta_mass_g_at_2700':(shape.Volume-source.Volume)*.0027})
        if valid:shapes[radius]=shape
    except Exception as e:candidates.append({'radius_mm':radius,'error':str(e)})
assert shapes
# Largest tested successful radius is a candidate, not stress-optimized by geometry alone.
radius=max(shapes);doc.RollTransferYoke.Shape=shapes[radius];doc.recompute()
parts={o.Name:o.Shape for o in doc.Objects if hasattr(o,'Shape') and 'Vendor' not in o.Name}
groups={o.Name:o.RigidGroup for o in doc.Objects if o.Name in parts}
RC=A.Vector(-30,0,45);PC=A.Vector(39,0,45)
rows=[]
for r,p in itertools.product((-20,-10,0,10,20),(-45,-30,-15,0,15,30,45)):
    posed={}
    for name,shape in parts.items():
        s=shape.copy()
        if groups[name]=='pitch':s.rotate(PC,A.Vector(0,1,0),p)
        if groups[name] in ('roll','pitch'):s.rotate(RC,A.Vector(1,0,0),r)
        posed[name]=s
    hits=[]
    for (na,a),(nb,b) in itertools.combinations(posed.items(),2):
        ba,bb=a.BoundBox,b.BoundBox
        if any(getattr(ba,k+'Max')<getattr(bb,k+'Min')-1e-7 or getattr(bb,k+'Max')<getattr(ba,k+'Min')-1e-7 for k in 'XYZ'):continue
        v=a.common(b).Volume
        if v>1e-4:hits.append({'parts':[na,nb],'volume_mm3':v})
    rows.append({'roll':r,'pitch':p,'penetrations':hits})
assert not any(row['penetrations'] for row in rows)
doc.saveAs(str(OUT/'wukong_ankle_structure_v11.FCStd'))
exports=[o for o in doc.Objects if hasattr(o,'Shape') and not o.Name.endswith('ClosedEnvelope')]
Part.export(exports,str(OUT/'wukong_ankle_structure_v11.step'))
Part.export([doc.RollTransferYoke],str(OUT/'RollTransferYoke.step'))
result={'status':'transition_candidate_not_manufacturing_release','radius_candidates':candidates,
        'selected_radius_mm':radius,'edge_count':len(selected),'sample_count':35,'samples':rows}
(OUT/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'candidates':candidates,'selected':radius,'penetration_frames':0},indent=2))

import FreeCAD as A, Part,json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/ankle_v08'
doc=A.openDocument(str(OUT/'wukong_ankle_structure_v08.FCStd'))
names=['FootPlate','RollStatorMount','RollBearingCarrier','RollBearingCap','Bearing6000_Envelope',
       'RollTransferYoke','PitchOutputShankBracket','BootInterfaceHeel','BootInterfaceToe']
parts=[]
for n in names:
    s=doc.getObject(n).Shape
    assert s.isValid() and len(s.Solids)==1 and s.Volume>0,n
    Part.export([doc.getObject(n)],str(OUT/(n+'.step')))
    parts.append(dict(name=n,volume_mm3=s.Volume,valid=True,solids=len(s.Solids)))
r=json.loads((OUT/'verification.json').read_text())
assert r['sample_count']==35 and r['penetration_frame_count']==0
assert r['interfaces']['nominal_engagement_mm']<r['interfaces']['drawing_output_depth_mm']
observed=json.loads((ROOT/'00_inputs/actuators/AK40_V3/cylindrical_interfaces.json').read_text())
output={(round(x['center'][0],4),round(x['center'][2],4)) for x in observed if abs(x['radius']-1.025)<1e-4 and x['y_min']<-13.9}
assert len(output)==3
assert all(math.isclose(math.hypot(x,z),13.5,abs_tol=1e-4) for x,z in output)
report={'parts':parts,'vendor_output_hole_center_check':'3 on PCD27 confirmed from STEP',
        'screw_engagement_note':'nominal only; plate/washer/bolt tolerances and strength not evaluated',
        'sampling':'35 recorded poses, no geometric penetrations in closed proxy parts'}
(OUT/'reload_checks.json').write_text(json.dumps(report,indent=2)+'\n')
print('PASS: saved document reloaded; 9 connected solid parts; PCD27 source check; 35 geometric poses.')
print('ALUMINUM_CONCEPT_MASS_KG',sum(doc.getObject(n).Shape.Volume for n in names[:4]+names[5:7])*2.7e-6)

"""Extract visualization triangles from saved FreeCAD geometry, no geometry editing."""
import FreeCAD as App
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'09_cad/ankle_v07_axial'
doc=App.openDocument(str(OUT/'wukong_ankle_v07.FCStd'))
objects=[]
for name,color in [('FootPlate_Concept','#6b849a'),('RollMotor_VendorSurface','#3f4958'),
                   ('PitchMotor_VendorSurface','#586376'),('Shank_ClearanceProxy','#c3cbd2')]:
    shape=doc.getObject(name).Shape
    points,faces=shape.tessellate(.25)
    objects.append({'name':name,'color':color,'points':[[v.x,v.y,v.z] for v in points],
                    'triangles':[list(f) for f in faces]})
(OUT/'render_mesh.json').write_text(json.dumps(objects)+'\n')
print('Exported saved-document mesh triangles:',sum(len(o['triangles']) for o in objects))

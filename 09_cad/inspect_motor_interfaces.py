import FreeCAD as App, Part, json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
s=Part.read(str(ROOT/'00_inputs/actuators/AK40_V3/model/AK40-10 V3.0 KV170.STEP'))
rows=[]
for i,f in enumerate(s.Faces):
    surf=f.Surface
    if isinstance(surf,Part.Cylinder) and abs(surf.Axis.y)>.999:
        b=f.BoundBox
        rows.append(dict(face=i,radius=surf.Radius,axis=list(surf.Axis),center=list(surf.Center),
                         y_min=b.YMin,y_max=b.YMax))
out=ROOT/'00_inputs/actuators/AK40_V3/cylindrical_interfaces.json'
out.write_text(json.dumps(rows,indent=2)+'\n')
print(json.dumps(rows,indent=2))

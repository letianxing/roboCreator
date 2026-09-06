import FreeCAD as A, Part,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
s=Part.read(str(ROOT/'09_cad/ankle_v09/RollTransferYoke.step'))
rows=[]
for i,e in enumerate(s.Edges):
    if len(e.Vertexes)!=2:continue
    a,b=[v.Point for v in e.Vertexes]
    if abs(a.y-b.y)<1e-5 and abs(a.z-b.z)<1e-5 and abs(a.x-b.x)>2.5 and -11<e.CenterOfMass.x<-5:
        rows.append({'edge':i+1,'endpoints':[list(a),list(b)],'length':e.Length})
print(json.dumps(rows,indent=2))

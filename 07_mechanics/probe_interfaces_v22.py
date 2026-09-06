"""Exact nearest-envelope gaps from saved CAD; not mounting/insulation/thermal approval."""
import json,sys
from pathlib import Path
import FreeCAD as A
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'09_cad'))
from geometry_v22 import g,box_gap,OUT
rows=[]
for name in ('lift_h114','lift_h117','lift_h119','stand_candidate'):
    doc=A.openDocument(str(OUT/(name+'.FCStd')));parts={o.Name:o.Shape.copy() for o in doc.Objects if hasattr(o,'Shape')};A.closeDocument(doc.Name);bb=g.boxes(parts);closest=[]
    for component in ('battery','power','computer'):
        pairs=sorted((box_gap(bb[component],bb[n]),n) for n in parts if n!=component);best=float('inf');other=None
        for lower,n in pairs:
            if lower>best+1e-6:break
            distance=parts[component].distToShape(parts[n])[0]
            if distance<best:best=distance;other=n
        closest.append({'component':component,'nearest_part':other,'distance_mm':best})
    result={'scene':name,'nearest_envelope_gaps':closest}
    if name=='stand_candidate':
        result['battery_bottom_to_pelvis_top_mm']=parts['battery'].BoundBox.ZMin-parts['PelvisPlate'].BoundBox.ZMax
        result['power_top_to_pelvis_bottom_mm']=parts['PelvisPlate'].BoundBox.ZMin-parts['power'].BoundBox.ZMax
    rows.append(result);print(name,closest,flush=True)
(ROOT/'07_mechanics/electronics_interfaces_v22.json').write_text(json.dumps({'scenes':rows,'mounts_cables_thermal_and_insulation_validated':False},indent=2)+'\n',encoding='utf-8')

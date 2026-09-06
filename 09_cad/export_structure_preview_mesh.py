"""Export visualization mesh from a saved structural CAD without modifying its geometry."""
import FreeCAD as A,json,argparse
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--variant',default='v11',choices=['v09','v11']);args=p.parse_args()
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad'/('ankle_'+args.variant)
doc=A.openDocument(str(OUT/('wukong_ankle_structure_'+args.variant+'.FCStd')))
data=[]
for o in doc.Objects:
    if not hasattr(o,'Shape') or o.Name.endswith('ClosedEnvelope'):continue
    pts,tri=o.Shape.tessellate(.3)
    color=(.43,.51,.6)
    if 'Vendor' in o.Name:color=(.18,.22,.28)
    if o.Name=='RollTransferYoke':color=(.82,.55,.2)
    if o.Name.startswith('BootInterface'):color=(.9,.62,.16)
    data.append({'name':o.Name,'color':list(color),'points':[list(v) for v in pts],'triangles':[list(t) for t in tri]})
(OUT/'render_mesh.json').write_text(json.dumps(data)+'\n')
print('Saved mesh from',len(data),'CAD objects')

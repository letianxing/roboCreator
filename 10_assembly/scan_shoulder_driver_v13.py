import FreeCAD as A,Part,json,math,itertools,re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'10_assembly/shoulder_mount_v13';OUT.mkdir(exist_ok=True)
data=json.loads((ROOT/'09_cad/fullbody_v13/wholebody_results.json').read_text())['results']
docs={p['pose']:A.openDocument(str(ROOT/'09_cad/fullbody_v13'/p['pose']/'wukong_fullbody_v13.FCStd')) for p in data}
rows=[]
for offset in (80,-80,-100,-120):
    counts={'hardware':0,'shell':0};cases=[]
    for p in data:
        doc=docs[p['pose']];names=[c['name'] for c in p['components'] if c['category'] in ('motor','hardware')]
        shapes={n:doc.getObject(n).Shape.copy() for n in names}
        for side in ('L','R'):
            n=side+'_arm_motor_1';j=next(j for j in p['joints'] if j['name']==side+'_arm_1')
            axis=A.Vector(*j['axis']);axis=axis if side=='L' else -axis
            target=A.Vector(*j['origin_mm'])+axis*math.copysign(abs(offset)-25.1,offset)
            cm=A.Vector(*next(c['com_mm'] for c in p['components'] if c['name']==n))
            shapes[n].translate(target-cm)
        hits=[]
        for side in ('L','R'):
            n=side+'_arm_motor_1';a=shapes[n]
            for c in p['components']:
                if c['name']==n:continue
                if c['category'] not in ('motor','hardware','shell'):continue
                b=shapes[c['name']] if c['name'] in shapes else doc.getObject(re.sub(r'[^A-Za-z0-9_]','_',c['name'])).Shape
                ba,bb=a.BoundBox,b.BoundBox
                if any(getattr(ba,k+'Max')<getattr(bb,k+'Min') or getattr(bb,k+'Max')<getattr(ba,k+'Min') for k in 'XYZ'):continue
                v=a.common(b).Volume
                if v>1e-2:
                    typ='shell' if c['category']=='shell' else 'hardware';counts[typ]+=1
                    hits.append({'parts':[n,c['name']],'kind':typ,'volume_mm3':v})
        cases.append({'pose':p['pose'],'hits':hits})
    row={'outboard_mm':offset,'counts':counts,'cases':cases};rows.append(row);print(offset,counts,flush=True)
(OUT/'scan.json').write_text(json.dumps({'status':'placement_scan_only','results':rows},indent=2)+'\n')

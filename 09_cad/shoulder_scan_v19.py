"""Search shoulder motor packages against integrated pose obstacles using conservative boxes."""
import FreeCAD as A, Part, json, math, itertools, hashlib
from pathlib import Path
import numpy as np
from reuse_body_v19 import load_body
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/body_v19';OUT.mkdir(exist_ok=True)
PF=ROOT/'03_system/shoulder_recovery_v19.json';cfg=json.loads(PF.read_text(encoding='utf-8'));env=load_body();env['OUT']=OUT;V=A.Vector
def ro(axis,deg):return A.Rotation(V(*axis),deg)
def repack(parts,H,B):
    plan=json.loads((ROOT/'03_system/electronics_repack_v18.json').read_text())
    for row in plan['changes']:
        s=parts[row['component']].copy();center=env['volcenter'](s)
        if row.get('rotation_world_z_deg'):s.rotate(center,B.multVec(V(0,0,1)),row['rotation_world_z_deg'])
        s.translate(B.multVec(V(*row['translation_world_mm'])));parts[row['component']]=s
    # Combine intended same-material ear/head joins, retaining total mass elsewhere.
    if 'WukongHeadShell' in parts:
        s=parts['WukongHeadShell']
        for n in ('MonkeyEarm1','MonkeyEar1'):s=s.fuse(parts[n])
        parts['WukongHeadShell']=s.removeSplitter();parts.pop('MonkeyEarm1');parts.pop('MonkeyEar1')
def make_scenes():
    stand=next(p for p in env['base']['cfg']['poses'] if p['name']=='stand')
    contact=json.loads((ROOT/'04_kinematics/body_contacts_v18.json').read_text());previous=json.loads((ROOT/'09_cad/body_v18/body_results.json').read_text())
    idx=previous['scenes'][2]['selected_leg_screen']['index'];kneel=contact['kneeling_candidates'][idx]['pose']
    arm=contact['arm_candidate'];scenes=[]
    for name,pose,q in [('stand',stand,cfg['stand_arm_candidates_deg'][0]),('cuff_knees',kneel,arm['upstream_q_deg'])]:
        parts,info,joints,nodes=env['body'](pose,'narrow',{'upstream_q_deg':q});H=V(*joints[2]['origin_mm']);beta=pose['foot_pitch_deg']-pose['q_deg'][5]-pose['q_deg'][3]-pose['q_deg'][2];B=ro((0,1,0),beta)
        repack(parts,H,B)
        # Discard old arms for search; final exact audit will restore all arm components.
        fixed={n:s for n,s in parts.items() if not n.startswith(('L_Shoulder','R_Shoulder','L_Elbow','R_Elbow','L_UpperArm','R_UpperArm','L_Forearm','R_Forearm','L_Wrist','R_Wrist','L_Hand','R_Hand','L_Gripper','R_Gripper','L_Gauntlet','R_Gauntlet'))}
        scenes.append({'name':name,'H':H,'B':B,'q':q,'parts':fixed,'pose':pose,'arm':arm})
    return scenes
def motors(scene,q,offsets,clocks):
    origin=scene['H']+scene['B'].multVec(V(*cfg['shoulder_origin_body_mm']));R=scene['B'];result=[]
    for i,(axis,off,clock) in enumerate(zip([(0,1,0),(1,0,0),(0,0,1)],offsets,clocks)):
        orientation=R.multiply(A.Rotation(V(0,0,1),V(*axis))).multiply(ro((0,0,1),clock));T=A.Placement(origin+R.multVec(V(*axis)*off),orientation)
        s=env['placed'](env['base']['xm'],T);result.append(s);R=R.multiply(ro(axis,q[i]))
    return result
def boxof(s):
    b=s.BoundBox;return np.array([b.XMin,b.YMin,b.ZMin]),np.array([b.XMax,b.YMax,b.ZMax])
def motor_boxes(scene,q,offsets,clocks):
    # Transform eight corners of the cached supplier box for conservative coarse screening.
    origin=scene['H']+scene['B'].multVec(V(*cfg['shoulder_origin_body_mm']));R=scene['B'];result=[]
    corners=[V(x,y,z) for x in (-14.25,14.25) for y in (-35.25,11.25) for z in (-19,23.7)]
    for i,(axis,off,clock) in enumerate(zip([(0,1,0),(1,0,0),(0,0,1)],offsets,clocks)):
        orientation=R.multiply(A.Rotation(V(0,0,1),V(*axis))).multiply(ro((0,0,1),clock));T=A.Placement(origin+R.multVec(V(*axis)*off),orientation)
        points=np.array([list(T.multVec(v)) for v in corners]);result.append((points.min(axis=0),points.max(axis=0)));R=R.multiply(ro(axis,q[i]))
    return result
def overlap(a,b):return bool(np.all(np.minimum(a[1],b[1])-np.maximum(a[0],b[0])>0))
def main():
    scenes=make_scenes();fixed=[]
    for sc in scenes:
        fixed.append([(n,boxof(s)) for n,s in sc['parts'].items() if not n.startswith(('R_', 'Monkey')) and n not in ('WukongHeadShell','CrownOccupancy')])
    rows=[]
    for offsets in itertools.product(cfg['shoulder_pitch_offsets_mm'],cfg['shoulder_roll_offsets_mm'],cfg['shoulder_yaw_offsets_mm']):
        for clocks in itertools.product(cfg['body_clockings_deg'],repeat=3):
            beststand=None
            for q in cfg['stand_arm_candidates_deg']:
                bb=motor_boxes(scenes[0],q,offsets,clocks)
                hits=[(i,n) for i,b in enumerate(bb) for n,f in fixed[0] if overlap(b,f)]+[(i,'shoulder'+str(j)) for i,j in itertools.combinations(range(3),2) if overlap(bb[i],bb[j])]
                width=2*max(b[1][1] for b in bb);score=(len(hits),width)
                if beststand is None or score<beststand[0]:beststand=(score,q,hits)
            q=scenes[1]['q'];bb=motor_boxes(scenes[1],q,offsets,clocks)
            hits=[(i,n) for i,b in enumerate(bb) for n,f in fixed[1] if overlap(b,f)]+[(i,'shoulder'+str(j)) for i,j in itertools.combinations(range(3),2) if overlap(bb[i],bb[j])]
            rows.append({'offsets_mm':list(offsets),'clock_deg':list(clocks),'stand_q_deg':beststand[1], 'stand_box_hits':beststand[2],'kneel_box_hits':hits,
              'box_score':len(beststand[2])+len(hits),'shoulder_width_reference_mm':beststand[0][1]})
    rows.sort(key=lambda r:(r['box_score'],r['shoulder_width_reference_mm']))
    (OUT/'shoulder_coarse_scan.json').write_text(json.dumps({'rows':rows,'scope':'conservative transformed manufacturer bounding boxes'},indent=2)+'\n')
    print('coarse screening finished',len(rows),'best score',rows[0]['box_score'],flush=True)
    # Exact positive volumes for a finite shortlist; shells included now.
    exact=[]
    for idx,candidate in enumerate(rows[:8]):
        errors=[]
        for i,sc in enumerate(scenes):
            q=candidate['stand_q_deg'] if i==0 else sc['q'];shapes=motors(sc,q,candidate['offsets_mm'],candidate['clock_deg'])
            for j,s in enumerate(shapes):
                for n,t in sc['parts'].items():
                    if n.startswith('R_'):continue
                    if s.BoundBox.intersect(t.BoundBox):
                        v=s.common(t).Volume
                        if v>1e-5:errors.append({'scene':sc['name'],'motor':j,'other':n,'volume_mm3':v})
                for k in range(j):
                    v=s.common(shapes[k]).Volume
                    if v>1e-5:errors.append({'scene':sc['name'],'motor':j,'other':'shoulder'+str(k),'volume_mm3':v})
        exact.append({**candidate,'exact_collisions':errors})
        print('exact shortlist',idx+1,'collisions',len(errors),flush=True)
    exact.sort(key=lambda r:(len(r['exact_collisions']),r['shoulder_width_reference_mm']))
    out={'input_sha256':hashlib.sha256(PF.read_bytes()).hexdigest(),'search_count':len(rows),'coarse_rows':rows,'exact_shortlist':exact,'selected':exact[0],
       'scope':'Search covers three shoulder motor packages vs fixed obstacles in two poses. Full arm, mounts and trajectory are checked separately.'}
    (OUT/'shoulder_scan.json').write_text(json.dumps(out,indent=2)+'\n');print('scanned',len(rows),'best',exact[0],flush=True)
if __name__=='__main__':main()

"""Exact solid screening before arm placement; retain constant-body collisions."""
import json,itertools,argparse,time
from geometry_v21 import Geometry,ROOT,V
g=Geometry()
rows=sum([json.loads((ROOT/'04_kinematics/recovery_v21'/f).read_text())['cases'] for f in ['coupled_search.json','forward_extension.json']],[])
rows=[r for r in rows if r['ik_found']];cache={};results=[]
def collide_changed(parts,info,changed):
    out=[];boxes=g.boxes(parts)
    for a,b in itertools.combinations(parts,2):
        if a not in changed and b not in changed:continue
        sa,sb=parts[a],parts[b]
        if boxes[a].intersect(boxes[b]):
            try:
                common=sa.common(sb)
                if common.isNull():raise ValueError('empty common result')
                v=common.Volume
            except Exception as error:
                distance=sa.distToShape(sb)[0]
                if distance>1e-6:continue
                out.append({'a':a,'b':b,'volume_mm3':0,'geometry_error':str(error),'distance_mm':distance});continue
            if v>1e-5:out.append({'a':a,'b':b,'volume_mm3':v})
    return out
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--replan',action='store_true');parser.add_argument('--midpitch',action='store_true');parser.add_argument('--power',action='store_true');args=parser.parse_args()
    global rows
    filename='leg_screen.json'
    if args.replan:
        source=json.loads((ROOT/'04_kinematics/recovery_v21/knee_replan.json').read_text());rows=source['cases'];filename='leg_replan_screen.json'
    if args.midpitch:
        source=json.loads((ROOT/'04_kinematics/recovery_v21/knee_midpitch.json').read_text());rows=[r for r in source['cases'] if r['ik_found']];filename='leg_midpitch_screen.json'
    if args.power:
        source=json.loads((ROOT/'04_kinematics/recovery_v21/power_refit.json').read_text());rows=[r for r in source['cases'] if r['ik_found']];filename='leg_power_screen.json'
    for i,r in enumerate(rows):
        started=time.perf_counter()
        key=(r['h_mm'],r['beta_deg'])
        if key not in cache:
            p,m=g.fixed(r)
            cache[key]=(p,m,collide_changed(p,m,set(p)))
        p0,m0,constant=cache[key];parts=dict(p0);info=dict(m0);lp,lm,j,f=g.leg(r['left_pose'])
        changed=set()
        for n,s in lp.items():
            s=s.copy();s.translate(V(0,60,0));parts['L_'+n]=s;info['L_'+n]={**lm[n],'group':'L_leg'};changed.add('L_'+n)
        boxes=g.boxes(parts)
        ground={n:parts[n].BoundBox.ZMin for n,b in boxes.items() if b.ZMin<-.5 and parts[n].BoundBox.ZMin<-.5}
        collision=collide_changed(parts,info,changed)
        out={'id':r['id'],'input':r,'ground_penetrations_mm':ground,'left_related_collisions':collision,'constant_body_collisions':constant,
             'leg_and_fixed_body_geometry_pass':not ground and not collision and not constant,'arms_checked':False}
        results.append(out)
        (g.out/filename).write_text(json.dumps({'cases':results,'expected_cases':len(rows),'completed':i+1==len(rows)},indent=2)+'\n',encoding='utf-8')
        print('leg',i+1,'/',len(rows),'id',r['id'],'ground',len(ground),'collision',len(collision),'constant',len(constant),'seconds',round(time.perf_counter()-started,2),flush=True)
    print('Passed partial screen',[r['id'] for r in results if r['leg_and_fixed_body_geometry_pass']],flush=True)
if __name__=='__main__':main()

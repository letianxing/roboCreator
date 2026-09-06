"""Whole-body arm branch checks on leg-screen survivors, with explicit static support."""
import json,sys,itertools,argparse
from pathlib import Path
import FreeCAD as A,Part
import numpy as np
from geometry_v21 import Geometry,ROOT,V
sys.path.insert(0,str(ROOT/'04_kinematics'));sys.path.insert(0,str(ROOT/'05_dynamics'))
from arm_candidates_v21 import candidates
from support_balance_v19 import reactions
g=Geometry()

def collide(parts,changed,first_only=False):
    bb=g.boxes(parts);out=[]
    for a,b in itertools.combinations(parts,2):
        if a not in changed and b not in changed:continue
        if bb[a].intersect(bb[b]):
            try:
                common=parts[a].common(parts[b])
                if common.isNull():raise ValueError('empty common result')
                vol=common.Volume
            except Exception as error:
                distance=parts[a].distToShape(parts[b])[0]
                if distance>1e-6:continue
                out.append({'a':a,'b':b,'volume_mm3':0,'geometry_error':str(error),'distance_mm':distance})
                if first_only:return out
                continue
            if vol>1e-5:
                out.append({'a':a,'b':b,'volume_mm3':vol})
                if first_only:return out
    return out

def mass_com(parts,info):
    mass=sum(r['mass_kg'] for r in info.values() if r['mass_kg'] is not None);com=V()
    for n,r in info.items():
        if r['mass_kg'] is not None:com+=g.env['volcenter'](parts[n])*(r['mass_kg']/mass)
    return mass,list(com)

def save(name,parts,info):
    doc=A.newDocument(name);objects=[];mesh=[]
    for n,s in parts.items():
        o=doc.addObject('PartDesign::Feature',n);o.Shape=s;o.addProperty('App::PropertyString','ValidationScope');o.ValidationScope='v21 research, not manufacturing or hardware release';objects.append(o)
        b=s.BoundBox;vis=Part.makeBox(b.XLength,b.YLength,b.ZLength,V(b.XMin,b.YMin,b.ZMin)) if info[n]['role'] in ('motor','electronics') else s
        pts,tris=vis.tessellate(1);mesh.append({'name':n,'role':info[n]['role'],'points':[list(p) for p in pts],'triangles':tris})
    doc.recompute();path=g.out/(name+'.FCStd');doc.saveAs(str(path));Part.export(objects,str(g.out/(name+'.step')));A.closeDocument(doc.Name)
    doc=A.openDocument(str(path));assert all(o.Shape.isValid() for o in doc.Objects if hasattr(o,'Shape'));A.closeDocument(doc.Name)
    (g.out/(name+'_mesh.json')).write_text(json.dumps(mesh)+'\n',encoding='utf-8')

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--refine',action='store_true');parser.add_argument('--midpitch',action='store_true');parser.add_argument('--power',action='store_true');args=parser.parse_args()
    if args.power:args.midpitch=True
    if args.midpitch:args.refine=True
    filename='arm_midpitch_screen.json' if args.midpitch else 'arm_refined_screen.json' if args.refine else 'arm_screen.json'
    if args.power:filename='arm_power_screen.json'
    screen=json.loads((g.out/('leg_power_screen.json' if args.power else 'leg_midpitch_screen.json' if args.midpitch else 'leg_replan_screen.json')).read_text());assert screen['completed']
    pool=[r['input'] for r in screen['cases'] if r['leg_and_fixed_body_geometry_pass']]
    pool.sort(key=lambda r:(-r['h_mm'],abs(r['beta_deg']-45),r['left_pose']['foot_translation_mm'][1]))
    cache={};out=[];selected=[]
    for row in pool:
        fixed,meta=g.fixed(row);lp,lm,_,_=g.leg(row['left_pose'])
        for n,s in lp.items():
            s=s.copy();s.translate(V(0,60,0));fixed['L_'+n]=s;meta['L_'+n]={**lm[n],'group':'L_leg'}
        key=(row['h_mm'],row['beta_deg'])
        if key not in cache:cache[key]=candidates(row,'midpitch' if args.midpitch else args.refine)
        arms=cache[key];side_results={};viable={}
        # Search a different shoulder-yaw branch first; stop at a valid side placement.
        order=sorted(range(len(arms)),key=lambda i:(arms[i]['q_deg'][2]<0,abs(arms[i]['q_deg'][2]-(0 if args.midpitch else 90)))) if args.refine else list(range(len(arms)))
        for side in (1,-1):
            if side==-1 and not viable[1]:
                side_results[str(side)]=[];viable[side]=[];break
            trials=[];good=[]
            for i in order:
                arm=arms[i]
                ap,am,cu,pa=g.arms(row,arm['q_deg'],side);parts={**fixed,**ap};bb=g.boxes(ap)
                ground={n:s.BoundBox.ZMin for n,s in ap.items() if bb[n].ZMin<-.5 and s.BoundBox.ZMin<-.5}
                col=collide(parts,set(ap),first_only=args.refine);trials.append({'arm_index':i,'arm':arm,'collisions':col,'ground_penetrations_mm':ground,'collision_scope':'First rejecting pair, or complete changed-pair check if none' if args.refine else 'All changed pairs'})
                print('arm',row['id'],side,'branch',i,'first collisions',len(col),'ground',len(ground),flush=True)
                if not col and not ground:
                    good.append((i,ap,am,pa))
                    if args.refine:break
            side_results[str(side)]=trials;viable[side]=good
        pair=None
        for left,right in itertools.product(viable[1],viable[-1]):
            li,lp,lm,lpa=left;ri,rp,rm,rpa=right;parts={**fixed,**lp,**rp};info={**meta,**lm,**rm}
            # Complete every modeled pair, including the opposite arm.
            col=collide(parts,set(parts))
            if col:continue
            mass,com=mass_com(parts,info);fx,fy,_=row['left_pose']['foot_translation_mm'];fy+=60
            anchor=json.loads((ROOT/'04_kinematics/recovery_v19/phase_results.json').read_text())['phase_samples'][0]['knee_contact_x_mm']
            supports=[[fx+dx,fy+dy,0] for dx in (-50,50) for dy in (-27.5,27.5)]+[[anchor,-132,0],lpa,rpa]
            balance=reactions(supports,com,mass)
            if not balance['nonnegative_vertical_balance_exists']:continue
            name='candidate_'+str(row['id']);save(name,parts,info)
            pair={'name':name,'input':row,'left_arm':arms[li],'right_arm':arms[ri],'mass_kg':mass,'COM_mm':com,
                  'support_points_mm':supports,'static_balance':balance,'all_modeled_collisions':col,
                  'motor_count':sum(m['role']=='motor' for m in info.values()),'full_recovery_validated':False}
            selected.append(pair);break
        out.append({'id':row['id'],'arm_branch_count':len(arms),'sides':side_results,'whole_body_candidate':pair})
        (g.out/filename).write_text(json.dumps({'partial_screen_survivors':len(pool),'processed':len(out),'cases':out,'selected':selected,'completed':len(out)==len(pool)},indent=2)+'\n',encoding='utf-8')
        print('wholebody',row['id'],'branches',len(arms),'left/right clear',len(viable[1]),len(viable[-1]),'selected',bool(pair),flush=True)
        if len(selected)>=3:break
    result={'partial_screen_survivors':len(pool),'processed':len(out),'cases':out,'selected':selected,'completed':len(out)==len(pool),
            'stop_rule':'Each side stops at first geometry-clear branch; all-pairs audit and static equilibrium then checked for combined pose. Not exhaustive arm search. Stop after three full-body static candidates.','full_recovery_validated':False}
    (g.out/filename).write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print('Selected',[r['name'] for r in selected],flush=True)
if __name__=='__main__':main()

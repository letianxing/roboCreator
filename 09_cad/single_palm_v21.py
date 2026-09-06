"""Alternative contact mode: planted left foot, right knee and right palm; left arm free."""
import json,argparse
import screen_arms_v21 as sa
from arm_candidates_v21 import candidates
from geometry_v21 import ROOT,V
parser=argparse.ArgumentParser();parser.add_argument('--extended',action='store_true');args=parser.parse_args()
g=sa.g;filename='single_palm_extended.json' if args.extended else 'single_palm_screen.json'
row=next(r for r in json.loads((ROOT/'04_kinematics/recovery_v21/power_refit.json').read_text())['cases'] if r['id']==544)
fixed,meta=g.fixed(row);lp,lm,_,_=g.leg(row['left_pose'])
for n,s in lp.items():
    s=s.copy();s.translate(V(0,60,0));fixed['L_'+n]=s;meta['L_'+n]={**lm[n],'group':'L_leg'}
free=[[p,0,y,e,0,0] for p,y,e in [(-80,90,90),(-100,90,90),(-60,90,60),(-80,-90,90),(-100,-90,90),(-100,0,120)]]
if args.extended:free=[[-120,0,0,45,0,0],[-120,0,90,0,0,0],[-140,0,0,45,0,0]]
trials=[];lefts=[]
def progress():
    (g.out/filename).write_text(json.dumps({'completed':False,'selected':[],'input':row,'trials':trials,'scope':'In-progress bounded contact-mode search, no pass implied'},indent=2)+'\n',encoding='utf-8')
for q in free:
    ap,am,_,_=g.arms(row,q,1);parts={**fixed,**ap};bb=g.boxes(ap);ground=[n for n,s in ap.items() if bb[n].ZMin<-.5 and s.BoundBox.ZMin<-.5];col=sa.collide(parts,set(ap),first_only=True)
    trials.append({'side':'left_free','q_deg':q,'collisions':col,'ground':ground});progress();print('free arm',q,'collision',len(col),'ground',len(ground),flush=True)
    if not col and not ground:lefts.append((q,ap,am));break
selected=[]
if lefts:
    arms=candidates(row,'midpitch');order=sorted(arms,key=lambda a:(a['q_deg'][2]>0,abs(a['q_deg'][2]+90),abs(a['target_mm'][1]-111)))[:12]
    for arm in order:
        rp,rm,_,rpa=g.arms(row,arm['q_deg'],-1);qL,lp,lm=lefts[0];parts={**fixed,**lp,**rp};info={**meta,**lm,**rm};bb=g.boxes(rp);ground=[n for n,s in rp.items() if bb[n].ZMin<-.5 and s.BoundBox.ZMin<-.5]
        col=sa.collide(parts,set(rp),first_only=True);trial={'side':'right_palm','arm':arm,'collisions':col,'ground':ground};trials.append(trial)
        progress()
        print('right palm',arm['target_mm'],'collision',len(col),'ground',len(ground),flush=True)
        if col or ground:continue
        allcol=sa.collide(parts,set(parts));mass,com=sa.mass_com(parts,info);fx,fy,_=row['left_pose']['foot_translation_mm'];fy+=60
        anchor=json.loads((ROOT/'04_kinematics/recovery_v19/phase_results.json').read_text())['phase_samples'][0]['knee_contact_x_mm'];support=[[fx+dx,fy+dy,0] for dx in (-50,50) for dy in (-27.5,27.5)]+[[anchor,-132,0],rpa];balance=sa.reactions(support,com,mass)
        trial.update(all_modeled_collisions=allcol,static_balance=balance)
        if allcol or not balance['nonnegative_vertical_balance_exists']:continue
        name='single_palm_544';sa.save(name,parts,info)
        selected.append({'name':name,'input':row,'left_arm':{'q_deg':qL,'contact':'free'},'right_arm':arm,'contact_mode':'one_right_palm','mass_kg':mass,'COM_mm':com,
                        'support_points_mm':support,'static_balance':balance,'all_modeled_collisions':allcol,'motor_count':sum(r['role']=='motor' for r in info.values()),'full_recovery_validated':False});break
out={'completed':True,'selected':selected,'input':row,'trials':trials,'free_arm_candidates':free,'scope':'Bounded alternative contact-mode search; listed free-arm poses and max12 right-palm IK branches, not an exhaustive search. No whole recovery validated.'}
(g.out/filename).write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8');print('single palm selected',bool(selected),flush=True)

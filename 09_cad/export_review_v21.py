"""Export best failed witnesses when no static candidate passed; never hide collisions."""
import json
import screen_arms_v21 as sa
from geometry_v21 import ROOT,V
g=sa.g
path=g.out/'arm_screen.json'
for candidate in ('arm_refined_screen.json','arm_midpitch_screen.json'):
    if (g.out/candidate).exists():path=g.out/candidate
if (g.out/'arm_power_screen.json').exists():path=g.out/'arm_power_screen.json'
single=g.out/'single_palm_screen.json'
if single.exists() and json.loads(single.read_text())['selected']:path=single
extended=g.out/'single_palm_extended.json'
if extended.exists() and json.loads(extended.read_text())['selected']:path=extended
data=json.loads(path.read_text());names=[]
inputs={r['id']:r for f in ['knee_replan.json','knee_midpitch.json','power_refit.json'] for r in json.loads((ROOT/'04_kinematics/recovery_v21'/f).read_text())['cases']}
if data['selected']:names=[r['name'] for r in data['selected']]
else:
    for item in data['cases']:
        row=inputs[item['id']];parts,info=g.fixed(row);lp,lm,_,_=g.leg(row['left_pose'])
        for n,s in lp.items():
            s=s.copy();s.translate(V(0,60,0));parts['L_'+n]=s;info['L_'+n]={**lm[n],'group':'L_leg'}
        chosen=[]
        for side in (1,-1):
            trials=item['sides'][str(side)]
            if not trials:
                if not chosen:break
                t={**chosen[0],'scope':'Mirrored left-arm witness for rejected CAD only; side search skipped because left arm failed'}
            else:t=min(trials,key=lambda t:(len(t['ground_penetrations_mm']),len(t['collisions']),sum(c['volume_mm3'] for c in t['collisions'])))
            ap,am,_,_=g.arms(row,t['arm']['q_deg'],side);parts.update(ap);info.update(am);chosen.append(t)
        if len(chosen)!=2:continue
        name='rejected_'+str(row['id']);sa.save(name,parts,info);names.append(name)
        (g.out/(name+'_evidence.json')).write_text(json.dumps({'input':row,'chosen_arm_trials':chosen,'all_collisions':sa.collide(parts,set(parts)),
            'manufacturable':False,'whole_recovery_validated':False},indent=2)+'\n',encoding='utf-8')
(g.out/'review_exports.json').write_text(json.dumps({'names':names,'has_static_pass':bool(data['selected'])},indent=2)+'\n',encoding='utf-8')
print('Review exports',names,flush=True)

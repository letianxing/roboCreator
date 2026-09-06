"""Per-component exact clearance screening, then joint electronics packaging combinations."""
import json,itertools,hashlib,time,argparse
from geometry_v22 import *
parser=argparse.ArgumentParser();parser.add_argument('--clearance',type=float);args=parser.parse_args()
nominal=cfg['nominal_electronics_clearance_mm'] if args.clearance is None else args.clearance
assert nominal>0
filename='layout_search.json' if args.clearance is None else 'layout_clearance_'+format(nominal,'g')+'.json'
changed=set(cfg['body_translation_candidates_mm']);samples=json.loads((ROOT/cfg['source_motion']).read_text())['samples'];scenes=[]
for h in cfg['coarse_heights_mm']:
    sample=next(s for s in samples if s['h_mm']==h);p,m,T=motion(sample)
    scenes.append({'name':'h'+str(h),'parts':{n:g.env['placed'](s,T.inverse()) for n,s in p.items()},'meta':m,'source':sample})
    print('built obstacle pose',h,flush=True)
p,m,T,source=stand();scenes.append({'name':'stand','parts':{n:g.env['placed'](s,T.inverse()) for n,s in p.items()},'meta':m,'source':source})
print('built standing regression pose',flush=True)
screen={};valid={}
for component,shifts in cfg['body_translation_candidates_mm'].items():
    records=[];valid[component]=[]
    for index,delta in enumerate(shifts):
        failures=[]
        for scene in scenes:
            test=scene['parts'][component].copy();test.translate(V(*delta));testbb=g.boxes({'test':test})['test']
            fixed={n:s for n,s in scene['parts'].items() if n not in changed};bb=g.boxes(fixed)
            for n,s in fixed.items():
                if box_gap(testbb,bb[n])>nominal+1e-6:continue
                check=pair_check(test,s,nominal)
                if not check['pass']:failures.append({'scene':scene['name'],'other':n,**check});break
            if failures:break
        record={'index':index,'translation_body_mm':delta,'pass':not failures,'first_failure':failures[:1]};records.append(record)
        if not failures:valid[component].append(delta)
        print(component,index,'pass',not failures,failures[:1],flush=True)
    screen[component]=records
    (OUT/filename).write_text(json.dumps({'component_screen':screen,'complete':False,'nominal_clearance_mm':nominal},indent=2)+'\n',encoding='utf-8')
combos=[]
for shifts in itertools.product(*(valid[n] for n in sorted(changed))):
    layout=dict(zip(sorted(changed),shifts));base=scenes[0]['parts'];parts={n:base[n].copy() for n in changed}
    for n in parts:parts[n].translate(V(*layout[n]))
    failures,near=changed_audit(parts,changed,nominal)
    if not failures:combos.append({'layout':layout,'translation_sum_mm':sum(V(*d).Length for d in shifts),'electronics_pair_checks':near})
combos.sort(key=lambda r:r['translation_sum_mm'])
out={'input_sha256':hashlib.sha256(PF.read_bytes()).hexdigest(),'source_motion_sha256':hashlib.sha256((ROOT/cfg['source_motion']).read_bytes()).hexdigest(),
     'component_screen':screen,'component_survivors':valid,'joint_layouts':combos,'complete':True,'nominal_clearance_mm':nominal,
     'scope':'Four discrete scenes, stated nominal electronics clearance. Component poses relative to v21. Continuous geometry, mounting, cables and dynamics unvalidated.'}
(OUT/filename).write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
print('combined valid layouts',len(combos),'best',combos[:1],flush=True)

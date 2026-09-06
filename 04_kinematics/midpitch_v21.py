"""Two bounded intermediate-tilt probes between30deg reach limit and45deg interference."""
import json,math
import numpy as np
import search_recovery_v21 as s
source=json.loads((s.OUT/'knee_replan.json').read_text())['cases'];rows=[]
for identifier in (344,345):
    r=next(x for x in source if x['id']==identifier);h=r['h_mm'];beta=35;x=r['hip_x_mm'];q=s.solve_leg(np.array(r['left_pose']['foot_translation_mm'])/1000,h,beta,x)
    theta=math.degrees(math.acos((h-32)/90));right=s.old.leg.leg_from_body(h,beta,theta-beta,120,35)['pose'];right['foot_translation_mm'][0]+=x
    rows.append({**r,'id':identifier+100,'beta_deg':beta,'right_pose':right,'left_pose':{**r['left_pose'],'q_deg':q['q_deg']},**q})
(s.OUT/'knee_midpitch.json').write_text(json.dumps({'cases':rows,'scope':'Two intermediate35deg body-tilt numerical probes; not full parameter-space proof'},indent=2)+'\n',encoding='utf-8')
print([(r['id'],r['ik_found']) for r in rows])

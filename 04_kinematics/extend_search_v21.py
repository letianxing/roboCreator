"""Separate forward-step extension; keep the original 192-case result untouched."""
import json,math,itertools,hashlib
import numpy as np
import search_recovery_v21 as s
first=json.loads((s.ROOT/'04_kinematics/recovery_v19/phase_results.json').read_text())['phase_samples'][0]
rows=[]
for h,beta in itertools.product(s.cfg['body_height_mm'],s.cfg['body_pitch_deg']):
    theta=math.degrees(math.acos((h-32)/90));x=first['knee_contact_x_mm']+90*math.sin(math.radians(theta))
    right=s.old.leg.leg_from_body(h,beta,theta-beta,90,35)['pose'];right['foot_translation_mm'][0]+=x
    for dx,dy in itertools.product([75,100,125],s.cfg['foot_y_relative_left_hip_mm']):
        target=np.array([x+dx,dy,2])/1000;r=s.solve_leg(target,h,beta,x)
        rows.append({'id':192+len(rows),'h_mm':h,'beta_deg':beta,'hip_x_mm':x,'right_pose':right,
            'left_pose':{'name':'forward_candidate','q_deg':r['q_deg'],'foot_pitch_deg':0,'foot_translation_mm':(target*1000).tolist()},**r})
    print('extended body',h,beta,'IK',sum(r['ik_found'] for r in rows[-12:]),'/12',flush=True)
out={'input_sha256':hashlib.sha256(s.PF.read_bytes()).hexdigest(),'extension_x_relative_hip_mm':[75,100,125],'cases':rows,'scope':'Numerical poses, no CAD/path/support validation.'}
(s.OUT/'forward_extension.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')

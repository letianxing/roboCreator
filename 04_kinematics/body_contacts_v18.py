"""Couple the v17 forearm contact candidate to leg position and contact constraints."""
import json,math,hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];PFILE=ROOT/'03_system/body_integration_v18.json';p=json.loads(PFILE.read_text(encoding='utf-8'))
def ry(deg):
    c,s=math.cos(math.radians(deg)),math.sin(math.radians(deg));return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def leg_from_body(h,beta,hp,k,ap):
    H=np.array([0.,0.,h]);Rt=ry(beta+hp);Rs=ry(beta+hp+k);Rf=ry(beta+hp+k+ap)
    K=H+Rt@np.array([0.,0.,-90]);R=K+Rs@np.array([0.,0.,-53]);P=R+Rs@np.array([0.,0.,-37]);F=P-Rf@np.array([0.,0.,35])
    return {'hip_mm':H.tolist(),'knee_mm':K.tolist(),'roll_mm':R.tolist(),'pitch_mm':P.tolist(),'foot_origin_mm':F.tolist(),'foot_pitch_deg':beta+hp+k+ap,
            'pose':{'name':'coupled_contact','foot_translation_mm':F.tolist(),'foot_pitch_deg':beta+hp+k+ap,'q_deg':[0,0,hp,k,0,ap]}}
def main():
    replay=json.loads((ROOT/'09_cad/gauntlet_v17/replay_results.json').read_text())
    arm=next(r['IK'] for r in replay['records'] if r['IK']['finite_search_pose_found'] and not r['has_ground_penetration_over_0_5mm'])
    h=arm['body_height_mm'];beta=arm['body_pitch_deg'];limits=p['coupling_limits_deg'];rows=[]
    # For flat feet phi_shin=-ankle, phi_thigh=phi_shin-knee. Foot-pad bottom is -2 mm.
    for ap in np.linspace(*limits['ankle_pitch'],141):
        for k in np.linspace(*limits['knee_pitch'],301):
            hp=-beta-ap-k
            if limits['hip_pitch'][0]<=hp<=limits['hip_pitch'][1]:
                required_h=37+90*math.cos(math.radians(-ap))+90*math.cos(math.radians(-ap-k))
                rows.append({'hip_pitch_deg':hp,'knee_deg':float(k),'ankle_deg':float(ap),'required_body_height_mm':required_h})
    best=min(rows,key=lambda r:r['required_body_height_mm']);closest=min(rows,key=lambda r:abs(r['required_body_height_mm']-h))
    kneeling=[];c=(h-p['kneeling_contact_height_mm'])/90
    if abs(c)<=1:
        for sign in (-1,1):
            theta=sign*math.degrees(math.acos(c));hp=theta-beta
            if limits['hip_pitch'][0]<=hp<=limits['hip_pitch'][1]:
                for k in p['kneeling_knee_samples_deg']:
                    for ap in p['kneeling_ankle_samples_deg']:
                        r=leg_from_body(h,beta,hp,k,ap);r['branch']=sign;kneeling.append(r)
    # Flat forearm fixes wrist height at pad offset 25 mm; elbow and wrist are level.
    upper_arm_max_h=25+80-80*math.cos(math.radians(beta))
    summary={'input_sha256':hashlib.sha256(PFILE.read_bytes()).hexdigest(),'arm_candidate':arm,
      'flat_feet':{'valid_limit_samples':len(rows),'minimum_body_height_mm':best['required_body_height_mm'],'minimum_sample':best,
                   'closest_sample':closest,'target_body_height_mm':h,'height_gap_mm':best['required_body_height_mm']-h,'compatible_in_grid':abs(closest['required_body_height_mm']-h)<=.5},
      'flat_pad_upper_arm_height_bound_mm':upper_arm_max_h,'kneeling_candidates':kneeling,
      'limitations':['Limits are explicit design assumptions; do not claim whole-robot impossibility','Both-flat-feet failure does not reject knee/toe/other recovery contact phases',
                     'Knee contact candidates require actual CAD ground and self-collision checks; bare motor housing is not a validated knee pad']}
    out=ROOT/'04_kinematics/body_contacts_v18.json';out.write_text(json.dumps(summary,indent=2)+'\n')
    print('flat feet minimum body height',best['required_body_height_mm'],'arm target',h,'kneeling candidates',len(kneeling))
if __name__=='__main__':main()

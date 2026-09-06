"""Compute a small toe-touch transition and a common-body asymmetric foot endpoint."""
import math,json,hashlib,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'04_kinematics'));import recovery_phases_v19 as old
PF=ROOT/'03_system/contact_route_v20.json';p=json.loads(PF.read_text());OUT=ROOT/'04_kinematics/recovery_v20';OUT.mkdir(exist_ok=True)
def main():
    data=json.loads((ROOT/'04_kinematics/recovery_v19/phase_results.json').read_text());start=data['phase_samples'][0]
    q0=start['arm']['q_deg']+[0,0];e=p['toe_engagement'];delta=math.degrees(math.asin(e['native_wrist_z_mm']/math.hypot(e['native_palm_front_x_mm'],e['native_wrist_z_mm']-e['native_palm_bottom_z_mm']))-math.atan2(e['native_wrist_z_mm']-e['native_palm_bottom_z_mm'],e['native_palm_front_x_mm']))
    samples=[]
    for u in np.linspace(0,1,e['samples']):
        q=q0.copy();q[-1]=-float(u)*delta
        # Conjugating into the native hand frame gives +delta about native Y.
        angle=math.radians(float(u)*delta)
        z=e['native_wrist_z_mm']-e['native_palm_front_x_mm']*math.sin(angle)-(e['native_wrist_z_mm']-e['native_palm_bottom_z_mm'])*math.cos(angle)
        samples.append({'fraction':float(u),'q_deg':q,'front_edge_predicted_height_mm':z})
    a=p['asymmetric_foot'];h=a['body_height_mm'];beta=a['body_pitch_deg'];radius=32
    theta=math.degrees(math.acos((h-radius)/90));xhip=start['knee_contact_x_mm']+90*math.sin(math.radians(theta))
    right=old.leg.leg_from_body(h,beta,theta-beta,90,35)['pose'];right['foot_translation_mm'][0]+=xhip
    ap=a['swing_ankle_pitch_deg'];phi_shin=-ap
    cos_thigh=(h-37-90*math.cos(math.radians(phi_shin)))/90
    assert -1<=cos_thigh<=1
    phi_thigh=-math.degrees(math.acos(cos_thigh));k=phi_shin-phi_thigh;hp=phi_thigh-beta
    footx=xhip-90*math.sin(math.radians(phi_shin))-90*math.sin(math.radians(phi_thigh))
    left={'name':'left_foot_planted','foot_translation_mm':[footx,0,2],'foot_pitch_deg':0,'q_deg':[0,0,hp,k,0,ap]}
    seed=next(r for r in data['phase_samples'] if r['body_height_mm']==90 and r['body_pitch_deg']==60 and r['arm']['contact_type']=='palm')
    target=a['palm_target_mm'].copy();target[1]-=6 # legacy solver shoulder75; actual assembly shoulder81
    arm=old.solve_arm(target,h,beta,xhip,True,np.deg2rad(seed['arm']['q_deg'][:4]))
    bounded=a['swing_knee_limit_deg'][0]<=k<=a['swing_knee_limit_deg'][1] and a['hip_limit_deg'][0]<=hp<=a['hip_limit_deg'][1]
    out={'input_sha256':hashlib.sha256(PF.read_bytes()).hexdigest(),'initial':start,'toe_engagement_deg':delta,'toe_samples':samples,
      'asymmetric_endpoint':{'body_height_mm':h,'body_pitch_deg':beta,'hip_x_mm':xhip,'left_leg_pose':left,'right_leg_pose':right,'left_leg_within_assumed_limits':bounded,'arms':arm},
      'whole_recovery_validated':False,'limits':['Knee radius and joint limits are design assumptions','Contact reaction and CAD collision must be checked independently','Hand/foot replanting paths between endpoints remain to be planned']}
    (OUT/'contact_candidates.json').write_text(json.dumps(out,indent=2)+'\n');print('toe rotation',delta,'asymmetric hp/k/ap',hp,k,ap,'arm IK',arm['ik_found'])
if __name__=='__main__':main()

"""Half-mm local pose samples; preserve exact v21 integer samples for regression."""
import json,math
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from search_recovery_v21 import solve_leg,leg_fk,old
from arm_candidates_v21 import fk
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'04_kinematics/recovery_v22';OUT.mkdir(exist_ok=True)
source=json.loads((ROOT/'09_cad/body_v21/local_lift.json').read_text());lookup={r['h_mm']:r for r in source['samples']}
base=json.loads((ROOT/'09_cad/body_v21/single_palm_extended.json').read_text())['selected'][0]

def arm(row,seed):
    lo=np.deg2rad([-160,-120,-160,0,-130,-160]);hi=np.deg2rad([160,120,160,150,130,160]);target=np.array(base['right_arm']['target_mm'])/1000
    def residual(q):
        p,n,z=fk(q,row);return np.r_[p-target,.08*(n-[0,0,-1]),3*min(0,z-.001)]
    fit=least_squares(residual,np.clip(np.deg2rad(seed),lo+1e-10,hi-1e-10),bounds=(lo,hi),max_nfev=160,ftol=1e-11,xtol=1e-11,gtol=1e-11)
    p,n,z=fk(fit.x,row);pe=float(np.linalg.norm(p-target)*1000);ne=math.degrees(math.acos(np.clip(n@np.array([0,0,-1]),-1,1)))
    return {'q_deg':np.rad2deg(fit.x).tolist(),'position_error_mm':pe,'normal_error_deg':ne,'ik_found':bool(pe<.05 and ne<.05 and z>=.00099)}

def main():
    rows=[];anchor=base['support_points_mm'][4][0];foot=base['input']['left_pose']['foot_translation_mm'];beta=35
    for h in np.arange(114,120.01,.5):
        h=float(h)
        if h in lookup:
            r=lookup[h];row={key:r[key] for key in ['h_mm','input','left_arm','right_arm','leg_ik']};row['from_v21_exact_sample']=True
        else:
            theta=math.degrees(math.acos((h-32)/90));x=anchor+90*math.sin(math.radians(theta));pose={**base['input'],'h_mm':h,'hip_x_mm':x}
            leg=solve_leg(np.array(foot)/1000,h,beta,x);pose['left_pose']={**base['input']['left_pose'],'q_deg':leg['q_deg']}
            right=old.leg.leg_from_body(h,beta,theta-beta,source['support_shin_world_pitch_deg']-theta,35)['pose'];right['foot_translation_mm'][0]+=x;pose['right_pose']=right
            right_arm=arm(pose,lookup[math.floor(h)]['right_arm']['q_deg'])
            row={'h_mm':h,'input':pose,'left_arm':base['left_arm'],'right_arm':right_arm,'leg_ik':leg,'from_v21_exact_sample':False}
        closure=[]
        for key in ['left_pose','right_pose']:
            q=row['input'][key];p,R=leg_fk(np.deg2rad(q['q_deg']),h,beta,row['input']['hip_x_mm']);target=np.array(q['foot_translation_mm'])/1000
            er=float(np.linalg.norm(p-target)*1000);Rtarget=old.arm.rot('y',math.radians(q['foot_pitch_deg']));rotation_error=float(np.linalg.norm(R-Rtarget))
            assert er<.01 and rotation_error<.001,(h,key,er,rotation_error)
            closure.append({'leg':key,'position_error_mm':er,'rotation_matrix_error':rotation_error})
        assert row['leg_ik']['ik_found'] and row['right_arm']['ik_found']
        row['closure_checks']=closure;rows.append(row)
    (OUT/'segment_samples.json').write_text(json.dumps({'samples':rows,'support_points_mm':base['support_points_mm'],'scope':'13 local0.5mm samples, not a full recovery or continuous swept-volume proof'},indent=2)+'\n',encoding='utf-8')
    print('Half-mm samples and bilateral closure passed',len(rows))
if __name__=='__main__':main()

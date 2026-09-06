"""Fixed-contact local body-lift probe around a verified static candidate, not full recovery."""
import json,sys,math
import numpy as np
from scipy.optimize import least_squares
from geometry_v21 import ROOT,V
import screen_arms_v21 as sa
sys.path.insert(0,str(ROOT/'04_kinematics'))
from search_recovery_v21 import solve_leg,old
from arm_candidates_v21 import fk
g=sa.g

def solve_arm(row,target,seed):
    lo=np.deg2rad([-160,-120,-160,0,-130,-160]);hi=np.deg2rad([160,120,160,150,130,160]);target=np.array(target)/1000
    def res(q):
        p,n,z=fk(q,row);return np.r_[p-target,.08*(n-[0,0,-1]),3*min(0,z-.001)]
    fit=least_squares(res,np.clip(np.deg2rad(seed),lo+1e-10,hi-1e-10),bounds=(lo,hi),max_nfev=160,ftol=1e-11,xtol=1e-11,gtol=1e-11)
    p,n,z=fk(fit.x,row);pe=float(np.linalg.norm(p-target)*1000);ne=math.degrees(math.acos(np.clip(n@np.array([0,0,-1]),-1,1)))
    return {'q_deg':np.rad2deg(fit.x).tolist(),'position_error_mm':pe,'normal_error_deg':ne,'cuff_min_z_mm':float(z*1000),'ik_found':bool(pe<.05 and ne<.05 and z>=.00099)}

def main():
    path=g.out/'arm_refined_screen.json'
    if (g.out/'arm_midpitch_screen.json').exists():path=g.out/'arm_midpitch_screen.json'
    if (g.out/'arm_power_screen.json').exists():path=g.out/'arm_power_screen.json'
    single=g.out/'single_palm_screen.json'
    if single.exists() and json.loads(single.read_text())['selected']:path=single
    extended=g.out/'single_palm_extended.json'
    if extended.exists() and json.loads(extended.read_text())['selected']:path=extended
    if not path.exists():path=g.out/'arm_screen.json'
    selected=json.loads(path.read_text())['selected']
    if not selected:
        (g.out/'local_lift.json').write_text(json.dumps({'status':'not_run','reason':'No whole-body static candidate passed','full_recovery_validated':False},indent=2)+'\n');return
    source=selected[0];base=source['input'];foot=base['left_pose']['foot_translation_mm'];anchor=source['support_points_mm'][4][0];qL=source['left_arm']['q_deg'];qR=source['right_arm']['q_deg'];rows=[]
    single_mode=source.get('contact_mode')=='one_right_palm'
    shin_angle=math.degrees(math.acos((base['h_mm']-32)/90))+base['right_pose']['q_deg'][3]
    for h in list(range(120,109,-1))+[121,122]:
        if h>120:qL=source['left_arm']['q_deg'];qR=source['right_arm']['q_deg']
        beta=base['beta_deg'];theta=math.degrees(math.acos((h-32)/90));x=anchor+90*math.sin(math.radians(theta));row={**base,'h_mm':h,'hip_x_mm':x}
        leg=solve_leg(np.array(foot)/1000,h,beta,x);row['left_pose']={**base['left_pose'],'q_deg':leg['q_deg']}
        right=old.leg.leg_from_body(h,beta,theta-beta,shin_angle-theta,35)['pose'];right['foot_translation_mm'][0]+=x;row['right_pose']=right
        aL={'q_deg':qL,'ik_found':True,'scope':'Free arm retains body-relative joint angles'} if single_mode else solve_arm(row,source['left_arm']['target_mm'],qL)
        aR=solve_arm(row,source['right_arm']['target_mm'],qR)
        out={'h_mm':h,'input':row,'leg_ik':leg,'left_arm':aL,'right_arm':aR,'sample_geometry_and_static_pass':False,'geometry_checked':False}
        if leg['ik_found'] and aL['ik_found'] and aR['ik_found']:
            qL=aL['q_deg'];qR=aR['q_deg'];parts,info=g.fixed(row);lp,lm,_,_=g.leg(row['left_pose'])
            for n,s in lp.items():
                s=s.copy();s.translate(V(0,60,0));parts['L_'+n]=s;info['L_'+n]={**lm[n],'group':'L_leg'}
            palms=[]
            for side,q in [(1,qL),(-1,qR)]:
                ap,am,_,pa=g.arms(row,q,side);parts.update(ap);info.update(am)
                if not single_mode or side==-1:palms.append(pa)
            col=sa.collide(parts,set(parts));bb=g.boxes(parts);ground={n:s.BoundBox.ZMin for n,s in parts.items() if bb[n].ZMin<-.5 and s.BoundBox.ZMin<-.5}
            mass,com=sa.mass_com(parts,info);support=source['support_points_mm'][:5]+palms;balance=sa.reactions(support,com,mass)
            passed=not col and not ground and balance['nonnegative_vertical_balance_exists']
            out.update(geometry_checked=True,collisions=col,ground_penetrations_mm=ground,mass_kg=mass,COM_mm=com,support_points_mm=support,static_balance=balance,sample_geometry_and_static_pass=passed)
            if passed and h in (110,115):sa.save('lift_h'+str(h),parts,info)
        rows.append(out)
        (g.out/'local_lift.json').write_text(json.dumps({'source_candidate':source['name'],'samples':rows,'complete_probe':False,'full_recovery_validated':False},indent=2)+'\n',encoding='utf-8')
        print('lift h',h,'IK',leg['ik_found'],aL['ik_found'],aR['ik_found'],'sample',out['sample_geometry_and_static_pass'],flush=True)
    rows.sort(key=lambda r:r['h_mm']);deltas=[]
    for a,b in zip(rows,rows[1:]):
        if not (a['sample_geometry_and_static_pass'] and b['sample_geometry_and_static_pass']):continue
        qa=a['leg_ik']['q_deg']+a['input']['right_pose']['q_deg']+a['left_arm']['q_deg']+a['right_arm']['q_deg'];qb=b['leg_ik']['q_deg']+b['input']['right_pose']['q_deg']+b['left_arm']['q_deg']+b['right_arm']['q_deg']
        deltas.append({'height_interval_mm':[a['h_mm'],b['h_mm']],'maximum_joint_increment_deg':max(abs(x-y) for x,y in zip(qa,qb))})
    out={'source_candidate':source['name'],'samples':rows,'adjacent_sample_joint_changes':deltas,'complete_probe':True,'full_recovery_validated':False,
         'contact_mode':source.get('contact_mode','two_palms'),'support_shin_world_pitch_deg':shin_angle,
         'scope':'Same flat left sole, right knee and supporting palm anchors. Right shank/foot absolute pose retained by varying knee angle. 1mm height samples, not certified continuous swept-volume/dynamic stability. No timing, force controller or cuff-to-palm replanting path.'}
    (g.out/'local_lift.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()

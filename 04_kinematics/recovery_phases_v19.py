"""Contact-mode feasibility and explicitly provisional recovery phase samples."""
import json,math,importlib.util,hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];PF=ROOT/'03_system/shoulder_recovery_v19.json';cfg=json.loads(PF.read_text())
spec=importlib.util.spec_from_file_location('arm17',ROOT/'04_kinematics/gauntlet_contact_v17.py');arm=importlib.util.module_from_spec(spec);spec.loader.exec_module(arm)
spec2=importlib.util.spec_from_file_location('contacts18',ROOT/'04_kinematics/body_contacts_v18.py');leg=importlib.util.module_from_spec(spec2);spec2.loader.exec_module(leg)
outdir=ROOT/'04_kinematics/recovery_v19';outdir.mkdir(exist_ok=True)
def solve_arm(target,h,beta,x,palm=False,seed_q=None):
    n=6 if palm else 4;lo=np.deg2rad([-160,-120,-160,0,-130,-160][:n]);hi=np.deg2rad([160,120,160,150,130,160][:n])
    def fk(q):
        pos,normal,nodes=arm.fk(q[:4],h/1000,math.radians(beta),28,180)
        if palm:
            R=nodes['forearm_rotation']@arm.rot('x',q[4])@arm.rot('y',q[5]);pos=nodes['wrist']+R@np.array([.0245,0,-.028]);normal=R@np.array([1.,0,0])
        return pos+np.array([x/1000,0,0]),normal
    def res(q):
        pos,norm=fk(q);e=np.r_[pos-np.array(target)/1000,.08*(norm-np.array([0,0,-1]))]
        if palm:
            _,_,nodes=arm.fk(q[:4],h/1000,math.radians(beta),28,180)
            R=nodes['forearm_rotation'];center=nodes['wrist']+R@np.array([.025,0,.012])
            corners=[center+R@np.array([0,sy*.020,sx*.010]) for sy in (-1,1) for sx in (-1,1)]
            # Palm support requires the fixed cuff pad to lift clear of the floor.
            e=np.r_[e,[3*min(0,c[2]-.001) for c in corners]]
        return e
    rng=np.random.default_rng(19019);best=None;solutions=[]
    for i in range(20):
        q=np.clip(np.r_[seed_q,np.zeros(n-len(seed_q))] if i==0 and seed_q is not None else rng.uniform(lo,hi),lo,hi)
        for _ in range(130):
            e=res(q)
            if np.linalg.norm(e)<1e-8:break
            J=np.column_stack([(res(q+np.eye(n)[j]*1e-6)-e)/1e-6 for j in range(n)])
            step=-np.linalg.solve(J.T@J+1e-6*np.eye(n),J.T@e);step=np.clip(step,-.25,.25)
            accepted=False
            for scale in (1,.5,.1):
                q2=np.clip(q+scale*step,lo,hi)
                if np.linalg.norm(res(q2))<np.linalg.norm(e):q=q2;accepted=True;break
            if not accepted:break
        error=np.linalg.norm(res(q))
        if best is None or error<best[0]:best=(error,q)
        if error<1e-5:solutions.append(q.copy())
    if solutions:
        reference=np.r_[seed_q,np.zeros(n-len(seed_q))] if seed_q is not None else np.zeros(n)
        # Keep the nearby joint branch, reducing gratuitous wrist and shoulder wrapping.
        chosen=min(solutions,key=lambda v:float(np.sum((v-reference)**2)))
        best=(np.linalg.norm(res(chosen)),chosen)
    q=best[1];point,normal=fk(q)
    pe=float(np.linalg.norm(point-np.array(target)/1000)*1000);ne=math.degrees(math.acos(np.clip(normal@np.array([0,0,-1]),-1,1)))
    return {'q_deg':np.rad2deg(q).tolist(),'point_mm':(point*1000).tolist(),'position_error_mm':pe,'normal_error_deg':ne,'ik_found':pe<=2 and ne<=5,'contact_type':'palm' if palm else 'cuff',
            'alternative_solutions_q_deg':[np.rad2deg(v).tolist() for v in solutions]}

def main():
    source=json.loads((ROOT/'04_kinematics/body_contacts_v18.json').read_text())['arm_candidate'];h0=source['body_height_mm'];radius=cfg['knee_pad']['outer_radius_mm'];beta0=60
    theta0=math.degrees(math.acos((h0-radius)/90));knee_x=-90*math.sin(math.radians(theta0));target=[150,125,0]
    records=[]
    for h,beta in [(h0,60),(75,60),(75,75),(90,60),(90,75),(90,85),(105,95),(118,100),(122,100),(122,90)]:
        theta=math.degrees(math.acos(np.clip((h-radius)/90,-1,1)));hip_x=knee_x+90*math.sin(math.radians(theta));hp=theta-beta
        pose=leg.leg_from_body(h,beta,hp,90,35)['pose'];pose['foot_translation_mm'][0]+=hip_x
        for mode in ('cuff','palm'):
            a=solve_arm(target,h,beta,hip_x,mode=='palm',np.deg2rad(source['upstream_q_deg']))
            records.append({'body_height_mm':h,'body_pitch_deg':beta,'hip_x_mm':hip_x,'leg_pose':pose,'arm':a,
             'knee_contact_x_mm':knee_x,'cuff_height_upper_bound_mm':25+80-80*math.cos(math.radians(beta)),'wholebody_validated':False})
    # Toe-anchored symmetric transfer probe. It is retained even when fixed hands cannot reach.
    phi0=math.degrees(math.atan2(55,37))+math.degrees(math.acos(32/math.sqrt(55**2+37**2)))
    theta=0;phi_shin=90;ap0=phi0-phi_shin
    start=leg.leg_from_body(122,90,-90,90,ap0);F=np.array(start['foot_origin_mm'])+np.array([knee_x,0,0]);Rf=leg.ry(phi0)
    toe=F+Rf@np.array([55.,0.,-2.]);toe[2]=0
    path=[]
    for t in np.linspace(0,1,25):
        phi=phi0*(1-t);ap=ap0*(1-t)-10*t;knee=90*(1-t)+20*t;beta=100*(1-t)
        Rf=leg.ry(phi);foot=toe-Rf@np.array([55.,0.,-2.]);P=foot+Rf@np.array([0.,0.,35]);Rs=leg.ry(phi-ap);K=P+Rs@np.array([0.,0.,90]);Rt=leg.ry(phi-ap-knee);H=K+Rt@np.array([0.,0.,90]);hp=phi-ap-knee-beta
        S=H+leg.ry(beta)@np.array([0.,75.,80.]);distance=float(np.linalg.norm(np.array(target)-S));arm_max=80+85+math.hypot(24.5,28)
        path.append({'s':float(t),'body_height_mm':float(H[2]),'hip_x_mm':float(H[0]),'body_pitch_deg':beta,
            'leg_pose':{'name':'toe_transfer','foot_translation_mm':foot.tolist(),'foot_pitch_deg':phi,'q_deg':[0,0,hp,knee,0,ap]},
            'shoulder_to_fixed_hand_mm':distance,'absolute_arm_reach_bound_mm':arm_max,'fixed_hand_reachable_necessary_condition':distance<=arm_max,
            'contact_mode':'palms_and_toes_until_release','wholebody_validated':False})
    first_failure=next((r for r in path if not r['fixed_hand_reachable_necessary_condition']),None)
    out={'input_sha256':hashlib.sha256(PF.read_bytes()).hexdigest(),'phase_samples':records,'toe_transfer_probe':path,
      'toe_start_pitch_deg':phi0,'toe_anchor_mm':toe.tolist(),'first_fixed_hand_reach_failure':first_failure,
      'route_status':'not_executable','next_required_mode':'Replant hands or use asymmetric single-foot placement; fixed-cuff-only rise and this fixed-hand symmetric route are not validated',
      'limits':['IK search failure is not a global proof','Reach-bound failure is conclusive for the specified fixed hand and arm lengths','No dynamics/contact/friction or collision feasibility implied by interpolation']}
    (outdir/'phase_results.json').write_text(json.dumps(out,indent=2)+'\n')
    print('phase IK',[(round(r['body_height_mm'],2),r['body_pitch_deg'],r['arm']['contact_type'],r['arm']['ik_found']) for r in records],flush=True)
    print('toe route first fixed-hand failure',first_failure and first_failure['s'],flush=True)
if __name__=='__main__':main()

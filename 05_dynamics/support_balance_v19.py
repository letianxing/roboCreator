"""Quasi-static vertical contact balance and recovery transition diagnostics."""
import json,math,itertools,hashlib,importlib.util
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];PF=ROOT/'03_system/shoulder_recovery_v19.json';cfg=json.loads(PF.read_text());OUT=ROOT/'05_dynamics/support_v19';OUT.mkdir(exist_ok=True)
spec=importlib.util.spec_from_file_location('arm17',ROOT/'04_kinematics/gauntlet_contact_v17.py');arm=importlib.util.module_from_spec(spec);spec.loader.exec_module(arm)
def reactions(points,com,mass):
    points=np.asarray(points)/1000;com=np.asarray(com)/1000;weight=mass*9.81;A=np.vstack([np.ones(len(points)),points[:,0],points[:,1]]);b=weight*np.r_[1,com[:2]];best=None
    for n in range(3,len(points)+1):
        for subset in itertools.combinations(range(len(points)),n):
            matrix=A[:,subset];f=np.linalg.lstsq(matrix,b,rcond=None)[0]
            if min(f)<-1e-8 or np.linalg.norm(matrix@f-b)>1e-8:continue
            full=np.zeros(len(points));full[list(subset)]=f
            if best is None or np.linalg.norm(full)<np.linalg.norm(best):best=full
    if best is None:return {'nonnegative_vertical_balance_exists':False,'forces_N':None}
    return {'nonnegative_vertical_balance_exists':True,'forces_N':best.tolist(),'force_residual_N':float(sum(best)-weight),
            'moment_residual_Nm':(A[1:]@best-b[1:]).tolist()}
def main():
    assembly=json.loads((ROOT/'09_cad/body_v19/assembly_results.json').read_text());phases=json.loads((ROOT/'04_kinematics/recovery_v19/phase_results.json').read_text());results=[]
    for sc in assembly['scenes']:
        if sc['phase']['support']=='right_cuff_and_knees':continue
        if sc['phase']['support']=='feet':points=[[x,y,0] for x in (-50,50) for y in (-87.5,-32.5,32.5,87.5)]
        else:points=sc['knee_contact_points_mm']+(sc['cuff_contact_points_mm'] if sc['phase']['support']=='cuffs_and_knees' else sc['palm_contact_points_mm'])
        balance=reactions(points,sc['approx_COM_mm'],sc['known_or_allocated_mass_kg']);balance.update(scene=sc['name'],points_mm=points,assumed_mass_kg=sc['known_or_allocated_mass_kg'],COM_mm=sc['approx_COM_mm'],geometry_has_collisions=bool(sc['collisions']),dynamic_stability_verified=False)
        if sc['phase']['support']=='palms_and_knees' and balance['forces_N']:
            source=next(r for r in phases['phase_samples'] if r['body_height_mm']==90 and r['body_pitch_deg']==60 and r['arm']['contact_type']=='palm')
            q=np.deg2rad(source['arm']['q_deg']);_,_,nodes=arm.fk(q[:4],.09,math.radians(60),28,180);W=nodes['wrist']+np.array([source['hip_x_mm']/1000,.006,0]);F=balance['forces_N'][2]
            lever=np.array(sc['palm_contact_points_mm'][0])/1000-W
            balance['left_wrist_external_moment_magnitude_Nm']=float(F*np.linalg.norm(lever[:2]))
            balance['wrist_limit_note']='External ground-force moment only; distal weight/inertia/friction and actuator thermal limits still needed.'
        results.append(balance)
    # Inspect a direct symmetric cuff-to-palm interpolation before claiming the modes connect.
    start=next(r for r in phases['phase_samples'] if r['arm']['contact_type']=='cuff' and r['arm']['ik_found']);end=next(r for r in phases['phase_samples'] if abs(r['body_height_mm']-start['body_height_mm'])<1e-6 and r['arm']['contact_type']=='palm')
    a=np.deg2rad(start['arm']['q_deg']+[0,0]);b=np.deg2rad(end['arm']['q_deg']);rows=[]
    for s in np.linspace(0,1,41):
        q=(1-s)*a+s*b;_,_,nodes=arm.fk(q[:4],start['body_height_mm']/1000,math.radians(start['body_pitch_deg']),28,180);R=nodes['forearm_rotation'];W=nodes['wrist'];Rp=R@arm.rot('x',q[4])@arm.rot('y',q[5])
        cuff=[W+R@np.array([.025,sy*.020,.012+sx*.010]) for sy in (-1,1) for sx in (-1,1)]
        palm=[W+Rp@np.array([.0245,sy*.018,-.028+sx*.026]) for sy in (-1,1) for sx in (-1,1)]
        cz=min(p[2] for p in cuff)*1000;pz=min(p[2] for p in palm)*1000
        rows.append({'s':float(s),'cuff_min_z_mm':float(cz),'palm_min_z_mm':float(pz),'penetrates_over_0_5mm':bool(min(cz,pz)<-.5),'both_hand_supports_clear_ground_over_0_5mm':bool(min(cz,pz)>.5)})
    kneel=next(r for r in results if r['scene']=='cuff_knee_candidate');single=[]
    for held in (2,3):
        support=[kneel['points_mm'][i] for i in (0,1,held)]
        samples=[]
        for dx,dy in itertools.product((-20,0,20),(-10,0,10)):
            com=np.array(kneel['COM_mm'])+np.array([dx,dy,0]);r=reactions(support,com,kneel['assumed_mass_kg']);samples.append({'COM_offset_mm':[dx,dy],**r})
        single.append({'held_hand_index':held,'support_points_mm':support,'assumed_COM_perturbation_results':samples,
                       'all_perturbations_have_vertical_balance':all(r['nonnegative_vertical_balance_exists'] for r in samples)})
    midpoint=next((r for r in assembly['scenes'] if r['phase']['support']=='right_cuff_and_knees'),None)
    midpoint_balance=None
    if midpoint:
        pts=midpoint['knee_contact_points_mm']+[midpoint['cuff_contact_points_mm'][1]]
        midpoint_balance={'scene':midpoint['name'],**reactions(pts,midpoint['approx_COM_mm'],midpoint['known_or_allocated_mass_kg'])}
    out={'input_sha256':hashlib.sha256(PF.read_bytes()).hexdigest(),'balances':results,'direct_symmetric_handoff':rows,'alternating_one_hand_support':single,'actual_midpoint_static_balance':midpoint_balance,
          'handoff_geometry_violations':sum(r['penetrates_over_0_5mm'] or r['both_hand_supports_clear_ground_over_0_5mm'] for r in rows),
          'status':'conditional_static_balance_and_rejected_path_diagnostics','whole_recovery_verified':False,
          'limitations':['No friction or elastic pad model','Mass/COM are estimates with missing hardware','Zero contact residual does not establish dynamic or real-machine stability',
                         'No shoulder/knee strength or allowable pressure established','Direct joint interpolation is a tested proposal, not a control trajectory']}
    (OUT/'balance_results.json').write_text(json.dumps(out,indent=2)+'\n')
    print([(r['scene'],r['nonnegative_vertical_balance_exists'],r['forces_N']) for r in results]);print('handoff violations',out['handoff_geometry_violations'])
if __name__=='__main__':main()

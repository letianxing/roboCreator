"""Incremental exact toe-touch checks and a wholebody asymmetric support endpoint."""
import FreeCAD as A, Part, json,math,hashlib,sys
from pathlib import Path
import numpy as np
from route_geometry_v20 import load
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/body_v20';OUT.mkdir(exist_ok=True);mod=load();mod.env['OUT']=OUT
PF=ROOT/'03_system/contact_route_v20.json';cfg=json.loads(PF.read_text());data=json.loads((ROOT/'04_kinematics/recovery_v20/contact_candidates.json').read_text());V=A.Vector
sys.path.insert(0,str(ROOT/'05_dynamics'));from support_balance_v19 import reactions
sys.path.insert(0,str(ROOT/'04_kinematics'));import recovery_phases_v19 as old
def summary(whole,info):
    mass=sum(r['mass_kg'] for r in info.values() if r['mass_kg'] is not None);com=V(0,0,0)
    for n,r in info.items():
        if r['mass_kg'] is not None:com+=mod.env['volcenter'](whole[n])*(r['mass_kg']/mass)
    return mass,list(com)
def exact_changed(whole,changed):
    collisions=[];keys=list(whole)
    for i,a in enumerate(keys):
        for b in keys[i+1:]:
            if a not in changed and b not in changed:continue
            s,t=whole[a],whole[b]
            if s.BoundBox.intersect(t.BoundBox):
                v=s.common(t).Volume
                if v>1e-5:collisions.append({'a':a,'b':b,'volume_mm3':v})
    return collisions
def main():
    start=data['initial'];q0=start['arm']['q_deg']+[0,0];scene={'name':'toe_start','pose':start['leg_pose'],'q':q0,'support':'cuffs_and_knees'}
    whole,info,joints,knees,cuffs,palms=mod.build_geometry(scene)
    base_collisions=mod.env['collide'](whole,info);assert not base_collisions
    _,_,nodes=old.arm.fk(np.deg2rad(q0[:4]),start['body_height_mm']/1000,math.radians(start['body_pitch_deg']),28,180)
    W=nodes['wrist']*1000+np.array([start['hip_x_mm'],6,0]);axis=nodes['forearm_rotation'][:,1]
    rows=[];exported=[]
    for i,sample in enumerate(data['toe_samples']):
        trial={n:s.copy() for n,s in whole.items()};angle=sample['q_deg'][-1]
        for n in ['L_HandAndPalm','L_GripperMotor']:trial[n].rotate(V(*W),V(*axis),angle)
        collisions=exact_changed(trial,{'L_HandAndPalm','L_GripperMotor'})
        R=nodes['forearm_rotation']@old.arm.rot('y',math.radians(angle));toe=W+R@np.array([24.5,0,-54.])
        minz=min(trial[n].BoundBox.ZMin for n in ['L_HandAndPalm','L_GripperMotor']);mass,com=summary(trial,info)
        supported=reactions(knees+cuffs,com,mass)
        rows.append({'sample':i,**sample,'CAD_minimum_moving_z_mm':minz,'toe_center_mm':toe.tolist(),'collisions':collisions,'retained_cuff_knee_support':supported,
                     'mass_kg':mass,'COM_mm':com,'sampled_geometry_pass':not collisions and minz>=-.5})
        if i in (0,len(data['toe_samples'])//2,len(data['toe_samples'])-1):
            name=f'toe_engage_{i:02d}';r=mod.env['export'](name,trial,info,joints,[]);r['step']=rows[-1];exported.append(r)
        print('toe step',i,'ground',round(minz,5),'collisions',len(collisions),flush=True)
    terminal=rows[-1];unload=reactions(knees+[cuffs[1],terminal['toe_center_mm']],terminal['COM_mm'],terminal['mass_kg'])
    # Build right kneeling leg and common body first; replace only the left leg.
    endpoint=data['asymmetric_endpoint'];qa=endpoint['arms']['q_deg'];sc={'name':'asymmetric_foot_candidate','pose':endpoint['right_leg_pose'],'q':qa,'support':'palms_left_foot_right_knee'}
    body,mi,joints2,kn,cu,pa=mod.build_geometry(sc)
    lp,lmeta,lj,frames=mod.env['build_leg'](mod.env['narrow'],endpoint['left_leg_pose'],'narrow')
    assert (V(*lj[2]['origin_mm'])-V(*joints2[2]['origin_mm'])).Length<1e-6
    for n in list(body):
        if mi[n]['group']=='L_leg':body.pop(n);mi.pop(n)
    for n,s in lp.items():
        s=s.copy();s.translate(V(0,60,0));body['L_'+n]=s;mi['L_'+n]={**lmeta[n],'group':'L_leg'}
    K=V(*lj[3]['origin_mm'])+V(0,60,0);h=mod.holder(K);pad=mod.pad(K);name='L_ThighRail1';before=body[name];joined=before.fuse(h).removeSplitter();body[name]=joined;mi[name]['mass_kg']+=(joined.Volume-before.Volume)*2.7e-6
    body['L_KneePad']=pad;mi['L_KneePad']={'mass_kg':pad.Volume*mod.cfg['knee_pad']['density_kg_mm3'],'group':'L_leg','role':'pad'}
    collisions=mod.env['collide'](body,mi);r=mod.env['export']('asymmetric_foot_candidate',body,mi,lj,[])
    fx=endpoint['left_leg_pose']['foot_translation_mm'][0];footpoints=[[fx+dx,60+dy,0] for dx in (-50,50) for dy in (-27.5,27.5)];support=footpoints+[kn[1]]+pa
    r.update(collisions=collisions,ground_penetrations=[n for n,s in body.items() if s.BoundBox.ZMin<-.5],left_joints=lj,right_joints=joints2,
       support_points_mm=support,static_balance=reactions(support,r['approx_COM_mm'],r['known_or_allocated_mass_kg']),left_leg_pose=endpoint['left_leg_pose'],right_leg_pose=endpoint['right_leg_pose'],palm_points_mm=pa)
    result={'input_sha256':hashlib.sha256(PF.read_bytes()).hexdigest(),'toe_steps':rows,'toe_exports':exported,'left_cuff_unload_vertical_balance':unload,'asymmetric_endpoint':r,
       'whole_recovery_validated':False,'limitations':['Incremental checks only omit invariant pairs after full baseline check','Toe contact is a geometrical edge, no pressure/friction/compliance model',
          'Vertical force allocation is a necessary quasi-static condition, not verified force control','Asymmetric endpoint is not a connecting motion','Shoulder/elbow mounting interfaces and real material/thermal limits remain unfinished']}
    (OUT/'contact_validation.json').write_text(json.dumps(result,indent=2)+'\n');print('asymmetric',len(collisions),r['ground_penetrations'],r['static_balance'],flush=True)
if __name__=='__main__':main()

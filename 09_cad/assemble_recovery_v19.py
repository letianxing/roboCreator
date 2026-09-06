"""Full-body candidate with shoulder placement, elbow clearance and explicit knee pads."""
import FreeCAD as A, Part, json,math,hashlib,argparse
from pathlib import Path
from shoulder_scan_v19 import env,cfg,ro,repack,PF
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/body_v19';V=A.Vector
SP=ROOT/'03_system/shoulder_selected_v19.json';sel=json.loads(SP.read_text());phase=json.loads((ROOT/'04_kinematics/recovery_v19/phase_results.json').read_text())
def holder(K):
    k=cfg['knee_pad'];s=Part.makeCylinder(k['holder_outer_radius_mm'],k['holder_width_mm'],K+V(0,k['holder_y_mm']-k['holder_width_mm']/2,0),V(0,1,0))
    return s.cut(Part.makeCylinder(k['holder_inner_radius_mm'],k['holder_width_mm']+2,K+V(0,k['holder_y_mm']-k['holder_width_mm']/2-1,0),V(0,1,0)))
def pad(K):
    k=cfg['knee_pad'];s=Part.makeCylinder(k['outer_radius_mm'],k['width_mm'],K+V(0,k['local_y_mm']-k['width_mm']/2,0),V(0,1,0))
    return s.cut(Part.makeCylinder(k['inner_radius_mm'],k['width_mm']+2,K+V(0,k['local_y_mm']-k['width_mm']/2-1,0),V(0,1,0)))
def build(scene):
    pose=scene['pose'];q=scene['q'];whole,info,joints,nodes=env['body'](pose,'narrow',{'upstream_q_deg':q[:4]});H=V(*joints[2]['origin_mm'])
    beta=pose['foot_pitch_deg']-pose['q_deg'][5]-pose['q_deg'][3]-pose['q_deg'][2];B=ro((0,1,0),beta)
    repack(whole,H,B)
    whole['computer'].translate(B.multVec(V(-100,0,10)))
    whole['microphone'].rotate(env['volcenter'](whole['microphone']),B.multVec(V(1,0,0)),90)
    info['WukongHeadShell']['mass_kg']=.11
    for n in ('MonkeyEarm1','MonkeyEar1'):info.pop(n,None)
    for n in list(info):
        if info[n]['group'].endswith('_arm'):whole.pop(n,None);info.pop(n,None)
    def put(n,s,mass,group,role='structure'):
        assert s.isValid() and s.Volume>0,n
        whole[n]=s;info[n]={'mass_kg':mass,'group':group,'role':role,'source_type':'reference_or_geometry_estimate'}
    centers=[]
    K=V(*joints[3]['origin_mm'])
    for side,tag in [(1,'L'),(-1,'R')]:
        c=K+V(0,60,0);s=holder(c);rubber=pad(c)
        if side==-1:s=env['mirror'](s);rubber=env['mirror'](rubber)
        n=tag+'_ThighRail1';old=whole[n];joined=old.fuse(s).removeSplitter()
        info[n]['mass_kg']+=(joined.Volume-old.Volume)*2.7e-6;whole[n]=joined
        put(tag+'_KneePad',rubber,rubber.Volume*cfg['knee_pad']['density_kg_mm3'],tag+'_leg','pad')
        centers.append([K.x,side*(60+cfg['knee_pad']['local_y_mm']),K.z-cfg['knee_pad']['outer_radius_mm']])
    palm_points=[];cuff_points=[]
    for side,tag in [(1,'L'),(-1,'R')]:
        aq=scene.get('right_q',q) if side==-1 else q
        origin=H+B.multVec(V(*sel['shoulder_origin_body_mm']));R=B;parts=[]
        for i,(axis,off,clock) in enumerate(zip([(0,1,0),(1,0,0),(0,0,1)],sel['motor_offsets_mm'],sel['motor_clock_deg'])):
            T=A.Placement(origin+R.multVec(V(*axis)*off),R.multiply(A.Rotation(V(0,0,1),V(*axis))).multiply(ro((0,0,1),clock)))
            parts.append(('ShoulderMotor'+str(i),env['placed'](env['base']['xm'],T),.082,'motor'));R=R.multiply(ro(axis,aq[i]))
        E=origin+R.multVec(V(0,0,-80));unit=(E-origin).normalize()
        tube=env['base']['rod'](origin,E-unit*9,7)
        parts.append(('UpperArm',tube,.04*71/80,'structure'))
        em=env['placed'](env['base']['xm'],A.Placement(E+R.multVec(V(0,sel['elbow_motor_offset_mm'],0)),R.multiply(A.Rotation(V(0,0,1),V(0,1,0)))))
        parts.append(('ElbowMotor',em,.082,'motor'))
        R=R.multiply(ro((0,1,0),aq[3]));W=E+R.multVec(V(0,0,-85));base_hand_R=R.multiply(ro((0,1,0),90)).multiply(ro((1,0,0),180));T=A.Placement(W-base_hand_R.multVec(V(0,0,25)),base_hand_R)
        wristR=R.multiply(ro((1,0,0),aq[4])).multiply(ro((0,1,0),aq[5]));handR=wristR.multiply(ro((0,1,0),90)).multiply(ro((1,0,0),180));movingT=A.Placement(W-handR.multVec(V(0,0,25)),handR)
        for n,s0 in env['hand'].items():
            s=s0.copy()
            if n=='GauntletPrimaryPad':s=s.cut(env['hand']['ForearmLoadFrame'])
            if n=='ForearmLoadFrame':
                s=s.cut(Part.makeCylinder(4.1,50,V(-85,-25,25),V(0,1,0)))
                s=s.cut(env['base']['box']((0,0,25),(2*sel['wrist_frame_end_clearance_mm'],50,50)))
            trans=movingT if n=='HandAndPalm' else T
            mass=.023 if 'Motor' in n else s.Volume*2.7e-6 if n=='ForearmLoadFrame' else .025 if n=='HandAndPalm' else s.Volume*1.2e-6
            parts.append((n,env['placed'](s,trans),mass,'motor' if 'Motor' in n else 'pad' if 'Pad' in n else 'structure'))
        grip=env['placed'](env['base']['box']((50,0,24),(20,34,26)),movingT);parts.append(('GripperMotor',grip,.023,'motor'))
        for n,s,mass,role in parts:put(tag+'_'+n,s if side==1 else env['mirror'](s),mass,tag+'_arm',role)
        cp=T.multVec(V(-12,0,0));pp=movingT.multVec(V(28,0,.5));cuff_points.append([cp.x,side*cp.y,cp.z]);palm_points.append([pp.x,side*pp.y,pp.z])
    collisions=env['collide'](whole,info);result=env['export'](scene['name'],whole,info,joints,[])
    result.update(collisions=collisions,knee_contact_points_mm=centers,cuff_contact_points_mm=cuff_points,palm_contact_points_mm=palm_points,
                  phase=scene,missing_interfaces=['shoulder gimbal and supporting shafts','elbow clevis and axle across reserved 9mm end space','actual rubber bonding/fasteners and knee carrier strength'],
                  ground_penetrations=[n for n,s in whole.items() if s.BoundBox.ZMin<-.5])
    return result
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--handoff-only',action='store_true');args=parser.parse_args()
    env['OUT']=OUT
    stand=dict(next(p for p in env['base']['cfg']['poses'] if p['name']=='stand'));stand['foot_translation_mm']=[0,0,2]
    first=next(r for r in phase['phase_samples'] if r['arm']['contact_type']=='cuff' and r['arm']['ik_found'])
    palm=next(r for r in phase['phase_samples'] if r['body_height_mm']==90 and r['body_pitch_deg']==60 and r['arm']['contact_type']=='palm')
    scenes=[{'name':'stand_candidate','pose':stand,'q':sel['stand_arm_q_deg'],'support':'feet'},
       {'name':'cuff_knee_candidate','pose':first['leg_pose'],'q':first['arm']['q_deg']+[0,0],'support':'cuffs_and_knees'},
       {'name':'palm_transition_probe','pose':palm['leg_pose'],'q':palm['arm']['q_deg'],'support':'palms_and_knees'}]
    handoff=next(r for r in phase['phase_samples'] if abs(r['body_height_mm']-first['body_height_mm'])<1e-6 and r['arm']['contact_type']=='palm')
    start=first['arm']['q_deg']+[0,0];finish=handoff['arm']['q_deg'];mid=[(a+b)/2 for a,b in zip(start,finish)]
    transfer={'name':'alternating_handoff_midpoint','pose':first['leg_pose'],'q':mid,'right_q':start,'support':'right_cuff_and_knees'}
    scenes=[transfer] if args.handoff_only else scenes+[transfer]
    results=[]
    for sc in scenes:
        r=build(sc);results.append(r);print(sc['name'],'collisions',len(r['collisions']),'mass',r['known_or_allocated_mass_kg'],'width',r['bounds']['size_mm'][1],flush=True)
    out={'input_sha256':hashlib.sha256(PF.read_bytes()).hexdigest(),'selected_input_sha256':hashlib.sha256(SP.read_bytes()).hexdigest(),'scenes':results,
      'scope':'Exact saved-shape checks in three configurations; motor placement does not yet constitute released bearing/shaft interfaces'}
    target='handoff_assembly_results.json' if args.handoff_only else 'assembly_results.json'
    (OUT/target).write_text(json.dumps(out,indent=2)+'\n')
if __name__=='__main__':main()

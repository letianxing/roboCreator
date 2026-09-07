"""Manufacturer X330 geometry and a designed underside neck opening; no hardware release."""
import json,math,itertools,argparse
from neck_chain_v28 import A,Part,V,ROOT,placed,around,load,audit,sha,cfg,FIXED_BOUNDS,INTERNAL_PAIRS
OUT=ROOT/'09_cad/neck_hardware_v28';OUT.mkdir(exist_ok=True)
def main():
    global OUT
    parser=argparse.ArgumentParser();parser.add_argument('--swept-opening',action='store_true');parser.add_argument('--raise-camera',action='store_true');args=parser.parse_args()
    if args.swept_opening:OUT=ROOT/'09_cad/neck_hardware_v28_swept';OUT.mkdir(exist_ok=True)
    if args.raise_camera:
        assert args.swept_opening
        OUT=ROOT/'09_cad/neck_hardware_v28_final';OUT.mkdir(exist_ok=True)
    original,local,T=load();motor=Part.read(str(ROOT/'00_inputs/actuators/XC330/manufacturer.stp'))
    assert motor.isValid() and len(motor.Solids)==15
    yaw_origin=[0,0,125];pitch_origin=[15,0,136]
    if args.raise_camera:pitch_origin=[5,0,146];local['camera'].translate(V(3,0,15))
    # STEP front horn outer mounting plane is native Z=6.5. Body extends back toZ=-22.5 with idler.
    yaw=placed(motor,A.Placement(V(0,0,125-6.5),A.Rotation()))
    # Native X -> body Z, native Y -> body X, native Z -> body Y.
    R=A.Rotation(V(0,0,1),V(1,0,0),V(0,1,0),'ZXY')
    pitch=placed(motor,A.Placement(V(pitch_origin[0],-10,pitch_origin[2])-R.multVec(V(0,0,6.5)),R))
    # Circular underside aperture: separate editable proposal, not deletion of the entire head shell.
    original_head=local['WukongHeadShell']
    opening=Part.makeCylinder(30,33,V(0,0,95),V(0,0,1));envelope=None
    if args.swept_opening:
        def corners(s):
            b=s.cleaned().BoundBox
            return [V(x,y,z) for x,y,z in itertools.product([b.XMin,b.XMax],[b.YMin,b.YMax],[b.ZMin,b.ZMax])]
        ycorners,pcorners=corners(yaw),corners(pitch);bound=A.BoundBox()
        for y in range(-30,31):
            Yi=around(yaw_origin,[0,0,1],y).inverse()
            for p in range(51):
                Pi=around(pitch_origin,[0,1,0],p).inverse()
                for v in ycorners:bound.add(Pi.multVec(Yi.multVec(v)))
        for p in range(51):
            Pi=around(pitch_origin,[0,1,0],p).inverse()
            for v in pcorners:bound.add(Pi.multVec(v))
        ry=max((v-V(*yaw_origin)).Length for v in ycorners)
        rp=ry+(V(*pitch_origin)-V(*yaw_origin)).Length
        rcase=max((v-V(*pitch_origin)).Length for v in pcorners)
        between=max(2*(ry+rp)*math.sin(math.radians(.5)/2),2*rcase*math.sin(math.radians(.5)/2))
        pad=1.;assert between<pad
        lo=V(bound.XMin-pad,bound.YMin-pad,bound.ZMin-pad)
        opening=Part.makeBox(bound.XLength+2*pad,bound.YLength+2*pad,bound.ZLength+2*pad,lo)
        envelope={'lo_body_mm':list(lo),'hi_body_mm':[bound.XMax+pad,bound.YMax+pad,bound.ZMax+pad],
                  'grid_deg':1,'padding_mm':pad,'between_grid_motion_bound_mm':between,
                  'scope':'Conservative continuous containment of nominal motor BOXES in the proposed head-local opening for yaw+-30/pitch0..50 only; not whole-robot continuous clearance.'}
    head=original_head.cut(opening).removeSplitter();assert head.isValid() and len(head.Solids)==len(original_head.Solids)
    if args.raise_camera:
        head=head.cut(Part.makeBox(50,94,30,V(20,-47,135))).removeSplitter()
        assert head.isValid() and len(head.Solids)==len(original_head.Solids)
    local['WukongHeadShell']=head;local['NeckMotor0']=yaw;local['NeckMotor1']=pitch
    FIXED_BOUNDS.clear();INTERNAL_PAIRS.clear();rows=[]
    for yawdeg,pitchdeg in [(0,p) for p in range(0,51,5)]+[(y,p) for y,p in itertools.product([-30,-15,15,30],[0,30,45,50])]:
        Y=around(yaw_origin,[0,0,1],yawdeg);P=around(pitch_origin,[0,1,0],pitchdeg);H=Y.multiply(P)
        body={n:s.copy() for n,s in local.items()}
        for n in cfg['head_rigid_group']:body[n]=placed(local[n],H)
        body['NeckMotor1']=placed(pitch,Y)
        parts={n:placed(s,T) for n,s in body.items()}
        bad=audit(parts,set(cfg['head_rigid_group']+['NeckMotor0','NeckMotor1']))
        row={'yaw_deg':yawdeg,'pitch_deg':pitchdeg,'collisions':bad,'pass':not bad};rows.append(row)
        if (yawdeg,pitchdeg) in [(0,0),(0,45),(0,50),(30,45)]:
            doc=A.newDocument('hardware_neck');
            for n,s in parts.items():
                o=doc.addObject('PartDesign::Feature',n);o.Shape=s;o.addProperty('App::PropertyString','Scope');o.Scope='v28 proposed joint frames using manufacturer sharedX330 geometry; no bracket/horn fastener/physical approval'
            doc.recompute();doc.saveAs(str(OUT/('y'+str(yawdeg)+'_p'+str(pitchdeg)+'.FCStd')));A.closeDocument(doc.Name)
        result={'complete':False,'yaw_origin_body_mm':yaw_origin,'pitch_origin_yaw_neutral_body_mm':pitch_origin,'pitch_front_horn_neutral_body_mm':[pitch_origin[0],-10,pitch_origin[2]],
                'source_motor_sha256':sha(ROOT/'00_inputs/actuators/XC330/manufacturer.stp'),'neck_opening_radius_mm':None if args.swept_opening else 30,'neck_opening_z_body_mm':None if args.swept_opening else [95,128],
                'original_head_volume_mm3':original_head.Volume,'modified_head_volume_mm3':head.Volume,'cases':rows,'swept_motor_opening':envelope,
                'actual_brackets_and_load_path_verified':False,'manufacturer_geometry_includes_idler':True,'camera_center_neutral_body_mm':[43,0,150] if args.raise_camera else [40,0,135]}
        (OUT/'study.json').write_text(json.dumps(result,indent=2)+'\n');print('hardware',yawdeg,pitchdeg,'bad',[(r['a'],r['b']) for r in bad],flush=True)
    result['complete']=True;(OUT/'study.json').write_text(json.dumps(result,indent=2)+'\n')
if __name__=='__main__':main()

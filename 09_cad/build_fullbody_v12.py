"""Whole-robot research assembly, mass properties and quasi-static load screening.
Uses existing ankle solids; other mechanisms and costume are explicitly provisional.
"""
import FreeCAD as A,Part,json,math,itertools,hashlib,argparse
from pathlib import Path
import numpy as np
parser=argparse.ArgumentParser();parser.add_argument('--variant',choices=['v12','v13'],default='v12');args=parser.parse_args()
VER=args.variant;NEW=VER=='v13'
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad'/('fullbody_'+VER);OUT.mkdir(exist_ok=True)
cfg=json.loads((ROOT/'03_system/fullbody_v12_parameters.json').read_text())
arch=json.loads((ROOT/'03_system/architecture_v13.json').read_text()) if NEW else None
if NEW:cfg['geometry'].update(arch['geometry_overrides'])
if NEW:cfg['packaging_adjustments']['neck_pitch_origin_local_mm']=[-40,0,95]
g=cfg['geometry'];ma=cfg['mass_allowances']
poses=json.loads((ROOT/'04_kinematics'/('fullbody_poses_'+VER+'.json')).read_text())['poses']
ankle=A.openDocument(str(ROOT/'09_cad/ankle_v11/wukong_ankle_structure_v11.FCStd'))
ankle_parts={o.Name:(o.Shape.copy(),o.RigidGroup) for o in ankle.Objects if hasattr(o,'Shape') and 'Vendor' not in o.Name}
V=A.Vector
def rot(axis,q):return A.Rotation(V(*axis),math.degrees(q))
def cyl_between(a,b,r):
    v=b-a;return Part.makeCylinder(r,v.Length,a,v.normalize())
def box(center,size):return Part.makeBox(*size,center-V(*[v/2 for v in size]))
def ellipsoid(center,radii):
    m=A.Matrix();m.A11,m.A22,m.A33=radii
    s=Part.makeSphere(1).transformGeometry(m);s.translate(center);return s
def thin_ellipse_ring(center,rx,ry,h):
    outer=Part.makeCylinder(1,h);m=A.Matrix();m.A11=rx;m.A22=ry
    outer=outer.transformGeometry(m);inner=Part.makeCylinder(1,h+2)
    m.A11=rx-2;m.A22=ry-2;inner=inner.transformGeometry(m);inner.translate(V(0,0,-1))
    s=outer.cut(inner);s.translate(center);return s
def polygon_plate(x,yz,thickness):
    points=[V(x,y,z) for y,z in yz];wire=Part.makePolygon(points+[points[0]])
    return Part.Face(wire).extrude(V(thickness,0,0))
def mirror(s,side):
    if side==1:return s.copy()
    m=A.Matrix();m.A22=-1;return s.transformGeometry(m)
def mp(v,side):return V(v.x,side*v.y,v.z)
def maxis(v,side):return v if side==1 else V(-v.x,v.y,-v.z)

def build(pose):
    name=pose['name'];base=V(*pose['root_xyz_mm']);bodyrot=rot((0,1,0),pose['body_pitch_rad'])
    parts=[];joints=[]
    def bp(p):return base+bodyrot.multVec(V(*p))
    def bs(s):
        s=s.copy();s.rotate(V(0,0,0),V(0,1,0),math.degrees(pose['body_pitch_rad']));s.translate(base);return s
    def put(n,s,mass,limb='body',order=-1,category='frame',color=(.4,.45,.5),source='mass allowance / research envelope'):
        assert s.isValid() and s.Volume>0,(n,s.ShapeType,s.Volume)
        solids=s.Solids;assert solids,n
        volume=sum(t.Volume for t in solids)
        center=sum(t.Volume*np.array(list(t.CenterOfMass)) for t in solids)/volume
        inertia=np.zeros((3,3))
        for t in solids:
            im=t.MatrixOfInertia;d=np.array(list(t.CenterOfMass))-center
            inertia+=np.array([[im.A11,im.A12,im.A13],[im.A21,im.A22,im.A23],[im.A31,im.A32,im.A33]])+t.Volume*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
        inertia*=mass/volume*1e-6;cm=V(*center)
        parts.append({'name':n,'shape':s,'mass_kg':float(mass),'com_mm':list(cm),'inertia_kgm2':inertia.tolist(),
                      'limb':limb,'order':order,'category':category,'color':color,'source':source})
    def joint(n,point,axis,limb,order,q):
        joints.append({'name':n,'origin_mm':list(point),'axis':list(axis),'limb':limb,'order':order,'q_rad':float(q)})
    def motor(n,point,axis,limb,order,small=False,outboard=0,frame=None,local_axis=None):
        if NEW and frame is not None:
            side=-1 if n.startswith('R_') else 1
            lp=mp(point,side);la=maxis(axis,side);length=34 if small else 40.2
            center=lp+la*math.copysign(abs(outboard)-length/2-5,outboard) if outboard else lp-la*(length/2+5)
            if small:
                s=box(V(0,0,0),[20,34,26]);orientation=frame.multiply(A.Rotation(V(0,1,0),V(*local_axis)))
                s.Placement=A.Placement(center,orientation)
            else:s=Part.makeCylinder(26.5,length,center-la*length/2,la)
            put(n,mirror(s,side),.023 if small else .190,limb,order,'motor',(.23,.28,.33),'reference module envelope; axial remote mount not mechanically released')
            if outboard:
                sign=1 if outboard>0 else -1
                shaft=cyl_between(lp+la*(sign*10),center-la*(sign*length/2),3 if small else 5)
                put(n+'_transmission_allowance',mirror(shaft,side),arch['remote_small_transmission_mass_each_kg'] if small else arch['remote_main_transmission_mass_each_kg'],limb,order,'frame',(.55,.55,.6),'shaft/bearing/coupler mass allowance; schematic transmission')
            return
        # Rotation axes are axial vectors; placement offsets are polar vectors under mirroring.
        geometry_axis=-axis if n.startswith('R_') else axis
        if small:s=box(point-geometry_axis*17,[20,34,26]);mass=.023
        else:s=Part.makeCylinder(26.5,40.2,point-geometry_axis*45.2,geometry_axis);mass=.190
        put(n,s,mass,limb,order,'motor',(.23,.28,.33),'datasheet mass; approximate uniform-envelope inertia')
    # Two legs, with real ankle component geometry at the assumed fixed foot poses.
    for side in (1,-1):
        tag='L' if side==1 else 'R';limb='leg_'+tag;q=pose['left_leg_q_rad'];r=bodyrot
        pos=bp([0,g['foot_center_half_spacing'] if NEW else 55,0]);origins=[]
        for i,axis in enumerate([(0,0,1),(1,0,0),(0,1,0)]):
            if i==1:pos=pos+r.multVec(V(*(g['hip_offsets'][0])))
            if i==2:pos=pos+r.multVec(V(*(g['hip_offsets'][1])))
            ax=r.multVec(V(*axis));origins.append(pos)
            joint(tag+'_hip_'+str(i),mp(pos,side),maxis(ax,side),limb,i,q[i])
            motor(tag+'_hip_motor_'+str(i),mp(pos,side),maxis(ax,side),limb,i,outboard=arch['hip_driver_outboard_mm'][i] if NEW else 0,frame=r if NEW else None,local_axis=axis)
            r=r.multiply(rot(axis,q[i]))
        hip=pos;knee=hip+r.multVec(V(0,0,-90));ax=r.multVec(V(0,1,0))
        joint(tag+'_knee',mp(knee,side),maxis(ax,side),limb,3,q[3]);motor(tag+'_knee_motor',mp(knee,side),maxis(ax,side),limb,3,frame=r if NEW else None,local_axis=(0,1,0))
        r=r.multiply(rot((0,1,0),q[3]));ap=knee+r.multVec(V(0,0,-90))
        foot_y=g['foot_center_half_spacing'];foot_x=-39 if NEW else 0
        assert (ap-V(39+foot_x,foot_y,45)).Length<1e-5,(name,list(ap))
        joint(tag+'_ankle_pitch',mp(ap,side),maxis(V(0,1,0),side),limb,4,q[4])
        joint(tag+'_ankle_roll',V(-30+foot_x,side*foot_y,45),maxis(V(1,0,0),side),limb,5,0)
        put(tag+'_thigh_frame',mirror(cyl_between(hip,knee,8),side),ma['thigh_frame_each'],limb,2)
        put(tag+'_shin_frame',mirror(cyl_between(knee,ap,8),side),ma['shin_frame_each'],limb,3)
        put(tag+'_hip_mount_allowance',mirror(box(bp([-20,foot_y if NEW else 85,-4]),[55,65,15]),side),ma['hip_mount_each'],limb,0)
        for an,(shape,group) in ankle_parts.items():
            s=shape.copy()
            if group=='pitch':s.rotate(V(39,0,45),V(0,1,0),math.degrees(pose['ankle_shank_rotation_rad']))
            s=mirror(s,side);s.translate(V(foot_x,side*foot_y,0))
            idx={'fixed':5,'roll':4,'pitch':3}[group]
            if an.endswith('ClosedEnvelope'):mass=.190;cat='motor';src='AK40 manufacturer mass; closed envelope inertia'
            elif an=='Bearing6000_Envelope':mass=ma['bearing_each'];cat='frame';src='bearing mass allowance; ring inertia approximation'
            elif an.startswith('BootInterface'):mass=s.Volume*1.28e-6;cat='shell';src='PETG density assumed; CAD volume'
            else:mass=s.Volume*2.7e-6;cat='frame';src='aluminum density assumed; actual candidate CAD volume'
            put(tag+'_'+an,s,mass,limb,idx,cat,(.83,.56,.18) if cat=='shell' else (.42,.49,.57),src)
    # Arms follow the same offset chain as the IK solver, then mirror to the right.
    for side in (1,-1):
        tag='L' if side==1 else 'R';limb='arm_'+tag;r=bodyrot;pos=bp(g['shoulder_local_xyz']);q=pose['left_arm_q_rad'];points=[];frames=[]
        steps=[V(0,0,0),V(0,0,0),V(0,0,-g['upper_arm_length']),V(0,0,-g['forearm_length']),V(0,0,0)] if NEW else [V(0,0,0),V(0,55,0),V(0,0,-55),V(0,0,-80),V(0,0,-80)]
        axes=[(0,1,0),(1,0,0),(0,1,0),(1,0,0),(0,1,0)] if NEW else [(0,1,0),(1,0,0),(0,0,1),(0,1,0),(1,0,0)]
        for i,axis in enumerate(axes):
            pos=pos+r.multVec(steps[i]);ax=r.multVec(V(*axis));points.append(pos)
            frames.append(r)
            joint(tag+'_arm_'+str(i),mp(pos,side),maxis(ax,side),limb,i,q[i]);motor(tag+'_arm_motor_'+str(i),mp(pos,side),maxis(ax,side),limb,i,i>=3 if NEW else i==4,outboard=arch['arm_driver_outboard_mm'][i] if NEW else 0,frame=r if NEW else None,local_axis=axis)
            r=r.multiply(rot(axis,q[i]))
        tip=pos+r.multVec(V(0,0,-40));direction=r.multVec(V(0,0,-1))
        assert (tip-V(*pose['arm_tip_mm'])).Length<1e-4,(name,list(tip),pose['arm_tip_mm'])
        ui,fi=(1,2) if NEW else (2,3)
        put(tag+'_upper_arm_frame',mirror(cyl_between(points[ui],points[ui+1],7),side),ma['upper_arm_frame_each']*(95/80 if NEW else 1),limb,ui)
        put(tag+'_forearm_frame',mirror(cyl_between(points[fi],points[fi+1],7),side),ma['forearm_frame_each']*(100/80 if NEW else 1),limb,fi)
        joint(tag+'_grip',mp(tip,side),maxis(direction,side),limb,5,0)
        if NEW:
            cm=points[3]+frames[3].multVec(V(*arch['gripper_motor_forearm_frame_offset_mm']))
            sh=box(V(0,0,0),[20,34,26]);sh.Placement=A.Placement(cm,frames[3])
            put(tag+'_grip_motor',mirror(sh,side),.023,limb,2,'motor',(.23,.28,.33),'forearm-mounted grip drive; tendon path not designed')
            put(tag+'_grip_transmission',mirror(cyl_between(cm,tip,1),side),.015,limb,2,'frame',(.5,.5,.55),'straight visual guide only, not collision-checked routed cable')
        else:motor(tag+'_grip_motor',mp(tip-direction*12,side),maxis(direction,side),limb,5,True)
        palm=Part.makeSphere(11,tip-direction*11)
        put(tag+'_hand_pad_proxy',mirror(palm,side),ma['gripper_structure_each'],limb,5,'frame',(.56,.28,.12))
        cap_center=points[0]-bodyrot.multVec(V(0,1,0))*25.1
        shoulder_shell=Part.makeSphere(35 if NEW else 38,cap_center).cut(Part.makeSphere(32.5 if NEW else 35,cap_center))
        shoulder_shell=shoulder_shell.common(Part.makeBox(100,100,45,cap_center+V(-50,-50,6)))
        if NEW:shoulder_shell=shoulder_shell.common(Part.makeBox(100,50,100,cap_center+V(-50,12,-50)))
        put(tag+'_gold_shoulder_cap',mirror(shoulder_shell,side),shoulder_shell.Volume*1.28e-6,limb,0,'shell',(.91,.62,.1),'concept shell volume; provisional PETG')
    # Central frame and board envelopes. Positions are provisional packaging, not chosen hardware.
    torso=box(V(0,0,45),[90,110,90]).cut(box(V(0,0,45),[86,106,86]))
    put('Torso_Frame',bs(torso),ma['torso_frame'])
    for n,loc,size,mass,color in [
        ('Battery',[20,0,18] if NEW else [0,0,18],[40,80,32],.25,(.15,.35,.7)),
        ('MicArray',[-38,0,42] if NEW else [0,0,42],[8,88.8,78.1] if NEW else [78.1,88.8,8],.05,(.15,.5,.22)),
        ('Computer',[33,0,60] if NEW else [0,0,53],[10,70,45] if NEW else [45,70,10],.12,(.1,.48,.3)),
        ('Power_Wiring',[-22,0,45] if NEW else cfg['packaging_adjustments']['power_and_wiring_center_local_mm'],[14,25,80] if NEW else [14,80,25],.10,(.25,.25,.3)),
        ('DepthCamera',[0,0,135],[25,90,25],.13,(.1,.1,.12))]:
        put(n,bs(box(V(*loc),size)),mass,category='hardware',color=color)
    for i,axis in enumerate([(0,0,1),(0,1,0)]):
        point=bp([0,0,95] if i==0 else cfg['packaging_adjustments']['neck_pitch_origin_local_mm']);ax=bodyrot.multVec(V(*axis));joint('neck_'+str(i),point,ax,'head',i,0);motor('neck_motor_'+str(i),point,ax,'head',i,True,frame=bodyrot if NEW else None,local_axis=axis)
    put('Head_InternalFrame',bs(box(V(0,0,115),[30,60,15])),ma['head_frame'],'head',1)
    # Wukong costume volumes remain part of the mass model, never hidden behind a cosmetic render.
    def costume(n,s,color,limb='body',order=-1):put(n,bs(s),s.Volume*1.28e-6,limb,order,'shell',color,'concept CAD volume; assumed PETG density, not measured')
    gold=(.91,.61,.12);red=(.65,.06,.08);fur=(.42,.21,.085);skin=(.93,.73,.44)
    hz=135
    head=ellipsoid(V(0,0,hz),[37,58,45]).cut(ellipsoid(V(0,0,hz),[35,56,43]))
    aperture=box(V(32,0,hz),[44,94,28]);neck_open=box(V(0,0,96),[42,94 if NEW else 78,30])
    neck_rear_service=box(V(-40 if NEW else -30,-17,95),[24,38,32])
    costume('Monkey_HeadShell',head.cut(aperture).cut(neck_open).cut(neck_rear_service),fur,'head',1)
    for side in (1,-1):
        costume('Monkey_Ear_'+str(side),ellipsoid(V(0,side*60,hz+1),[9,12,16]),fur,'head',1)
        costume('Monkey_EarInner_'+str(side),ellipsoid(V(7,side*60,hz+1),[3,8,10]),skin,'head',1)
        cheek=ellipsoid(V(25,side*19,hz-5),[12,25,27]).cut(aperture)
        costume('Monkey_Cheek_'+str(side),cheek,skin,'head',1)
    costume('Monkey_Muzzle',ellipsoid(V(33,0,hz-24),[11,27,14]),skin,'head',1)
    costume('Monkey_Nose',ellipsoid(V(43,0,hz-16),[3,7,4]),(.14,.07,.035),'head',1)
    for side in (1,-1):
        costume('Sensor_Eye_'+str(side),Part.makeCylinder(7,1,V(13,side*25,hz),V(1,0,0)),(.1,.7,.85),'head',1)
    smile=Part.makeCompound([cyl_between(V(43,-12,hz-25),V(44,0,hz-29),.7),cyl_between(V(44,0,hz-29),V(43,12,hz-25),.7)])
    costume('Monkey_Smile',smile,(.12,.07,.04),'head',1)
    costume('Gold_CrownBand',thin_ellipse_ring(V(0,0,hz+34),29,45,4),gold,'head',1)
    costume('GreatSage_CrownFront',polygon_plate(25,[(-23,hz+35),(0,hz+65),(23,hz+35)],3),gold,'head',1)
    for side in (1,-1):
        costume('Crown_Wing_'+str(side),polygon_plate(17,[(side*18,hz+36),(side*40,hz+54),(side*35,hz+33)],3),gold,'head',1)
    chest=polygon_plate(46,[(-49,82),(49,82),(43,28),(0,20),(-43,28)],2.5)
    costume('Gold_ChestArmor',chest,gold)
    costume('Red_ChestPanel',polygon_plate(49,[(-29,70),(29,70),(24,39),(0,30),(-24,39)],1.5),red)
    costume('Gold_ChestMedallion',Part.makeCylinder(11,2,V(51,0,53),V(1,0,0)),gold)
    belt=box(V(0,0,10),[100,122,12]).cut(box(V(0,0,10),[92,114,14]))
    if NEW:belt=belt.common(Part.makeCompound([box(V(48,0,10),[5,122,14]),box(V(-48,0,10),[5,122,14])]))
    costume('Red_WaistSash',belt,red)
    for side in (-1,0,1):
        yy=side*35
        costume('Gold_SkirtPlate_'+str(side),polygon_plate(48,[(yy-14,3),(yy+14,3),(yy+12,-36),(yy,-43),(yy-12,-36)],2),gold)
    cloak=box(V(-48,0,52),[2,96,70])
    if NEW:cloak=cloak.cut(neck_rear_service)
    costume('Red_BackPanel',cloak,red)
    assert len(joints)==26 and len([p for p in parts if p['category']=='motor'])==26
    mechanical=[p for p in parts if p['category']!='shell'];shell=[p for p in parts if p['category']=='shell']
    def aggregate(items):
        mass=sum(p['mass_kg'] for p in items);com=sum(p['mass_kg']*np.array(p['com_mm']) for p in items)/mass
        inertia=np.zeros((3,3))
        for p in items:
            d=(np.array(p['com_mm'])-com)/1000
            inertia+=np.array(p['inertia_kgm2'])+p['mass_kg']*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
        return {'mass_kg':mass,'com_mm':com.tolist(),'inertia_about_com_kgm2':inertia.tolist()}
    bare=aggregate(mechanical);full=aggregate(parts)
    carried=pose['payload_carried'];bottle_point=V(*(pose['arm_target_mm'] or [100,33,130]));bottle_point.y=0;bottle_point.z-=15
    supported=list(parts)
    if carried:supported.append({'mass_kg':.58,'com_mm':list(bottle_point),'inertia_kgm2':np.zeros((3,3)).tolist()})
    total=aggregate(supported);mass=total['mass_kg'];com=np.array(total['com_mm'])
    # Nonnegative symmetric reactions, gravity only. Hand support uses chosen target contacts, conditionally.
    hands_fraction=0.
    if pose['palms_support']:hands_fraction=com[0]/pose['arm_target_mm'][0]
    reactions_feasible=bool(0<=hands_fraction<=1)
    foot_cop_x=0. if pose['palms_support'] else float(com[0])
    feet_half=g['foot_center_half_spacing'];foot_center_x=-39 if NEW else 0
    reactions_feasible &= abs(foot_cop_x-foot_center_x)<=75
    ft=mass*9.81*(1-hands_fraction)/2;ht=mass*9.81*hands_fraction/2
    feet_force={1:ft+mass*9.81*com[1]/(2*feet_half),-1:ft-mass*9.81*com[1]/(2*feet_half)}
    reactions_feasible &= min(feet_force.values())>=0
    torque=[]
    for j in joints:
        if j['limb']=='head':continue
        origin=np.array(j['origin_mm']);axis=np.array(j['axis']);moment=np.zeros(3)
        for p in parts:
            if p['limb']==j['limb'] and p['order']>=j['order']:
                moment+=np.cross((np.array(p['com_mm'])-origin)/1000,[0,0,-p['mass_kg']*9.81])
        side=1 if j['limb'].endswith('L') else -1
        if j['limb'].startswith('leg'):
            moment+=np.cross((np.array([foot_cop_x,side*feet_half,0])-origin)/1000,[0,0,feet_force[side]])
        else:
            if carried:moment+=np.cross((np.array(list(bottle_point))-origin)/1000,[0,0,-.58*9.81/2])
            if pose['palms_support']:
                cp=np.array(pose['arm_target_mm']);cp[1]*=side
                moment+=np.cross((cp-origin)/1000,[0,0,ht])
        torque.append({'joint':j['name'],'quasistatic_required_Nm':float(-moment@axis),
                       'conditional_on_contacts':pose['palms_support']})
    # Audit only non-overlapping component categories; frame-to-motor mounting interfaces are not evaluated.
    collisions=[]
    selected=[p for p in parts if p['category'] in ('motor','hardware')]
    for a,b in itertools.combinations(selected,2):
        ba,bb=a['shape'].BoundBox,b['shape'].BoundBox
        if any(getattr(ba,k+'Max')<getattr(bb,k+'Min') or getattr(bb,k+'Max')<getattr(ba,k+'Min') for k in 'XYZ'):continue
        v=a['shape'].common(b['shape']).Volume
        if v>1e-3:collisions.append({'parts':[a['name'],b['name']],'overlap_mm3':v})
    shell_hits=[]
    for a,b in itertools.product(shell,selected):
        ba,bb=a['shape'].BoundBox,b['shape'].BoundBox
        if any(getattr(ba,k+'Max')<getattr(bb,k+'Min') or getattr(bb,k+'Max')<getattr(ba,k+'Min') for k in 'XYZ'):continue
        v=a['shape'].common(b['shape']).Volume
        if v>1e-2:shell_hits.append({'parts':[a['name'],b['name']],'overlap_mm3':v})
    bounds=Part.makeCompound([p['shape'] for p in parts]).optimalBoundingBox()
    summary={'pose':name,'ik':pose,'bare':bare,'with_costume':full,'with_supported_payload':total,
        'appearance_mass_kg':sum(p['mass_kg'] for p in shell),'dimensions_mm':[bounds.XLength,bounds.YLength,bounds.ZLength],
        'bounds_mm':[bounds.XMin,bounds.XMax,bounds.YMin,bounds.YMax,bounds.ZMin,bounds.ZMax],
        'contact_reactions':{'feasible_algebraically':bool(reactions_feasible),'left_foot_N':float(feet_force[1]),'right_foot_N':float(feet_force[-1]),'each_hand_N':float(ht),'foot_cop_x_mm':foot_cop_x,
           'warning':'Only quasi-static force/moment balance; hand contact requires IK/geometry success'},
        'joint_loads':torque,'hardware_interferences':collisions,'appearance_interferences':shell_hits,
        'components':[{k:v for k,v in p.items() if k not in ('shape','color')} for p in parts],'joints':joints}
    if NEW or name in ('standing','handover','getup_intermediate'):
        folder=OUT/name;folder.mkdir(exist_ok=True);doc=A.newDocument('Wukong_'+name+'_'+VER);exports=[];mesh=[]
        mg=doc.addObject('App::DocumentObjectGroup','Mechanical_Research');sg=doc.addObject('App::DocumentObjectGroup','QitianDasheng_Concept')
        for p in parts:
            o=doc.addObject('Part::Feature',p['name']);o.Shape=p['shape'];o.addProperty('App::PropertyString','Status');o.Status=p['source']
            o.addProperty('App::PropertyFloat','AssignedMassKg');o.AssignedMassKg=p['mass_kg'];exports.append(o)
            (sg if p['category']=='shell' else mg).addObject(o)
            vs,fs=p['shape'].tessellate(.8)
            mesh.append({'name':p['name'],'layer':p['category'],'color':p['color'],'points':[list(v) for v in vs],'triangles':[list(f) for f in fs]})
        if carried:
            bottle=Part.makeCylinder(33,200,bottle_point-V(0,0,115)).fuse(Part.makeCone(33,12,20,bottle_point+V(0,0,85))).fuse(Part.makeCylinder(12,10,bottle_point+V(0,0,105)))
            o=doc.addObject('Part::Feature','Bottle_VisualEnvelope');o.Shape=bottle;exports.append(o)
            vs,fs=bottle.tessellate(.8)
            mesh.append({'name':o.Name,'layer':'payload','color':(.22,.6,.9),'points':[list(v) for v in vs],'triangles':[list(f) for f in fs]})
        doc.recompute();doc.saveAs(str(folder/('wukong_fullbody_'+VER+'.FCStd')));Part.export(exports,str(folder/('wukong_fullbody_'+VER+'.step')))
        (folder/'render_mesh.json').write_text(json.dumps(mesh)+'\n');A.closeDocument(doc.Name)
    print(name,'mass',round(full['mass_kg'],3),'shell',round(summary['appearance_mass_kg'],3),'dims',summary['dimensions_mm'],
          'hardware_hits',len(collisions),'shell_hits',len(shell_hits),flush=True)
    return summary

# Kernel mass-properties translation check: inertia must be about component COM.
t=Part.makeBox(2,3,4);i=t.MatrixOfInertia.A11;t.translate(V(100,50,20));assert abs(t.MatrixOfInertia.A11-i)<1e-8
results=[build(p) for p in poses]
(OUT/'wholebody_results.json').write_text(json.dumps({'status':'research_fullbody_not_release','results':results,
 'limitations':['Rigid mirrored limbs; real mounting handedness still unverified','No trajectories or collision-free motion certificate',
   'Material/pack/board/frames mass allowances provisional','Costume not user-approved and may interfere',
   'Inertias uniform-volume approximations, not identified dynamics','Gravity/contact loads omit joint accelerations and impacts']},indent=2)+'\n')

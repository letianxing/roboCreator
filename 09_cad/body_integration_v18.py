"""Pair v17 legs under one body frame and test actual CAD ground/contact constraints."""
import FreeCAD as A, Part, json, math, hashlib
from pathlib import Path
from leg_reuse_v18 import load_v17,narrow_v17,build_leg
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/body_v18';OUT.mkdir(exist_ok=True)
PF=ROOT/'03_system/body_integration_v18.json';cfg=json.loads(PF.read_text(encoding='utf-8'));contacts=json.loads((ROOT/'04_kinematics/body_contacts_v18.json').read_text())
base=load_v17();narrow=load_v17();narrow_v17(narrow);V=A.Vector
srcdoc=A.openDocument(str(ROOT/'09_cad/support_mechanisms_v16/hand_p0_p0.FCStd'));hand={o.Name:o.Shape.copy() for o in srcdoc.Objects if hasattr(o,'Shape')};A.closeDocument(srcdoc.Name)
def ro(axis,deg):return A.Rotation(V(*axis),deg)
def placed(s,T):return base['placed'](s,T)
def translated(s,v):
    o=s.copy();o.translate(v);return o
def mirror(s):
    # Use the kernel's plane mirror, which preserves analytic surfaces and solid orientation.
    return s.mirror(V(0,0,0),V(0,1,0))
def bbox(shape):
    b=shape.BoundBox;return {'min_mm':[b.XMin,b.YMin,b.ZMin],'max_mm':[b.XMax,b.YMax,b.ZMax],'size_mm':[b.XLength,b.YLength,b.ZLength]}
def volcenter(s):
    v=sum(x.Volume for x in s.Solids);c=V(0,0,0)
    for x in s.Solids:c+=x.CenterOfMass*(x.Volume/v)
    return c

def body(pose,variant,armrow=None,include_shell=True):
    env=narrow if variant=='narrow' else base;parts,meta,joints,frames=build_leg(env,pose,variant)
    whole={};info={};half=cfg['foot_center_spacing_mm']/2
    for side,tag in [(1,'L'),(-1,'R')]:
        for name,s in parts.items():
            shape=s.copy() if side==1 else mirror(s)
            shape.translate(V(0,side*half,0));key=tag+'_'+name;whole[key]=shape;info[key]={**meta[name],'group':tag+'_leg'}
    H=V(*joints[2]['origin_mm']);B=frames['body'].Rotation
    def bp(v):return H+B.multVec(V(*v))
    def put(name,s,mass,group,role='structure',status='assumed'):
        assert s.isValid() and s.Volume>0,name
        whole[name]=s;info[name]={'mass_kg':mass,'group':group,'role':role,'source_type':status}
    def localbox(name,c,sz,mass,group,role='structure'):
        put(name,placed(base['box'](c,sz),A.Placement(H,B)),mass,group,role)
    plate=base['box'](cfg['pelvis_plate_center_body_mm'],cfg['pelvis_plate_size_mm'])
    put('PelvisPlate',placed(plate,A.Placement(H,B)),plate.Volume*2.7e-6,'body')
    # Electronics use their documented/assumed full-size blocks; no shrinking to fit.
    for name,row in cfg['electronics'].items():localbox(name,row['center_body_mm'],row['size_mm'],row['mass_allowance_kg'],'electronics','electronics')
    q=armrow['upstream_q_deg'] if armrow else [-20,10,0,45]
    arm_nodes=[]
    for side,tag in [(1,'L'),(-1,'R')]:
        aparts=[];origin=bp([0,75,80]);R=B
        def armput(label,s,mass,role):aparts.append((label,s,mass,role))
        axes=[(0,1,0),(1,0,0),(0,0,1)];offsets=[cfg['shoulder_offsets_mm']['pitch_y'],cfg['shoulder_offsets_mm']['roll_x'],cfg['shoulder_offsets_mm']['yaw_z']]
        for i,(axis,off) in enumerate(zip(axes,offsets)):
            s=placed(base['xm'],A.Placement(origin+R.multVec(V(*axis)*off),R.multiply(A.Rotation(V(0,0,1),V(*axis)))))
            armput('ShoulderMotor'+str(i),s,.082,'motor');R=R.multiply(ro(axis,q[i]))
        E=origin+R.multVec(V(0,0,-80));upper=base['rod'](origin,E,7);armput('UpperArm',upper,.04,'structure')
        motor=placed(base['xm'],A.Placement(E+R.multVec(V(0,30,0)),R.multiply(A.Rotation(V(0,0,1),V(0,1,0)))))
        armput('ElbowMotor',motor,.082,'motor');R=R.multiply(ro((0,1,0),q[3]));W=E+R.multVec(V(0,0,-85))
        RH=R.multiply(ro((0,1,0),90)).multiply(ro((1,0,0),180));T=A.Placement(W-RH.multVec(V(0,0,25)),RH)
        for n,s in hand.items():
            mass=.023 if 'Motor' in n else .029487495670262796 if n=='ForearmLoadFrame' else .025 if n=='HandAndPalm' else .005
            armput(n,placed(s,T),mass,'motor' if 'Motor' in n else 'structure')
        gripper=placed(base['box']((50,0,24),(20,34,26)),T);armput('GripperMotor',gripper,.023,'motor')
        for n,s,mass,role in aparts:
            if side==-1:s=mirror(s)
            put(tag+'_'+n,s,mass,tag+'_arm',role)
        arm_nodes.append({'side':tag,'shoulder_mm':[origin.x,side*origin.y,origin.z],
                          'pad_center_mm':list(T.multVec(V(-12,0,0))) if side==1 else [T.multVec(V(-12,0,0)).x,-T.multVec(V(-12,0,0)).y,T.multVec(V(-12,0,0)).z]})
    for i,center in enumerate([[-25,0,122],[-25,0,155]]):localbox('NeckMotor'+str(i),center,[20,34,26],.023,'neck','motor')
    if include_shell:
        # Thin head shell with camera opening, ears and simple crown occupancy.
        def ellipse(c,rad):
            mat=A.Matrix();mat.A11,mat.A22,mat.A33=rad;s=Part.makeSphere(1).transformGeometry(mat);s.translate(V(*c));return s
        outer=ellipse([0,0,137],[37,55,38]);inner=ellipse([0,0,137],[35,53,36]);head=outer.cut(inner)
        head=head.cut(base['box']((28,0,135),(35,94,30)))
        put('WukongHeadShell',placed(head,A.Placement(H,B)),.09,'appearance','shell')
        for side in (-1,1):
            ear=ellipse([-3,side*55,137],[10,9,17]);put('MonkeyEar'+str(side).replace('-','m'),placed(ear,A.Placement(H,B)),.01,'appearance','shell')
        crown=base['box']((0,0,178),(35,35,4));put('CrownOccupancy',placed(crown,A.Placement(H,B)),.02,'appearance','shell')
    return whole,info,joints,arm_nodes

def collide(parts,info,cross_only=False):
    keys=list(parts);rows=[]
    for i,a in enumerate(keys):
        for b in keys[i+1:]:
            if cross_only and info[a]['group']==info[b]['group']:continue
            sa,sb=parts[a],parts[b]
            if sa.BoundBox.intersect(sb.BoundBox):
                v=sa.common(sb).Volume
                if v>1e-5:rows.append({'a':a,'b':b,'volume_mm3':v,'group_a':info[a]['group'],'group_b':info[b]['group']})
    return rows

def export(name,parts,info,joints,nodes):
    doc=A.newDocument(name);objects=[];mesh=[]
    for n,s in parts.items():
        o=doc.addObject('PartDesign::Feature',n);o.Shape=s;o.addProperty('App::PropertyString','Scope');o.Scope='v18 assembly study; unresolved interfaces and collisions retained';objects.append(o)
        # Use motor/electronics boxes in the preview to reduce file size; CAD retains actual shapes.
        b=s.BoundBox;visual=Part.makeBox(b.XLength,b.YLength,b.ZLength,V(b.XMin,b.YMin,b.ZMin)) if info[n]['role'] in ('motor','electronics') else s
        pts,tris=visual.tessellate(1);mesh.append({'name':n,'role':info[n]['role'],'group':info[n]['group'],'points':[list(v) for v in pts],'triangles':tris})
    doc.recompute();f=OUT/(name+'.FCStd');doc.saveAs(str(f));Part.export(objects,str(OUT/(name+'.step')));A.closeDocument(doc.Name)
    reopened=A.openDocument(str(f));assert all(o.Shape.isValid() for o in reopened.Objects if hasattr(o,'Shape'));A.closeDocument(reopened.Name)
    (OUT/(name+'_mesh.json')).write_text(json.dumps(mesh)+'\n')
    allshape=Part.makeCompound(list(parts.values()));known=sum(r['mass_kg'] for r in info.values() if r['mass_kg'] is not None)
    com=V(0,0,0)
    for n,row in info.items():
        if row['mass_kg'] is not None:com+=volcenter(parts[n])*(row['mass_kg']/known)
    return {'name':name,'bounds':bbox(allshape),'known_or_allocated_mass_kg':known,'approx_COM_mm':list(com),
            'motor_count':sum(r['role']=='motor' for r in info.values()),'component_minimum_z_mm':{n:s.BoundBox.ZMin for n,s in parts.items()},
            'arm_nodes':nodes,'manufacturable':False,'task_verified':False}

# Compare baseline/narrow neutral and standing package first.
stand=next(p for p in base['cfg']['poses'] if p['name']=='stand');rows=[]
for variant in ('baseline','narrow'):
    parts,info,joints,nodes=body(stand,variant)
    result=export('stand_'+variant,parts,info,joints,nodes);result['collisions']=collide(parts,info)
    # Leg-only bilateral width is separated from shoulders, head and total body width.
    result['legs_bounds']=bbox(Part.makeCompound([s for n,s in parts.items() if info[n]['group'].endswith('_leg')]))
    rows.append(result);print(variant,'mass',result['known_or_allocated_mass_kg'],'width',result['bounds']['size_mm'][1],'leg width',result['legs_bounds']['size_mm'][1],'collisions',len(result['collisions']),flush=True)

# Screen all knee-contact leg candidates by actual geometry against the ground.
ground=[];cached=[]
for i,trial in enumerate(contacts['kneeling_candidates']):
    parts,meta,joints,frames=build_leg(narrow,trial['pose'],'narrow')
    minz=min(s.BoundBox.ZMin for n,s in parts.items() if meta[n]['role']!='keepout')
    entry={'index':i,'branch':trial['branch'],'q_deg':trial['pose']['q_deg'],'minimum_leg_z_mm':minz,
           'penetrating_parts':[n for n,s in parts.items() if meta[n]['role']!='keepout' and s.BoundBox.ZMin<-cfg['ground_tolerance_mm']], 'leg_self_collisions':None}
    if minz>=-cfg['ground_tolerance_mm']:
        imeta={n:{**m,'group':m['rigid_group']} for n,m in meta.items()};entry['leg_self_collisions']=collide(parts,imeta);cached.append((i,parts,meta))
    ground.append(entry)
    if i%10==0:print('knee candidates checked',i+1,'/',len(contacts['kneeling_candidates']),flush=True)
valid=[r for r in ground if not r['penetrating_parts'] and r['leg_self_collisions']==[]]
pool=valid if valid else [r for r in ground if not r['penetrating_parts']] or ground
chosen=min(pool,key=lambda r:(len(r['leg_self_collisions'] or []) if r['leg_self_collisions'] is not None else 999,len(r['penetrating_parts']),-r['minimum_leg_z_mm']))
trial=contacts['kneeling_candidates'][chosen['index']]
parts,info,joints,nodes=body(trial['pose'],'narrow',contacts['arm_candidate'])
probe=export('kneeling_contact_probe',parts,info,joints,nodes);probe['collisions']=collide(parts,info);probe['selected_leg_screen']=chosen;rows.append(probe)
summary={'input_sha256':hashlib.sha256(PF.read_bytes()).hexdigest(),'scenes':rows,'knee_contact_trials':ground,'knee_candidates_without_ground_or_self_collision':len(valid),
         'limitations':['Right-side CAD is a reflected packaging reference, actual motor mounting/handedness must be verified',
          'All positive common volumes retained; no collision exemptions for intended joints',
          'Knee ground clearance alone does not validate a protective knee pad or load path',
          'Mass combines supplier motor values, approximate geometry densities and provisional allocations',
          'No wholebody task, appearance approval, dynamics or manufacturing release']}
(OUT/'body_results.json').write_text(json.dumps(summary,indent=2)+'\n')
print('knee valid candidates',len(valid),'selected',chosen,'wholebody collisions',len(probe['collisions']),flush=True)

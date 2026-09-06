"""Six-joint single-leg integration, preserving source geometry and auditing every pair."""
import FreeCAD as A, Part, json, math, hashlib, itertools, argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/single_leg_v17';OUT.mkdir(exist_ok=True)
parser=argparse.ArgumentParser();parser.add_argument('--repair-interfaces',action='store_true');args=parser.parse_args()
if args.repair_interfaces:OUT=OUT/'interface_revision';OUT.mkdir(exist_ok=True)
PFILE=ROOT/'03_system/single_leg_v17.json';cfg=json.loads(PFILE.read_text(encoding='utf-8'));V=A.Vector
source=ROOT/cfg['source_ankle'];source_hash=hashlib.sha256(source.read_bytes()).hexdigest()
doc=A.openDocument(str(source));src={o.Name:o.Shape.copy() for o in doc.Objects if hasattr(o,'Shape')};A.closeDocument(doc.Name)
old=json.loads((ROOT/'09_cad/support_mechanisms_v16/mechanism_results.json').read_text());oldmass=next(r['mass_from_geometry_kg'] for r in old['records'] if r['pose']=='ankle_p0_p0')
xm=Part.Shape();xm.read(str(ROOT/'00_inputs/actuators/XM430/XM_H-430_idler.stp'))
def rot(axis,deg):return A.Rotation(V(*axis),deg)
def placed(s,T):
    out=s.copy();out.Placement=T.multiply(s.Placement);return out
def frame(point,R):return A.Placement(point,R)
def box(c,sz):return Part.makeBox(*sz,V(*c)-V(*[x/2 for x in sz]))
def rod(a,b,r):
    delta=b-a;return Part.makeCylinder(r,delta.Length,a,delta.normalize())
def union(parts):
    s=parts[0]
    for t in parts[1:]:s=s.fuse(t)
    return s.removeSplitter()
def cx(x,y,z,length,outer,inner=0):
    s=Part.makeCylinder(outer,length,V(x-length/2,y,z),V(1,0,0))
    return s.cut(Part.makeCylinder(inner,length+2,V(x-length/2-1,y,z),V(1,0,0))) if inner else s
if args.repair_interfaces:
    # Replace concept overlaps with explicit clearances, bearing pockets and shafts.
    pieces=[]
    for solid in src['FixedFrame'].Solids:
        s=solid.copy();s.translate(V(5 if s.BoundBox.XMin>0 else -5,0,0));pieces.append(s)
    pieces += [box((0,y,114),(72,4,6)) for y in (-38,14)]
    pieces += [box((x,0,z),(6,24,24)) for x in (-34,34) for z in (72,100)]
    fixed=union(pieces)
    for z in (72,100):fixed=fixed.cut(Part.makeCylinder(8,100,V(-50,0,z),V(1,0,0)))
    src['FixedFrame']=fixed
    src['FixedBearings']=Part.makeCompound([cx(x,0,z,6,8,4.1) for x in (-34,34) for z in (72,100)])
    carrier=src['RollCarrier'].cut(Part.makeCylinder(27.0,42.2,V(0,-21.1,35),V(0,1,0)))
    carrier=carrier.fuse(cx(0,0,72,40,8,4)).cut(Part.makeCylinder(4,100,V(-50,0,72),V(1,0,0)))
    src['RollCarrier']=carrier.removeSplitter()
    foot=[box((0,0,2),(110,66,4))]
    for y in (-31,31):
        side=box((0,y,24),(60,4,40)).cut(box((0,y,18),(40,6,16)))
        side=side.cut(Part.makeCylinder(4.2,8,V(0,y-4,35),V(0,1,0)));foot.append(side)
    src['FootStructure']=union(foot)
    for n in ('FixedFrame','RollCarrier','FootStructure'):oldmass[n]=src[n].Volume*2.7e-6
    oldmass['FixedBearings']=src['FixedBearings'].Volume*7.85e-6
def bounds(shapes):
    b=Part.makeCompound(list(shapes)).BoundBox
    return {'min_mm':[b.XMin,b.YMin,b.ZMin],'max_mm':[b.XMax,b.YMax,b.ZMax],'size_mm':[b.XLength,b.YLength,b.ZLength]}

def build(p,koffset):
    hy,hr,hp,k,ar,ap=p['q_deg'];Rf=rot((0,1,0),p['foot_pitch_deg']);Tf=frame(V(*p['foot_translation_mm']),Rf)
    # Full inverse of the v16 foot transform, giving a planted foot in world coordinates.
    localP=V(0,0,cfg['pitch_axis_height_mm']);localR=V(0,0,72)
    P=Tf.multVec(localP);Rc=Rf.multiply(rot((0,1,0),-ap));R= P+Rc.multVec(V(0,0,37))
    Rs=Rc.multiply(rot((1,0,0),-ar));K=R+Rs.multVec(V(0,0,cfg['knee_to_roll_axis_mm']))
    Rt=Rs.multiply(rot((0,1,0),-k));H=K+Rt.multVec(V(0,0,cfg['thigh_length_mm']))
    Rh=Rt.multiply(rot((0,1,0),-hp));HR=H+Rh.multVec(V(0,0,24))
    Rhr=Rh.multiply(rot((1,0,0),-hr));HY=HR+Rhr.multVec(V(0,0,24));Rb=Rhr.multiply(rot((0,0,1),-hy))
    Tc=frame(P-Rc.multVec(localP),Rc);Ts=frame(R-Rs.multVec(localR),Rs)
    parts={};meta={}
    def put(name,shape,group,mass=None,source_type='assumed',role='structure'):
        assert shape.isValid() and shape.Volume>0,name
        parts[name]=shape;meta[name]={'rigid_group':group,'mass_kg':mass,'source_type':source_type,'role':role}
    for n,s in src.items():
        if n in ('RollCranks','RollRod'):continue
        group='foot' if n in ('FootStructure','SolePad') else 'carrier' if n in ('RollCarrier','PitchMotorEnvelope') else 'shin'
        T={'foot':Tf,'carrier':Tc,'shin':Ts}[group]
        mass=.19 if n=='PitchMotorEnvelope' else .082 if n=='RollMotorVendor' else oldmass[n]
        put(n,placed(s,T),group,mass,'v16_source_CAD' if mass is not None else 'unresolved', 'motor' if 'Motor' in n else 'structure')
    a=math.radians(ar);x=44 if args.repair_interfaces else 36;r=15
    top=V(x,r*math.cos(a),100+r*math.sin(a));bot=V(x,r*math.cos(a),72+r*math.sin(a))
    linkage=[('RollRod',rod(bot,top,2)),('RollCrankTop',rod(V(x,0,100),top,4)),('RollCrankBottom',rod(V(x,0,72),bot,4))]
    if args.repair_interfaces:
        linkage=[]
        rodtop=V(49,top.y,top.z);rodbot=V(49,bot.y,bot.z)
        linkrod=union([rod(rodbot,rodtop,2),cx(49,top.y,top.z,4,5,2),cx(49,bot.y,bot.z,4,5,2)])
        # Drill both assembled eye holes after union; earlier overlapping pin volumes are removed.
        for point in (top,bot):linkrod=linkrod.cut(Part.makeCylinder(2,20,V(39,point.y,point.z),V(1,0,0)))
        linkage.append(('RollRod',linkrod))
        for label,z in [('Top',100),('Bottom',72)]:
            crank=union([cx(44,0,z,4,6),cx(44,15,z,4,6),box((44,7.5,z),(4,15,12))])
            for y,rr in [(0,4),(15,2)]:crank=crank.cut(Part.makeCylinder(rr,8,V(40,y,z),V(1,0,0)))
            crank.rotate(V(44,0,z),V(1,0,0),ar);linkage.append(('RollCrank'+label,crank))
        for i,point in enumerate((top,bot)):linkage.append(('RollPin'+str(i),cx(46.5,point.y,point.z,13,2)))
        linkage += [('RollOutputShaft',cx(0,0,72,92,4)),('RollInputExtension',cx(35,0,100,22,4)),('RollIdleExtension',cx(-28,0,100,18,4))]
    for n,s in linkage:
        rho=cfg['densities_kg_mm3']['aluminum' if 'Crank' in n else 'steel'];put(n,placed(s,Ts),'linkage',s.Volume*rho,'CAD_density')
    def ak(name,origin,Rj,offset,group):
        center=origin+Rj.multVec(V(0,offset,0));axis=Rj.multVec(V(0,1,0))
        s=Part.makeCylinder(26.5,40.2,center-axis*20.1,axis);put(name,s,group,.19,'datasheet','motor')
    ak('KneeMotor',K,Rt,koffset,'thigh');ak('HipPitchMotor',H,Rh,cfg['hip_pitch_outboard_mm'],'hip_pitch_parent')
    for name,origin,Rj,axis,off,group in [('HipRollMotor',HR,Rhr,(1,0,0),cfg['hip_roll_outboard_mm'],'hip_roll_parent'),('HipYawMotor',HY,Rb,(0,0,1),cfg['hip_yaw_outboard_mm'],'pelvis')]:
        orientation=Rj.multiply(A.Rotation(V(0,0,1),V(*axis)));position=origin+Rj.multVec(V(*axis)*off)
        put(name,placed(xm,frame(position,orientation)),group,.082,'datasheet','motor')
    # Separate physical connecting rails; motor/output hole patterns remain to be designed.
    for side in (-1,1):
        rail=box((side*35,-12,123),(4,48,10));put('ShinKneeRail'+str(side).replace('-','m'),placed(rail,Ts),'shin',rail.Volume*2.7e-6,'CAD_density')
        # Parallel side plates lie in separate Y planes from the shin and motor body.
        # Their increased width is reported; attachment shafts/holes are unresolved.
        side_y=-47 if side==-1 else 72
        beam=box((0,side_y,45),(28,4,90));put('ThighRail'+str(side).replace('-','m'),placed(beam,frame(K,Rt)),'thigh',beam.Volume*2.7e-6,'CAD_density')
    # An explicit cable service-space cylinder: counted as keepout, never as verified routing.
    cable=rod(V(-40,-20,82),V(-40,-20,125),3)
    put('CableServiceSpace',placed(cable,Ts),'shin',None,'assumed','keepout')
    joints=[{'name':n,'origin_mm':list(o),'axis':list(Rj.multVec(V(*ax))),'q_deg':q} for n,o,Rj,ax,q in [
        ('hip_yaw',HY,Rb,(0,0,1),hy),('hip_roll',HR,Rhr,(1,0,0),hr),('hip_pitch',H,Rh,(0,1,0),hp),
        ('knee_pitch',K,Rt,(0,1,0),k),('ankle_roll',R,Rs,(1,0,0),ar),('ankle_pitch',P,Rc,(0,1,0),ap)]]
    assert abs((K-H).Length-90)<1e-8 and abs((R-K).Length-53)<1e-8 and abs((P-R).Length-37)<1e-8
    return parts,meta,joints,{'foot':Tf,'shin':Ts,'carrier':Tc,'body':frame(HY,Rb)}

def pair_audit(parts,meta,only_knee=False):
    collisions=[];mindist=1e9
    for a,b in itertools.combinations(parts,2):
        if only_knee and 'KneeMotor' not in (a,b):continue
        # Use bounding boxes only for broad phase, exact common solid for intersections.
        sa,sb=parts[a],parts[b];d=sa.distToShape(sb)[0] if only_knee else None
        if only_knee:mindist=min(mindist,d)
        if sa.BoundBox.intersect(sb.BoundBox):
            v=sa.common(sb).Volume
            if v>1e-5:collisions.append({'a':a,'b':b,'volume_mm3':v,'same_rigid_group':meta[a]['rigid_group']==meta[b]['rigid_group'],
                'involves_keepout':meta[a]['role']=='keepout' or meta[b]['role']=='keepout','source_inherited':a in src and b in src})
    return collisions,mindist

scan=[]
for off in cfg['knee_outboard_scan_mm']:
    cases=[]
    for p in cfg['poses']:
        parts,meta,joints,T=build(p,off);collisions,gap=pair_audit(parts,meta,True)
        cases.append({'pose':p['name'],'knee_collisions':collisions,'knee_min_gap_mm':gap})
    scan.append({'knee_outboard_mm':off,'cases':cases,'total_intersections':sum(len(c['knee_collisions']) for c in cases),'minimum_gap_mm':min(c['knee_min_gap_mm'] for c in cases)})
valid=[r for r in scan if r['total_intersections']==0 and r['minimum_gap_mm']>=cfg['clearance_target_mm']]
chosen=min(valid,key=lambda x:x['knee_outboard_mm']) if valid else min(scan,key=lambda x:(x['total_intersections'],-x['minimum_gap_mm']))
print('knee scan',[(r['knee_outboard_mm'],r['total_intersections'],round(r['minimum_gap_mm'],2)) for r in scan],flush=True)
records=[];meshes=[]
for p in cfg['poses']:
    parts,meta,joints,T=build(p,chosen['knee_outboard_mm']);collisions,_=pair_audit(parts,meta)
    doc=A.newDocument('Leg_v17_'+p['name']);objects=[];rows=[]
    for n,s in parts.items():
        o=doc.addObject('PartDesign::Feature',n);o.Shape=s;o.addProperty('App::PropertyString','RigidGroup');o.RigidGroup=meta[n]['rigid_group'];objects.append(o)
        volume=sum(t.Volume for t in s.Solids)
        center=V(0,0,0)
        for solid in s.Solids:center+=solid.CenterOfMass*(solid.Volume/volume)
        rows.append({'name':n,**meta[n],'center_of_volume_mm':list(center),'solid_count':len(s.Solids)})
        pts,tris=s.tessellate(1.0);meshes.append({'pose':p['name'],'name':n,'role':meta[n]['role'],'points':[list(v) for v in pts],'triangles':tris})
    doc.recompute();f=OUT/(p['name']+'.FCStd');doc.saveAs(str(f));Part.export(objects,str(OUT/(p['name']+'.step')));A.closeDocument(doc.Name)
    reopened=A.openDocument(str(f));assert all(o.Shape.isValid() for o in reopened.Objects if hasattr(o,'Shape'));A.closeDocument(reopened.Name)
    known=sum(x['mass_kg'] for x in rows if x['mass_kg'] is not None);motor_count=sum(x['role']=='motor' for x in rows);assert motor_count==6
    records.append({'pose':p['name'],'input':p,'joints':joints,'parts':rows,'known_mass_kg':known,'missing_mass_parts':[x['name'] for x in rows if x['mass_kg'] is None and x['role']!='keepout'],
        'bounds':bounds(parts.values()),'collisions':collisions,'manufacturable':False,'kinematic_chain_closure_checked':True,
        'ideal_crown_point_mm':list(T['body'].multVec(V(0,0,132))),'note':'Crown point is a body-frame requirement marker, not a head/torso or dressed assembly.'})
    print(p['name'],'mass',known,'collisions',len(collisions),flush=True)
assert hashlib.sha256(source.read_bytes()).hexdigest()==source_hash
summary={'input_sha256':hashlib.sha256(PFILE.read_bytes()).hexdigest(),'source_sha256':source_hash,'freecad_version':A.Version(),'knee_scan':scan,'chosen':chosen,
         'interface_revision':args.repair_interfaces,
         'records':records,'limitations':['All positive-volume pairs retained, including same-group intersections requiring joining details',
         'Inherited v16 interfaces still unresolved; this is a full-length geometric integration study',
         'No manufacturable shaft/bolt/bearing or verified cable routing', 'Five isolated configurations, no common pelvis gait or fullbody support proof']}
(OUT/'integration_results.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
(OUT/'render_mesh.json').write_text(json.dumps(meshes)+'\n',encoding='utf-8')

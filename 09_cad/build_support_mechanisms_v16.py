"""Build local ankle and forearm support mechanism concepts in FreeCAD."""
import FreeCAD as A, Part, json, math, hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/support_mechanisms_v16';OUT.mkdir(exist_ok=True)
PFILE=ROOT/'03_system/support_mechanisms_v16.json';cfg=json.loads(PFILE.read_text(encoding='utf-8'));V=A.Vector
DENS={'aluminum':2.70e-6,'steel':7.85e-6}
xm=Part.Shape();xm.read(str(ROOT/'00_inputs/actuators/XM430/XM_H-430_idler.stp'));assert xm.isValid() and xm.Solids
def centerbox(c,s):return Part.makeBox(*s,V(c[0]-s[0]/2,c[1]-s[1]/2,c[2]-s[2]/2))
def cylinder_between(a,b,r):
    a,b=V(*a),V(*b);d=b-a;return Part.makeCylinder(r,d.Length,a,d.normalize())
def ring(axis_center,axis,length,ro,ri):
    c=V(*axis_center);u=V(*axis);return Part.makeCylinder(ro,length,c-u*length/2,u).cut(Part.makeCylinder(ri,length+2,c-u*(length/2+1),u))
def rotate(s,center,axis,deg):
    t=s.copy();t.rotate(V(*center),V(*axis),deg);return t
def compound(parts):return Part.makeCompound(parts)
def fuse(parts):
    s=parts[0]
    for x in parts[1:]:s=s.fuse(x)
    return s.removeSplitter()
def add(doc,name,shape,category,color,material=None):
    assert shape.isValid() and shape.Volume>0,(name,shape.Volume)
    o=doc.addObject('PartDesign::Feature',name);o.Shape=shape;o.addProperty('App::PropertyString','Category');o.Category=category
    o.addProperty('App::PropertyString','EvidenceScope');o.EvidenceScope='v0.16 mechanism concept; not manufacturing release'
    if o.ViewObject is not None:o.ViewObject.ShapeColor=color
    return o,shape.Volume*DENS[material] if material else None
def xm_axis_x(center):
    # Native X becomes world Y and native Y becomes world Z: lower Z height, wider calf.
    s=xm.copy();s.Placement=A.Placement(V(*center),A.Rotation(V(0,0,1),V(1,0,0)));return s
def ak_axis_y(center):return Part.makeCylinder(26.5,40.2,V(*center)-V(0,20.1,0),V(0,1,0))
def ankle_shapes(roll_deg,pitch_deg):
    a=cfg['ankle'];R=a['roll_axis_xyz_mm'];P=a['pitch_axis_xyz_mm'];M=a['roll_motor_axis_xyz_mm'];x=a['crank_plane_x_mm'];r=a['crank_radius_mm']
    frames=[]
    for xp in (-29,29):
        frames += [centerbox((xp,-38,92),(4,4,50)),centerbox((xp,14,92),(4,4,50)),centerbox((xp,-12,114),(4,52,6)),centerbox((xp,-12,75),(4,52,8))]
    fixed_frame=fuse(frames);fixed_bearings=compound([ring(R,(1,0,0),4,8,4.1),ring(M,(1,0,0),4,8,4.1)]);roll_motor=xm_axis_x(M)
    motor_mount_center=(P[0],-22.6,P[2])
    carrier0=fuse([centerbox((-25,-22.5,53.5),(5,5,37)),centerbox((25,-22.5,53.5),(5,5,37)),centerbox((0,-11,69.5),(54,27,5)),ring(motor_mount_center,(0,1,0),5,29,26.7)])
    carrier=rotate(carrier0,R,(1,0,0),roll_deg);ak=rotate(ak_axis_y(P),R,(1,0,0),roll_deg)
    sole=centerbox((0,0,2),a['sole_mm']);pad=centerbox((0,0,-1),a['sole_pad_mm']);cheeks=[]
    for y in (-23,23):
        plate=centerbox((0,y,19.5),(60,4,31));window=centerbox((0,y,18.5),(40,6,17));cheeks.append(plate.cut(window))
    foot0=fuse([sole]+cheeks);foot=rotate(rotate(foot0,P,(0,1,0),pitch_deg),R,(1,0,0),roll_deg);pad=rotate(rotate(pad,P,(0,1,0),pitch_deg),R,(1,0,0),roll_deg)
    th=math.radians(roll_deg);off=V(0,r*math.cos(th),r*math.sin(th));top=V(x,M[1],M[2])+off;bottom=V(x,R[1],R[2])+off
    rod=cylinder_between(tuple(bottom),tuple(top),2);cranks=compound([cylinder_between((x,M[1],M[2]),tuple(top),4),cylinder_between((x,R[1],R[2]),tuple(bottom),4)])
    return {'FixedFrame':fixed_frame,'FixedBearings':fixed_bearings,'RollMotorVendor':roll_motor,'RollCarrier':carrier,'PitchMotorEnvelope':ak,'FootStructure':foot,'SolePad':pad,'RollRod':rod,'RollCranks':cranks}
def hand_shapes(pitch_deg):
    h=cfg['forearm_support'];W=h['wrist_axis_xyz_mm'];fixed=[centerbox((-42.5,-15,25),(85,4,8)),centerbox((-42.5,15,25),(85,4,8)),centerbox((-75,0,25),(8,34,12))]
    for y in (-15,15):fixed.append(cylinder_between((-35,y,25),(-12,y,4.5),4))
    fixed_frame=fuse(fixed);gauntlet=centerbox((-12,0,1.5),h['gauntlet_pad_mm']);xc1=centerbox((-38,0,45),(34,20,26));xc2=centerbox((-8,0,50),(20,34,26))
    # Palm bottom is 0.5 mm above the primary gauntlet pad in the support stop pose.
    palm0=centerbox((28,0,3.0),(52,36,5));root0=fuse([centerbox((3,-15,14.25),(6,4,21.5)),centerbox((3,15,14.25),(6,4,21.5)),palm0]);palm=rotate(root0,W,(0,1,0),pitch_deg)
    return {'ForearmLoadFrame':fixed_frame,'GauntletPrimaryPad':gauntlet,'WristMotorEnvelope0':xc1,'WristMotorEnvelope1':xc2,'HandAndPalm':palm}
records=[];render=[]
for kind,poses in [('ankle',cfg['ankle']['pose_samples_deg']),('hand',[[v,0] for v in cfg['forearm_support']['wrist_pitch_samples_deg']])]:
    for pair in poses:
        q0,q1=pair;name=f'{kind}_{q0:+g}_{q1:+g}'.replace('+','p').replace('-','m');shapes=ankle_shapes(q0,q1) if kind=='ankle' else hand_shapes(q0)
        doc=A.newDocument(name);masses={};objs=[]
        for n,s in shapes.items():
            material=None;category='reference'
            if n in ('FixedFrame','RollCarrier','FootStructure','ForearmLoadFrame'):material='aluminum';category='load_structure'
            elif n=='RollRod':material='steel';category='drive_structure'
            elif n=='RollCranks':material='aluminum';category='drive_structure'
            elif n in ('SolePad','GauntletPrimaryPad'):category='contact_pad'
            elif 'Motor' in n:category='motor_envelope'
            o,m=add(doc,n,s,category,(.68,.7,.72) if category=='load_structure' else (.2,.45,.75) if category=='motor_envelope' else (.2,.2,.2),material);objs.append(o);masses[n]=m
            verts,tris=s.tessellate(.8);render.append({'pose':name,'kind':kind,'name':n,'points':[list(v) for v in verts],'triangles':tris,'category':category})
        doc.recompute();f=OUT/(name+'.FCStd');doc.saveAs(str(f));Part.export(objs,str(OUT/(name+'.step')));bounds=compound(list(shapes.values())).BoundBox
        checks=[];keys=list(shapes);ignore={tuple(sorted(x)) for x in [('FixedFrame','FixedBearings'),('FixedFrame','RollMotorVendor'),('FixedFrame','RollCarrier'),('FixedBearings','RollCarrier'),('FixedBearings','RollMotorVendor'),('RollCarrier','PitchMotorEnvelope'),('RollCarrier','RollCranks'),('RollMotorVendor','RollCranks'),('RollRod','RollCranks'),('FootStructure','SolePad'),('FootStructure','RollCarrier'),('ForearmLoadFrame','GauntletPrimaryPad')]}
        for i,k1 in enumerate(keys):
            for k2 in keys[i+1:]:
                if tuple(sorted((k1,k2))) not in ignore:
                    common=shapes[k1].common(shapes[k2]).Volume
                    if common>1e-5:checks.append({'a':k1,'b':k2,'intersection_mm3':common})
        records.append({'pose':name,'kind':kind,'q_deg':pair,'bounds_mm':[bounds.XMin,bounds.YMin,bounds.ZMin,bounds.XMax,bounds.YMax,bounds.ZMax],'unexpected_intersections':checks,'mass_from_geometry_kg':masses,'files':[f.name,name+'.step'],'manufacturable':False})
        A.closeDocument(doc.Name);d=A.openDocument(str(f));assert all(o.Shape.isValid() for o in d.Objects if hasattr(o,'Shape'));A.closeDocument(d.Name)
neutral=next(r for r in records if r['pose']=='ankle_p0_p0');half=cfg['ankle']['foot_center_spacing_mm']/2
# Mirror asymmetric frame protrusions toward the centerline for the bilateral neutral reference.
bilateral={'left_y_mm':[neutral['bounds_mm'][1]+half,neutral['bounds_mm'][4]+half],'right_y_mm':[-neutral['bounds_mm'][4]-half,-neutral['bounds_mm'][1]-half]}
bilateral['inside_gap_mm']=bilateral['left_y_mm'][0]-bilateral['right_y_mm'][1];bilateral['outside_width_mm']=bilateral['left_y_mm'][1]-bilateral['right_y_mm'][0]
summary={'version':'v0.16','input_sha256':hashlib.sha256(PFILE.read_bytes()).hexdigest(),'freecad_version':A.Version(),'records':records,'neutral_ankle_structure_mass_kg':sum(v for v in neutral['mass_from_geometry_kg'].values() if v is not None),'neutral_forearm_support_structure_mass_kg':next(sum(v for v in r['mass_from_geometry_kg'].values() if v is not None) for r in records if r['pose']=='hand_p0_p0'),'bilateral_neutral_bounds':bilateral,'clearance_to_nominal_knee_axis_z_mm':cfg['ankle']['nominal_knee_axis_z_mm']-neutral['bounds_mm'][5],'motor_masses_excluded_from_shape_volume':{'XM430_roll_kg':.082,'AK40_pitch_kg':.190},'unmodeled_mass':['bearings','shafts','horns','rod ends','fasteners','cables','rubber pads','PETG costume_shell'],'scope':'Local schematic solids; no vendor interface release, continuous sweep, strength, fatigue or wholebody integration.'}
(OUT/'mechanism_results.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8');(OUT/'render_mesh.json').write_text(json.dumps(render)+'\n')
print('records',len(records),'unexpected intersections',sum(len(r['unexpected_intersections']) for r in records),'ankle structure kg',summary['neutral_ankle_structure_mass_kg'],'forearm support structure kg',summary['neutral_forearm_support_structure_mass_kg'])

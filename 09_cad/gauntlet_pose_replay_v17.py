"""Replay solved pad poses using the saved v16 hand solids, not just an IK point."""
import FreeCAD as A, Part, json, math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/gauntlet_v17';OUT.mkdir(exist_ok=True)
data=json.loads((ROOT/'04_kinematics/gauntlet_contact_v17.json').read_text())
doc=A.openDocument(str(ROOT/'09_cad/support_mechanisms_v16/hand_p0_p0.FCStd'));source={o.Name:o.Shape.copy() for o in doc.Objects if hasattr(o,'Shape')};A.closeDocument(doc.Name)
def ro(axis,deg):return A.Rotation(A.Vector(*axis),deg)
V=A.Vector
chosen=[min([r for r in data['records'] if r['architecture_dof']==26],key=lambda r:r['position_error_mm']+.5*r['normal_error_deg'])]
chosen += [r for r in data['records'] if r['finite_search_pose_found']]
records=[];render=[]
for i,row in enumerate(chosen):
    q=row['upstream_q_deg'];Rb=ro((0,1,0),row['body_pitch_deg']);Rf=Rb.multiply(ro((0,1,0),q[0])).multiply(ro((1,0,0),q[1]))
    if row['architecture_dof']==28:Rf=Rf.multiply(ro((0,0,1),q[2])).multiply(ro((0,1,0),q[3]))
    else:Rf=Rf.multiply(ro((0,1,0),q[2]))
    # C is the proper rotation taking v16 +X -> arm -Z, +Y -> arm +Y.
    C=ro((0,1,0),90);mount=ro((1,0,0),row['mount_roll_deg']);W=V(*row['nodes_mm']['wrist']);R=Rf.multiply(C).multiply(mount)
    T=A.Placement(W-R.multVec(V(0,0,25)),R)
    name=f'contact_{row["architecture_dof"]}_{i}';doc=A.newDocument(name);objects=[];heights={}
    for n,shape in source.items():
        s=shape.copy();s.Placement=T.multiply(s.Placement)
        o=doc.addObject('PartDesign::Feature',n);o.Shape=s;objects.append(o);heights[n]=s.BoundBox.ZMin
        pts,tris=s.tessellate(1);render.append({'pose':name,'name':n,'points':[list(v) for v in pts],'triangles':tris})
    S=V(*row['nodes_mm']['shoulder']);E=V(*row['nodes_mm']['elbow']);delta=E-S
    link=Part.makeCylinder(7,delta.Length,S,delta.normalize());o=doc.addObject('PartDesign::Feature','UpperArmGeometry');o.Shape=link;objects.append(o)
    pts,tris=link.tessellate(1);render.append({'pose':name,'name':'UpperArmGeometry','points':[list(v) for v in pts],'triangles':tris})
    expected=V(*row['contact_mm']);actual=T.multVec(V(-12,0,0));error=(actual-expected).Length;assert error<1e-5
    cornerz=[T.multVec(V(-12+dx,dy,0)).z for dx in (-10,10) for dy in (-20,20)]
    doc.recompute();f=OUT/(name+'.FCStd');doc.saveAs(str(f));Part.export(objects,str(OUT/(name+'.step')));A.closeDocument(doc.Name)
    records.append({'name':name,'IK':row,'pad_CAD_to_IK_error_mm':error,'pad_corner_height_mm':cornerz,'component_minimum_ground_z_mm':heights,
                    'has_ground_penetration_over_0_5mm':min(heights.values())<-.5,'wholebody_verified':False})
(OUT/'replay_results.json').write_text(json.dumps({'records':records,'scope':'Real saved hand shape transform; arm drive, torso and legs not collision checked'},indent=2)+'\n')
(OUT/'render_mesh.json').write_text(json.dumps(render)+'\n')
print([(r['name'],r['pad_CAD_to_IK_error_mm'],r['has_ground_penetration_over_0_5mm']) for r in records])

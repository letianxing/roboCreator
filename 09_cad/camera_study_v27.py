"""Catalogue-envelope and nominal optical-frustum study, NOT camera/neck validation."""
import json,math,hashlib
from pathlib import Path
import FreeCAD as A
import Part
import numpy as np
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'09_cad/camera_v27';OUT.mkdir(exist_ok=True)
CP=ROOT/'08_electrical/camera_candidates_v27.json';PF=ROOT/'03_system/camera_study_v27.json'
C=json.loads(CP.read_text());P=json.loads(PF.read_text());V=A.Vector

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def bounds_gap(a,b):
    return math.sqrt(sum(max(0,l2-h1,l1-h2)**2 for l1,h1,l2,h2 in [(a.XMin,a.XMax,b.XMin,b.XMax),(a.YMin,a.YMax,b.YMin,b.YMax),(a.ZMin,a.ZMax,b.ZMin,b.ZMax)]))
def pose(camera,x,pitch):
    center=V(x,0,P['body_origin_height_stand_mm']+135);R=A.Rotation(V(0,1,0),pitch)
    d,w,h=camera['xyz_mm'];shape=Part.makeBox(d,w,h,V(-d/2,-w/2,-h/2));shape.Placement=A.Placement(center,R)
    origin=center+R.multVec(V(d/2,0,0))
    return shape,center,R,origin
def visible(point,origin,R,fov):
    local=R.inverted().multVec(V(*point)-origin);z=local.x
    ok=z>0 and abs(local.y)<=z*math.tan(math.radians(fov[0]/2)) and abs(local.z)<=z*math.tan(math.radians(fov[1]/2))
    return {'within_nominal_fov':bool(ok),'axial_Z_mm':z,'local_forward_left_up_mm':list(local)}
def obstruction(origin,point,obstacles):
    end=V(*point);start=origin+(end-origin).normalize()*.1
    ray=Part.makeLine(start,end);bb=ray.BoundBox;hits=[]
    for n,s in obstacles.items():
        if bounds_gap(bb,s.BoundBox)>1e-6:continue
        common=ray.common(s)
        if not common.isNull() and common.Length>1e-5:hits.append({'object':n,'intersected_length_mm':common.Length})
    return hits
def frustum(origin,R,fov,near=100,far=600):
    planes=[]
    for distance in [near,far]:
        hw=distance*math.tan(math.radians(fov[0]/2));hh=distance*math.tan(math.radians(fov[1]/2))
        planes.append([origin+R.multVec(V(distance,y,z)) for y,z in [(-hw,-hh),(hw,-hh),(hw,hh),(-hw,hh)]])
    edges=[]
    for corners in planes:
        for i in range(4):edges.append(Part.makeLine(corners[i],corners[(i+1)%4]))
    edges.extend(Part.makeLine(a,b) for a,b in zip(*planes))
    return Part.makeCompound(edges)
def save_case(name,parts,camera_shape,origin,R,fov):
    doc=A.newDocument(name)
    for n,s in {**parts,'camera':camera_shape,'NominalDepthFrustum_PROBE':frustum(origin,R,fov)}.items():
        o=doc.addObject('PartDesign::Feature',n);o.Shape=s;o.addProperty('App::PropertyString','Scope')
        o.Scope='Catalogue box and assumed optical center; not exact hardware, implemented neck motion or manufacturing release'
    doc.recompute();path=OUT/(name+'.FCStd');doc.saveAs(str(path));A.closeDocument(doc.Name)
    d=A.openDocument(str(path));assert len(d.Objects)==119 and all(o.Shape.isValid() for o in d.Objects);A.closeDocument(d.Name)
    return str(path.relative_to(ROOT))

def main():
    src=ROOT/'09_cad/step_repair_v26/repaired/stand.FCStd';original_hash=sha(src)
    doc=A.openDocument(str(src));allparts={o.Name:o.Shape.copy() for o in doc.Objects};A.closeDocument(doc.Name)
    oldcam=allparts.pop('camera');oldcenter=V(*[25,0,P['body_origin_height_stand_mm']+135])
    assert (oldcam.Solids[0].CenterOfMass-oldcenter).Length<1e-6
    # Nonmaterial service/optics probes must not masquerade as opaque robot parts.
    obstacles={n:s for n,s in allparts.items() if 'PROBE' not in n and 'Keepout' not in n and 'ServiceSpace' not in n}
    assembly=[];optical=[];massrows=[]
    baseline=json.loads((ROOT/'09_cad/uac_mount_v25_service/integration.json').read_text())
    standbase=next(r for r in baseline['mass_sensitivity'] if r['scene']=='stand' and r['extra_board_wire_assumption_kg']==0)
    for camera in C['candidates']:
        for x in P['candidate_center_x_mm']:
            for pitch in P['camera_down_pitch_deg']:
                shape,center,R,origin=pose(camera,x,pitch);collisions=[];near=[]
                for n,s in allparts.items():
                    if bounds_gap(shape.BoundBox,s.BoundBox)>2:continue
                    d=shape.distToShape(s)[0];volume=shape.common(s).Volume if d<1e-7 else 0
                    near.append({'object':n,'distance_mm':d,'overlap_mm3':volume})
                    if volume>1e-5:collisions.append(n)
                case={'candidate':camera['id'],'center_body_mm':[x,0,135],'pitch_deg':pitch,'collisions':collisions,'near_pairs':near,
                      'box_geometry_pass':not collisions,'optical_origin_assumed_world_mm':list(origin),'neck_linkage_verified':False}
                if camera['id'] in ('D436','Gemini335') and (x,pitch) in [(25,0),(40,45)]:
                    case['study_CAD']=save_case(camera['id']+'_x'+str(x)+'_p'+str(pitch),allparts,shape,origin,R,camera['depth_fov_deg'])
                assembly.append(case)
                # Occlusion is evaluated only for the current position and one forward study position.
                ray_enabled=(x,pitch) in [(25,0),(40,45)]
                for mode in camera['modes']:
                    fov=mode.get('depth_fov_deg',camera['depth_fov_deg'])
                    for name,point in P['task_probe_points_mm'].items():
                        depth=visible(point,origin,R,fov);rgb=visible(point,origin,R,camera['rgb_fov_deg'])
                        hits=obstruction(origin,point,obstacles) if ray_enabled and (depth['within_nominal_fov'] or rgb['within_nominal_fov']) else None
                        z=depth['axial_Z_mm']
                        optical.append({'candidate':camera['id'],'x_body_mm':x,'pitch_deg':pitch,'mode':mode['resolution'],'mode_inferred':mode.get('inferred',False),
                            'task':name,'depth':depth,'rgb':rgb,'above_mode_minZ':z>=mode['min_z_mm'],'within_ideal_Z':camera['ideal_depth_mm'][0]<=z<=camera['ideal_depth_mm'][1],
                            'below_ideal_maxZ':z<=camera['ideal_depth_mm'][1],'ray_hits':hits,
                            'nominal_depth_rgb_and_minZ':depth['within_nominal_fov'] and rgb['within_nominal_fov'] and z>=mode['min_z_mm'] and z<=camera['ideal_depth_mm'][1],
                            'measured_visibility_verified':False})
                masses=[camera['mass_kg']] if camera['mass_kg'] is not None else P['D436_body_mass_probes_kg']
                for m in masses:
                    for extra in P['accessory_mass_assumptions_kg']:
                        total=standbase['mass_with_CAD_mount_kg']-P['old_camera_mass_allowance_kg']+m+extra
                        com=(V(*standbase['COM_mm'])*standbase['mass_with_CAD_mount_kg']-oldcenter*P['old_camera_mass_allowance_kg']+center*(m+extra))/total
                        massrows.append({'candidate':camera['id'],'x_body_mm':x,'pitch_deg':pitch,'body_mass_kg':m,'body_mass_is_probe':camera['mass_kg'] is None,
                            'accessory_assumption_kg':extra,'modeled_total_mass_kg':total,'COM_mm':list(com),
                            'neck_pitch_gravity_moment_probe_Nm':(m+extra)*9.81*abs(center.x)/1000,
                            'moment_reference':'ASSUMED pitch axis at body X0; actual neck frames/drive unknown'})
        print('Studied',camera['id'],flush=True)
    assert sha(src)==original_hash
    result={'complete':True,'candidate_input_sha256':sha(CP),'task_input_sha256':sha(PF),'source_CAD_sha256':original_hash,
        'assembly_cases':assembly,'optical_cases':optical,'mass_cases':massrows,
        'scope':'6 candidates x2 mounting X positions x4 sensor-center pitch probes; standing geometry only. Nominal single-center FOV, not rectified stereo coverage.',
        'limitations':['Nominal housings only, not exact STEP or plug/tolerance envelopes','Sensor pitch is NOT a validated neck motion',
            'Optical origins approximate: actual separate RGB/left/right pupils and stereo invalid band not modeled','Point probes do not establish full bottle, hands or small-part visibility',
            'Ray tests only two configurations/candidate; transparent/reflection/depth noise not modeled','No hardware, SDK or camera procurement approval']}
    (OUT/'study.json').write_text(json.dumps(result,indent=2)+'\n')
    print('Cases',len(assembly),len(optical),len(massrows),'box passes',sum(r['box_geometry_pass'] for r in assembly),flush=True)
if __name__=='__main__':main()

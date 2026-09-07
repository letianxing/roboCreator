"""Constant-length free-end circular flex-segment proxy, not an approved USB cable."""
import json,math,sys,argparse
from pathlib import Path
from neck_chain_v28 import A,Part,V,ROOT,placed,around,audit,FIXED_BOUNDS,INTERNAL_PAIRS
from camera_study_v27 import visible
OUT=ROOT/'09_cad/neck_hardware_v28_final'
def arc_proxy(a,b,length=70,radius=2):
    chord=b-a;d=chord.Length;assert 0<d<length
    lo,hi=1e-8,2*math.pi-1e-8
    for _ in range(70):
        theta=(lo+hi)/2
        if 2*math.sin(theta/2)/theta>d/length:lo=theta
        else:hi=theta
    theta=(lo+hi)/2;R=length/theta
    direction=chord/d;pref=V(0,-1,0);normal=(pref-direction*pref.dot(direction)).normalize()
    midpoint=(a+b)/2+normal*(R*(1-math.cos(theta/2)))
    edge=Part.Arc(a,midpoint,b).toShape();wire=Part.Wire([edge])
    section=Part.Wire([Part.makeCircle(radius,a,edge.tangentAt(edge.FirstParameter))]);shape=wire.makePipeShell([section],True,False)
    assert shape.isValid() and abs(wire.Length-length)<1e-6
    return shape,{'centerline_length_mm':wire.Length,'bend_radius_mm':R,'angle_rad':theta,'chord_mm':d,'midpoint_world_mm':list(midpoint)}
def center(s):
    return sum((v.CenterOfMass*v.Volume for v in s.Solids),V())/sum(v.Volume for v in s.Solids)
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--compact-loop',action='store_true');args=parser.parse_args()
    cable_length=38 if args.compact_loop else 70
    fixed_anchor=[12.5,-5,110] if args.compact_loop else [20,-5,110]
    head_anchor=[12.5,-5,145] if args.compact_loop else [25,-5,145]
    study=json.loads((OUT/'study.json').read_text());assert study['complete']
    doc=A.openDocument(str(OUT/'y0_p0.FCStd'));neutral={o.Name:o.Shape.copy() for o in doc.Objects};A.closeDocument(doc.Name)
    T=A.Placement(V(0,0,214.26539554219744),A.Rotation());local={n:placed(s,T.inverse()) for n,s in neutral.items()}
    base=json.loads((ROOT/'09_cad/step_repair_v26/verification.json').read_text());b=next(s for s in base['fresh_load_results'] if s['scene']=='stand')
    src=A.openDocument(str(ROOT/'09_cad/step_repair_v26/repaired/stand.FCStd'));original={o.Name:o.Shape.copy() for o in src.Objects};A.closeDocument(src.Name)
    masses={'WukongHeadShell':.11,'CrownOccupancy':.02,'camera':.09,'NeckMotor0':.023,'NeckMotor1':.023}
    task=json.loads((ROOT/'03_system/camera_study_v27.json').read_text())['task_probe_points_mm']
    rows=[];FIXED_BOUNDS.clear();INTERNAL_PAIRS.clear()
    for case in study['cases']:
        yaw,pitch=case['yaw_deg'],case['pitch_deg'];Y=around(study['yaw_origin_body_mm'],[0,0,1],yaw);H=Y.multiply(around(study['pitch_origin_yaw_neutral_body_mm'],[0,1,0],pitch))
        parts=dict(neutral)
        for n in ['WukongHeadShell','CrownOccupancy','camera']:parts[n]=placed(local[n],T.multiply(H))
        parts['NeckMotor1']=placed(local['NeckMotor1'],T.multiply(Y))
        a=T.multVec(V(*fixed_anchor));end=T.multVec(H.multVec(V(*head_anchor)))
        cable,arc=arc_proxy(a,end,length=cable_length);parts['NeckCable_PROBE']=cable
        # Existing moving head/motor bounds must not be cached as fixed across cable poses.
        for changed in ['WukongHeadShell','CrownOccupancy','camera','NeckMotor1']:FIXED_BOUNDS.pop(changed,None)
        bad=audit(parts,{'NeckCable_PROBE'})
        cc=study['camera_center_neutral_body_mm'];origin=T.multVec(H.multVec(V(cc[0]+12.5,cc[1],cc[2])));R=H.Rotation
        visibility={}
        for n,p in task.items():
            dep=visible(p,origin,R,[87,58]);rgb=visible(p,origin,R,[90,65]);z=dep['axial_Z_mm']
            visibility[n]={'depth_nominal':dep['within_nominal_fov'],'rgb_nominal':rgb['within_nominal_fov'],'axial_Z_mm':z,'inferred_848_MinZ_pass':z>=195,'inferred_640_MinZ_pass':z>=150}
        moment=V(*b['modeled_COM_mm_after'])*b['modeled_mass_kg_after']
        for n,m in masses.items():moment+=(center(parts[n])-center(original[n]))*m
        com=moment/b['modeled_mass_kg_after'];pivot=T.multVec(Y.multVec(V(*study['pitch_origin_yaw_neutral_body_mm'])));axis=Y.Rotation.multVec(V(0,1,0));tau=0
        for n in ['WukongHeadShell','CrownOccupancy','camera']:tau+=(center(parts[n])-pivot).cross(V(0,0,-9.81*masses[n])).dot(axis)/1000
        bounds=Part.makeCompound(list(parts.values())).cleaned().BoundBox
        row={**case,'cable':arc,'cable_collisions':bad,'combined_geometry_pass':case['pass'] and not bad,'visibility_probes':visibility,
             'assembly_envelope_xyz_mm':[bounds.XLength,bounds.YLength,bounds.ZLength],'highest_world_Z_mm':bounds.ZMax,
             'modeled_mass_kg_unchanged_allowances':b['modeled_mass_kg_after'],'COM_mm':list(com),'head_pitch_gravity_torque_Nm':tau}
        rows.append(row);print('cable',yaw,pitch,'R',round(arc['bend_radius_mm'],2),'hits',[(r['a'],r['b']) for r in bad],flush=True)
        if (yaw,pitch) in [(0,0),(0,45),(0,50),(30,45)]:
            doc=A.newDocument('cable_neck');mesh=[]
            for n,s in parts.items():
                o=doc.addObject('PartDesign::Feature',n);o.Shape=s;o.addProperty('App::PropertyString','Scope');o.Scope='v28 proposed joint chain and constant-length FREE-END cable proxy; no hardware release'
                vis=s
                if 'Motor' in n:
                    bb=s.BoundBox;vis=Part.makeBox(bb.XLength,bb.YLength,bb.ZLength,V(bb.XMin,bb.YMin,bb.ZMin))
                p,f=vis.tessellate(1);mesh.append({'name':n,'points':[list(v) for v in p],'triangles':f})
            doc.recompute();name=('compact_cable_y' if args.compact_loop else 'cable_y')+str(yaw)+'_p'+str(pitch);doc.saveAs(str(OUT/(name+'.FCStd')));A.closeDocument(doc.Name)
            (OUT/(name+'_mesh.json')).write_text(json.dumps(mesh)+'\n')
    result={'complete':True,'cases':rows,'cable_length_mm':cable_length,'bundle_radius_mm':2,'anchor_fixed_body_mm':fixed_anchor,'anchor_head_neutral_body_mm':head_anchor,
            'scope':'Only neck flex segment with free end tangents; actual USB leads/connectors/clamps/torsion/material radius unknown. Mass allowances retained, even after shell opening; extra brackets,idler,cable mass still missing. Point FOV is nominal, not stereo/occlusion/hardware validation.'}
    (OUT/('cable_and_loads_compact.json' if args.compact_loop else 'cable_and_loads.json')).write_text(json.dumps(result,indent=2)+'\n')
if __name__=='__main__':main()
